#!/usr/bin/env python3
"""Polls MediaMTX's HTTP API for reader counts on the camera RTSP paths and
enables/disables the matching Gazebo camera sensor by publishing to its
`enable_topic`.

Why this exists (see gazebo/gazebo-sitl-plan.html, blocker B2): the original
design gated rendering with MediaMTX's `runOnDemand` hook firing a
`gz topic ...` command. That cannot work -- `bluenviron/mediamtx` is built
FROM scratch (no shell, no `gz` CLI) and execs hooks directly with no
/bin/sh, so the hook fails with "exec: file not found". This inverts the
trigger instead: the gazebo container -- which DOES have Gazebo installed --
polls MediaMTX's read-only `/v3/paths/get/<name>` endpoint on an interval and
toggles its own cameras. No exec inside mediamtx, no custom mediaserver
image, no changes to the camera bridge.

Two hard-won design points (both measured against the running stack):

- **Persistent gz-transport publishers, and the desired state is
  re-published every tick, not only on transitions.** The first version
  spawned a fresh `gz topic -p ...` process per transition; a just-started
  publisher has to discover the subscriber first, and under reader-driven
  flapping (attach, gate switches, reader drops, reattach -- every few
  seconds) individual messages were simply lost: a camera stayed streaming
  for minutes after its `false`, and a 43-second reader window produced no
  frames because its `true` never landed. Long-lived publishers remove the
  per-message discovery race, and re-publishing the current desired state
  each tick means one lost/late message heals on the next tick instead of
  wedging the camera in the wrong state until the next transition. The
  plugin side is idempotent by design (see the B1 patch's
  ReconcileStreamingState), so repeats are free.

- **A disable hold-off (GZ_CAMERA_GATE_DISABLE_HOLD).** The reader that
  triggers the camera bridge's placeholder->RTP switch is dropped *by* that
  switch (the placeholder publisher it was attached to is killed), and a
  real GCS also drops/reattaches on brief network hiccups. Disabling the
  instant readers hit zero therefore tears the camera down right in the
  middle of every legitimate switch. Zero readers must persist for the
  hold-off before the camera is disabled; enabling stays immediate. This is
  the moral equivalent of the plan's original MediaMTX
  `runOnDemandCloseAfter: 10s`.

Runs as a background process inside the gazebo container, alongside
`gz sim` (see entrypoint.sh). It is deliberately decoupled from `gz sim`'s
lifecycle: if this process dies, the sim keeps running (cameras just stop
being gated -- entrypoint.sh logs that loudly); if `gz sim` exits, Docker
tears down the whole container regardless of whether this process is still
alive.

Config (env vars, all optional):
  GZ_CAMERA_GATE_MEDIASERVER_HOST  default: "<NET_PREFIX>.0.11" (mediaserver's
                                    static compose address; NET_PREFIX
                                    defaults to "172.28" same as compose)
  GZ_CAMERA_GATE_MEDIASERVER_PORT  default: "9997" (MediaMTX's MTX_API port)
  GZ_CAMERA_GATE_INTERVAL          default: "2" (seconds between polls)
  GZ_CAMERA_GATE_TIMEOUT           default: "2" (seconds, per HTTP request)
  GZ_CAMERA_GATE_DISABLE_HOLD      default: "10" (seconds of continuous zero
                                    readers before a camera is disabled;
                                    enable is immediate)
  GZ_CAMERA_GATE_PATHS             default: "cam1:/cam1/enable,cam2:/cam2/enable"
                                    comma-separated "<mediamtx path>:<gz enable
                                    topic>" pairs -- matches the paths cam1/
                                    gzcam1 and cam2/gzcam2 publish to (see
                                    docker-compose.yaml) and the enable_topic
                                    values baked into
                                    models/iris_dual_cam/model.sdf.template.
"""
from __future__ import annotations

import json
import os
import signal
import subprocess
import sys
import time
import urllib.error
import urllib.request

_RUNNING = True


def _handle_shutdown(_signum, _frame) -> None:
    global _RUNNING
    _RUNNING = False


def _log(msg: str) -> None:
    ts = time.strftime("%Y-%m-%dT%H:%M:%S", time.gmtime())
    print(f"[camera_gate] {ts}Z {msg}", file=sys.stderr, flush=True)


def _env_float(name: str, default: float) -> float:
    raw = os.environ.get(name)
    if not raw:
        return default
    try:
        return float(raw)
    except ValueError:
        _log(f"WARNING: {name}={raw!r} is not a number, using default {default}")
        return default


def _parse_paths(raw: str) -> dict[str, str]:
    """'cam1:/cam1/enable,cam2:/cam2/enable' -> {'cam1': '/cam1/enable', ...}"""
    mapping: dict[str, str] = {}
    for item in raw.split(","):
        item = item.strip()
        if not item:
            continue
        path, _, topic = item.partition(":")
        path, topic = path.strip(), topic.strip()
        if not path or not topic:
            _log(f"WARNING: skipping malformed GZ_CAMERA_GATE_PATHS entry {item!r}")
            continue
        mapping[path] = topic
    return mapping


def _default_mediaserver_host() -> str:
    # Matches the compose contract: mediaserver is always "<NET_PREFIX>.0.11"
    # (docker-compose.yaml), and the gazebo service already receives
    # NET_PREFIX so instances at a non-default prefix resolve correctly
    # without a second variable to keep in sync.
    return f"{os.environ.get('NET_PREFIX', '172.28')}.0.11"


class _BooleanPublisher:
    """One persistent gz-transport publisher per enable topic, with a
    subprocess `gz topic` fallback if the python bindings are ever missing
    from the image (they arrive via python3-gz-transport13; see the
    Dockerfile). The fallback keeps the gate functional but reintroduces
    the per-message discovery race the bindings exist to avoid -- the
    startup log says which mode is active so that's never a silent
    downgrade.
    """

    _node = None  # class-level: one gz-transport Node shared by all topics

    def __init__(self, topic: str) -> None:
        self.topic = topic
        self._pub = None
        self._Boolean = None
        cls = type(self)
        try:
            from gz.msgs10.boolean_pb2 import Boolean
            from gz.transport13 import Node
        except ImportError:
            return
        if cls._node is None:
            cls._node = Node()
        self._Boolean = Boolean
        self._pub = cls._node.advertise(topic, Boolean)

    @property
    def native(self) -> bool:
        return self._pub is not None

    def publish(self, enabled: bool) -> bool:
        if self._pub is not None:
            msg = self._Boolean()
            msg.data = enabled
            try:
                return bool(self._pub.publish(msg))
            except Exception as exc:  # bindings raise plain RuntimeError
                _log(f"ERROR: publish {self.topic} = {enabled} failed: {exc}")
                return False
        value = "true" if enabled else "false"
        cmd = ["gz", "topic", "-t", self.topic, "-m", "gz.msgs.Boolean",
               "-p", f"data: {value}"]
        try:
            subprocess.run(cmd, check=True, capture_output=True, timeout=5, text=True)
            return True
        except (subprocess.CalledProcessError, subprocess.TimeoutExpired, OSError) as exc:
            _log(f"ERROR: failed to publish {self.topic} = {value}: {exc}")
            return False


def reader_count(host: str, port: str, path: str, timeout: float) -> int:
    """Reader count for `path`. Returns 0 for both "zero readers" and "path
    not published yet" (MediaMTX's error response for an unknown path) --
    both mean "nobody is watching, camera should be off". Raises on anything
    that means "could not get a real answer" (mediaserver down, bad
    response), which the caller treats as "unknown, don't act."
    """
    url = f"http://{host}:{port}/v3/paths/get/{path}"
    try:
        with urllib.request.urlopen(url, timeout=timeout) as resp:
            body = resp.read()
    except urllib.error.HTTPError as exc:
        # MediaMTX has been observed returning the "path not found" error
        # both as a non-2xx status and as a 200 with a status:error body
        # (version-dependent) -- handle both the same way below.
        body = exc.read()

    data = json.loads(body)
    if isinstance(data, dict) and data.get("status") == "error":
        return 0
    readers = data.get("readers")
    if readers is None:
        raise ValueError(f"unexpected response shape for path {path!r}: {data!r}")
    return len(readers)


def _sleep_interruptible(seconds: float) -> None:
    # time.sleep() only returns early on a signal if the handler raises; ours
    # doesn't (it just flips a flag), so PEP 475 would otherwise auto-resume
    # the full sleep. Chunk it instead so a shutdown signal is noticed
    # promptly rather than after up to one whole poll interval.
    deadline = time.monotonic() + seconds
    while _RUNNING and time.monotonic() < deadline:
        time.sleep(min(0.5, max(0.0, deadline - time.monotonic())))


def main() -> int:
    signal.signal(signal.SIGTERM, _handle_shutdown)
    signal.signal(signal.SIGINT, _handle_shutdown)

    host = os.environ.get("GZ_CAMERA_GATE_MEDIASERVER_HOST") or _default_mediaserver_host()
    port = os.environ.get("GZ_CAMERA_GATE_MEDIASERVER_PORT", "9997")
    interval = _env_float("GZ_CAMERA_GATE_INTERVAL", 2.0)
    timeout = _env_float("GZ_CAMERA_GATE_TIMEOUT", 2.0)
    disable_hold = _env_float("GZ_CAMERA_GATE_DISABLE_HOLD", 10.0)
    cameras = _parse_paths(
        os.environ.get("GZ_CAMERA_GATE_PATHS", "cam1:/cam1/enable,cam2:/cam2/enable")
    )

    if not cameras:
        _log("no cameras configured (GZ_CAMERA_GATE_PATHS empty) -- nothing to gate, exiting")
        return 0

    publishers = {path: _BooleanPublisher(topic) for path, topic in cameras.items()}
    mode = "gz-transport bindings" if all(p.native for p in publishers.values()) \
        else "gz CLI fallback (per-message discovery race possible)"
    _log(
        f"polling http://{host}:{port} every {interval}s "
        f"(disable hold-off {disable_hold}s, publishing via {mode}) for: "
        + ", ".join(f"{p} -> {t}" for p, t in cameras.items())
    )

    # Desired state per path, re-published every tick (the plugin side is
    # idempotent, and repeats heal any lost/late message within one tick).
    desired: dict[str, bool] = dict.fromkeys(cameras, False)
    # Monotonic timestamp of the last poll that saw >0 readers, for the
    # disable hold-off. Initialized to 0 == "never".
    last_readers: dict[str, float] = dict.fromkeys(cameras, 0.0)
    mediaserver_reachable = True  # avoids a spurious "reachable again" log on startup

    while _RUNNING:
        for path, topic in cameras.items():
            try:
                count = reader_count(host, port, path, timeout)
            except (urllib.error.URLError, OSError, ValueError) as exc:
                if mediaserver_reachable:
                    _log(
                        f"WARNING: mediaserver unreachable at {host}:{port} ({exc}); "
                        "leaving cameras in their last known state until it comes back"
                    )
                    mediaserver_reachable = False
                continue

            if not mediaserver_reachable:
                _log(f"mediaserver reachable again at {host}:{port}")
                mediaserver_reachable = True

            now = time.monotonic()
            if count > 0:
                last_readers[path] = now
                want = True
            else:
                # Hold the camera on across brief reader gaps: the bridge's
                # own placeholder->RTP switch necessarily drops the reader
                # that triggered it, and a real GCS reconnects after
                # hiccups. Only a sustained zero-reader stretch disables.
                want = desired[path] and (now - last_readers[path]) < disable_hold

            if want != desired[path]:
                _log(
                    f"{path}: {count} reader(s) -> {topic} = "
                    f"{'true' if want else 'false'}"
                )
                desired[path] = want

            # Re-publish current desired state every tick (idempotent on
            # the plugin side; heals lost/late messages within one tick).
            publishers[path].publish(desired[path])

        _sleep_interruptible(interval)

    _log("received shutdown signal, exiting")
    return 0


if __name__ == "__main__":
    sys.exit(main())
