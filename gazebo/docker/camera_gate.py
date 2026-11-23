#!/usr/bin/env python3
"""Serves each Gazebo camera as an on-demand RTSP mount and renders a camera
only while its mount has a client.

Each GstCameraPlugin sensor sends bare RTP/H.264 to a UDP port on 127.0.0.1
inside this container. This process runs a GstRtspServer (default :8554) with
one shared mount per camera that reads that port, and drives the camera's
`enable_topic` from the mount's media lifecycle:

- media created (a client sent DESCRIBE)  -> enable the camera immediately
- media torn down (last client left)      -> disable once no media has
                                             existed for DISABLE_HOLD seconds

In the compose stack the only client is dfvideo-gazebo (source: "rtsp"), and
dfvideo itself only opens its upstream while a GCS is watching. So "a GCS opens
/cam1 on dfvideo" is what turns the Gazebo camera on, with no extra signal.

History: this used to poll MediaMTX's HTTP API for reader counts. That needed a
bootstrap placeholder publisher in the gzcam bridge, because MediaMTX never
reports a reader on a path that has no publisher, while the camera would not
publish until it had a reader. With the server in this process, the mount
exists whether or not the camera is rendering. Enabling at media creation, i.e.
before the media prerolls, lets the first DESCRIBE simply wait for Gazebo's
first frame instead of deadlocking.

Two hard-won design points are kept from the polling version, both measured
against the running stack:

- **Persistent gz-transport publishers, and the desired state is
  re-published every tick, not only on transitions.** A fresh `gz topic -p`
  per transition has to discover the subscriber first, and individual
  messages were simply lost under flapping. Long-lived publishers remove that
  race, and re-publishing every tick heals a lost or late message on the next
  tick. The plugin side is idempotent (the B1 patch's ReconcileStreamingState).
  The RTSP callbacks only record state; publishing happens on the GLib tick,
  never from inside a gz-transport callback (see gazebo/README.md, "Resolved
  the hard way").

- **A disable hold-off (GZ_CAMERA_GATE_DISABLE_HOLD).** Clients drop and
  reattach: dfvideo tears its upstream down when its own last viewer leaves,
  and a GCS reconnects after hiccups. Disabling the instant the media goes
  away would restart the camera on every reconnect. The camera must be
  without a client for the hold-off before it is disabled; enabling stays
  immediate.

Runs as a background process inside the gazebo container, alongside
`gz sim` (see entrypoint.sh). If this process dies, the sim keeps running but
camera video and the render gate are gone (entrypoint.sh logs that loudly).

Config (env vars, all optional):
  GZ_CAMERA_GATE_RTSP_PORT     default "8554": RTSP listen port (not published;
                               dfvideo-gazebo reaches it on the sim network)
  GZ_CAMERA_GATE_INTERVAL      default "2": seconds between re-publish ticks
  GZ_CAMERA_GATE_DISABLE_HOLD  default "10": seconds without a client before a
                               camera is disabled (enable is immediate)
  GZ_CAMERA_GATE_PATHS         default "cam1:/cam1/enable,cam2:/cam2/enable":
                               comma-separated "<rtsp mount>:<gz enable topic>"
                               pairs; the topics match the enable_topic values
                               in models/iris_dual_cam/model.sdf.template
  GZ_<MOUNT>_RTP_PORT          e.g. GZ_CAM1_RTP_PORT: the local UDP port that
                               mount reads, i.e. the camera's <udp_port>.
                               Required for every mount.
"""
from __future__ import annotations

import os
import signal
import subprocess
import sys
import time

import gi

gi.require_version("Gst", "1.0")
gi.require_version("GstRtspServer", "1.0")
from gi.repository import GLib, Gst, GstRtspServer  # noqa: E402

# A client that vanishes without TEARDOWN (e.g. dfvideo killed) keeps its
# session -- and therefore the camera -- until the session times out. The
# library default is 60 s; rtspsrc keeps a live session alive with periodic
# keep-alives well inside this.
_SESSION_TIMEOUT_S = 15


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
        path, topic = path.strip().strip("/"), topic.strip()
        if not path or not topic:
            _log(f"WARNING: skipping malformed GZ_CAMERA_GATE_PATHS entry {item!r}")
            continue
        mapping[path] = topic
    return mapping


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


class _Camera:
    """Media lifecycle -> desired enable state for one mount."""

    def __init__(self, path: str, topic: str, rtp_port: int, hold: float) -> None:
        self.path = path
        self.topic = topic
        self.rtp_port = rtp_port
        self.hold = hold
        self.publisher = _BooleanPublisher(topic)
        self.medias = 0          # live GstRTSPMedia objects for this mount
        self.last_media = 0.0    # monotonic time a media last existed; 0 = never
        self.desired = False

    def launch(self) -> str:
        # Re-payload rather than forward: h264parse makes the SPS/PPS from the
        # camera's in-band stream (x264enc key-int-max=10 in GstCameraPlugin)
        # available to the payloader, so the SDP carries sprop-parameter-sets
        # and a client joining mid-stream gets them with every IDR.
        caps = ("application/x-rtp,media=video,clock-rate=90000,"
                "encoding-name=H264,payload=96")
        return (f"( udpsrc address=127.0.0.1 port={self.rtp_port} caps=\"{caps}\" "
                "! rtph264depay ! h264parse "
                "! rtph264pay name=pay0 pt=96 config-interval=-1 )")

    def evaluate(self, now: float) -> bool:
        """Recompute the desired state; returns True when it changed."""
        if self.medias > 0:
            self.last_media = now
            want = True
        else:
            want = self.desired and (now - self.last_media) < self.hold
        changed = want != self.desired
        self.desired = want
        return changed


def main() -> int:
    Gst.init(None)

    port = os.environ.get("GZ_CAMERA_GATE_RTSP_PORT", "8554")
    interval = _env_float("GZ_CAMERA_GATE_INTERVAL", 2.0)
    disable_hold = _env_float("GZ_CAMERA_GATE_DISABLE_HOLD", 10.0)
    mapping = _parse_paths(
        os.environ.get("GZ_CAMERA_GATE_PATHS", "cam1:/cam1/enable,cam2:/cam2/enable")
    )
    if not mapping:
        _log("no cameras configured (GZ_CAMERA_GATE_PATHS empty) -- nothing to serve, exiting")
        return 0

    cameras: dict[str, _Camera] = {}
    for path, topic in mapping.items():
        env = f"GZ_{path.upper()}_RTP_PORT"
        raw = os.environ.get(env, "")
        if not raw.isdigit():
            _log(f"ERROR: mount {path!r} needs {env} (the camera's <udp_port>); got {raw!r}")
            return 1
        cameras[path] = _Camera(path, topic, int(raw), disable_hold)

    loop = GLib.MainLoop()

    def tick() -> bool:
        now = time.monotonic()
        for cam in cameras.values():
            if cam.evaluate(now):
                _log(f"/{cam.path}: {cam.medias} media -> {cam.topic} = "
                     f"{'true' if cam.desired else 'false'}")
            # Re-publish every tick (idempotent on the plugin side; heals a
            # lost or late message within one tick).
            cam.publisher.publish(cam.desired)
        return True

    def on_media_configure(_factory, media, cam: _Camera) -> None:
        cam.medias += 1
        # Enable now, not on the next tick: the DESCRIBE that created this
        # media is waiting for the camera's first frame. idle_add runs the
        # publish from the main loop, outside this signal emission.
        GLib.idle_add(lambda: tick() and False)

        def on_unprepared(_media) -> None:
            cam.medias = max(0, cam.medias - 1)
            cam.last_media = time.monotonic()

        media.connect("unprepared", on_unprepared)

    def on_client_connected(_server, client) -> None:
        def on_new_session(_client, session) -> None:
            session.set_timeout(_SESSION_TIMEOUT_S)

        client.connect("new-session", on_new_session)

    server = GstRtspServer.RTSPServer()
    server.set_address("0.0.0.0")
    server.set_service(port)
    server.connect("client-connected", on_client_connected)
    mounts = server.get_mount_points()
    for cam in cameras.values():
        factory = GstRtspServer.RTSPMediaFactory()
        factory.set_launch(cam.launch())
        # One media per mount, whatever the number of clients: one camera,
        # one UDP port.
        factory.set_shared(True)
        factory.connect("media-configure", on_media_configure, cam)
        mounts.add_factory(f"/{cam.path}", factory)
    if server.attach(None) == 0:
        _log(f"ERROR: could not listen on RTSP port {port}")
        return 1

    # Expire sessions of clients that vanished without TEARDOWN; the library
    # only does this when asked.
    pool = server.get_session_pool()
    GLib.timeout_add_seconds(2, lambda: pool.cleanup() >= 0)

    for sig in (signal.SIGTERM, signal.SIGINT):
        GLib.unix_signal_add(GLib.PRIORITY_DEFAULT, sig, lambda *_: loop.quit() or False)

    mode = "gz-transport bindings" if all(c.publisher.native for c in cameras.values()) \
        else "gz CLI fallback (per-message discovery race possible)"
    _log(
        f"serving rtsp://0.0.0.0:{port} (disable hold-off {disable_hold}s, tick {interval}s, "
        f"publishing via {mode}): "
        + ", ".join(f"/{c.path} <- udp:{c.rtp_port} -> {c.topic}" for c in cameras.values())
    )

    tick()  # publish the initial all-disabled state right away
    GLib.timeout_add(int(interval * 1000), tick)
    loop.run()
    _log("received shutdown signal, exiting")
    return 0


if __name__ == "__main__":
    sys.exit(main())
