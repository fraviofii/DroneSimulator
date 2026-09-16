# gazebo/

Assets for the `gazebo-sitl` image (`ghcr.io/draganfly/gazebo-sitl`), built
from `gazebo/docker/Dockerfile`. See `gazebo/gazebo-sitl-plan.html` for the
full design; this file only covers what lives in this directory.

## Layout

- `docker/Dockerfile` — Gazebo Harmonic (Ubuntu 24.04 base) + `ardupilot_gazebo`
  (`ArduPilotPlugin`, `GstCameraPlugin`) built from source, pinned to a fixed
  commit for reproducible builds. Mesa/EGL (`llvmpipe`) provides software
  rendering; `gz sim --headless-rendering` uses that path directly (no Xvfb).
- `docker/entrypoint.sh` — renders `models/iris_dual_cam/model.sdf.template`
  into a runtime SDF (the RTP destinations and FDM bind port are per-instance
  env vars, so they can't be baked into the image) and launches
  `gz sim -s -r --headless-rendering worlds/iris_runway.sdf`.
- `worlds/iris_runway.sdf` — ardupilot_gazebo's stock `iris_runway` world,
  swapping its `iris_with_gimbal` payload for `model://iris_dual_cam` and
  adding six static boxes/cones near the pad as landmarks for the downward
  camera.
- `models/iris_dual_cam/` — an Iris quadcopter (`iris_with_standoffs`
  airframe) with two **fixed** cameras instead of a gimbal: `camera_front`
  and `camera_down`, each running its own `GstCameraPlugin` instance
  streaming RTP/H.264 to a separate host:port (`gzcam1`/`gzcam2` in compose).
  Ships `model.sdf.template`, not `model.sdf` — see entrypoint.sh above.

## Non-obvious things

- **The rotor/PID/LiftDrag plugin block in `model.sdf.template` is copied
  verbatim from upstream's `iris_with_ardupilot/model.sdf`.** It is airframe
  tuning; nothing about the camera work changes it, and re-deriving it
  would only add risk.
- **`fdm_port_out` is deliberately absent.** ArduPilotPlugin deprecated it —
  the reply address is auto-detected from the first incoming packet, per
  its source. The compose contract's `GZ_FDM_OUT_PORT` therefore configures
  something on the `autopilot` service's side, not this model.
- **`<use_basic_pipeline>true</use_basic_pipeline>` is load-bearing, not
  optional.** Without it, `GstCameraPlugin` defaults to an MPEG-TS/UDP
  pipeline instead of bare RTP, which is not what the `--video-source rtp`
  bridge in `support-tools` expects.
- **`enable_topic` is set explicitly to `/cam1/enable` and `/cam2/enable`.**
  Left unset, the plugin derives `<sensor's gz topic>/enable_streaming`
  instead, a name MediaMTX's `runOnDemand` hook can't predict.
- **Line endings matter here.** This repo's Windows checkout uses
  `core.autocrlf=true` with no `.gitattributes`; `docker/entrypoint.sh` must
  stay LF (`\n`) on disk, or its shebang breaks inside the Linux container.
  If this file ever comes back CRLF after a fresh `git checkout` on Windows,
  that's why — either add a `.gitattributes` entry
  (`gazebo/docker/entrypoint.sh text eol=lf`) or re-save it LF-only before
  building.

## Resolved the hard way — history that will save you a re-derivation

- **A disabled `GstCameraPlugin` camera did NOT stop rendering upstream
  (B1), and the first fix wedged message dispatch.** Upstream subscribes to
  the sensor's image topic unconditionally, so `enable_topic` only gated
  the GStreamer encode thread and the sensor rendered every frame
  regardless (measured: publishing `false` left the frame rate unchanged;
  two cameras rendering drop RTF to ~0.44 vs ~0.92 for one). Patch v1 made
  the image subscription follow the enable state, but did the
  `node.Subscribe()`/`node.Unsubscribe()` calls (and the GStreamer thread
  join) *inside* the enable-topic callback — reentering gz-transport from
  its own callback thread intermittently wedged dispatch, and under
  reader-driven flapping enables/disables were lost for minutes at a time.
  The current patch (`docker/patches/0001-...`) records desired state in
  the callback and applies every transition on the simulation thread in
  `PreUpdate()` (`ReconcileStreamingState()`), which also makes repeated
  publishes of the same state idempotent — `camera_gate.py` re-publishes
  desired state every tick and relies on that.
- **`gz topic -p` one-shot publishes are droppable.** Each CLI invocation
  is a fresh process that must re-discover the subscriber; under rapid
  toggling, individual messages were measurably lost (a camera streamed on
  for minutes after its `false`). `camera_gate.py` therefore holds
  persistent `gz.transport13` python publishers (`python3-gz-transport13`,
  now an explicit Dockerfile dependency) and republishes state every poll
  tick; the `gz topic` subprocess path survives only as a logged fallback.
- **Mesa/llvmpipe fallback works unconfigured.** `--headless-rendering`
  initialises Ogre2 through EGL with no X server and no
  `LIBGL_ALWAYS_SOFTWARE`/`GALLIUM_DRIVER` env needed — verified against
  the running container (cameras render; one 720p camera ≈ 20 fps, RTF
  0.92–0.95).
