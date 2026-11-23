#!/usr/bin/env bash
# Renders the Iris-dual-camera model SDF from its template, then launches
# Gazebo headless (server only, no GUI) with rendering enabled so the camera
# sensors actually produce frames.
#
# The model is templated rather than static because GstCameraPlugin's
# <udp_host>/<udp_port> (where it sends RTP -- camera_gate.py's RTSP mounts in
# this container, 127.0.0.1 by default) and ArduPilotPlugin's <fdm_port_in>
# depend on the per-instance port map — the same
# reason docker-compose.yaml itself is all env-var defaults (see CLAUDE.md).
# Baking one instance's addresses into the image would break every other
# instance running side by side on this host.
set -euo pipefail

MODEL_SRC=/opt/gazebo/models-src/iris_dual_cam
MODEL_RUNTIME=/run/gazebo/models/iris_dual_cam
WORLD=/opt/gazebo/worlds/iris_runway.sdf

mkdir -p "${MODEL_RUNTIME}"
cp "${MODEL_SRC}/model.config" "${MODEL_RUNTIME}/model.config"

# Restrict envsubst to a known variable list: an unrestricted `envsubst` will
# also expand any stray "$" that happens to appear in the SDF (there
# shouldn't be one, but a bare `envsubst` with no arguments is a footgun that
# silently corrupts the file if one is ever added).
export GZ_FDM_ADDR="${GZ_FDM_ADDR:-0.0.0.0}"
export GZ_FDM_IN_PORT="${GZ_FDM_IN_PORT:-9002}"
export GZ_CAM1_HOST="${GZ_CAM1_HOST:-127.0.0.1}"
export GZ_CAM1_RTP_PORT="${GZ_CAM1_RTP_PORT:-5600}"
export GZ_CAM2_HOST="${GZ_CAM2_HOST:-127.0.0.1}"
export GZ_CAM2_RTP_PORT="${GZ_CAM2_RTP_PORT:-5601}"

envsubst \
    '${GZ_FDM_ADDR} ${GZ_FDM_IN_PORT} ${GZ_CAM1_HOST} ${GZ_CAM1_RTP_PORT} ${GZ_CAM2_HOST} ${GZ_CAM2_RTP_PORT}' \
    < "${MODEL_SRC}/model.sdf.template" \
    > "${MODEL_RUNTIME}/model.sdf"

# Prepend the runtime dir so `model://iris_dual_cam` resolves to the rendered
# copy, not the (nonexistent, template-only) source directory baked into the
# image at build time.
export GZ_SIM_RESOURCE_PATH="/run/gazebo/models:${GZ_SIM_RESOURCE_PATH:-}"

# Camera RTSP server + render gate (camera_gate.py): serves each camera's RTP
# as an on-demand RTSP mount and enables a camera only while its mount has a
# client (dfvideo-gazebo, i.e. a GCS watching). It runs alongside gz sim, not
# chained to it via `exec` (bash can only `wait` on its own direct children,
# so both have to be started as background jobs of *this* shell for the
# supervision below to observe either one exiting):
#   - if gz sim exits, the container's lifecycle is gz sim's -- tear down
#     camera_gate.py too and propagate gz sim's exit status.
#   - if camera_gate.py dies first, gz sim must keep running (physics and the
#     autopilot link don't depend on it), but camera video is gone with it,
#     so the loss must not go unnoticed -- log it loudly to stderr
#     (`docker logs`) the moment it happens, then keep waiting on gz sim.
python3 /opt/gazebo/camera_gate.py &
CAMERA_GATE_PID=$!

# -s          server only — no GUI process, nothing to point a display at.
# -r          start the simulation running immediately (matches ardupilot.org's
#             documented `gz sim -v4 -r iris_runway.sdf` invocation; without it
#             the sim loads paused and never steps).
# --headless-rendering
#             routes Ogre2 through EGL/PBuffer instead of GLX, which is what
#             lets camera sensors render with no X server at all — verified
#             against gz-rendering's Ogre2RenderEngine.cc (CreateContext()
#             branches on the "headless" engine param onto the EGL path).
#             Without this flag Ogre2 tries to open a GLX/X11 display first
#             and only falls back to headless if that fails.
# -v          console verbosity 0-4, GZ_VERBOSITY defaults to 1 in the
#             Dockerfile (matches gz sim's own default).
gz sim -s -r --headless-rendering -v"${GZ_VERBOSITY:-1}" "${WORLD}" &
GZ_PID=$!

# This script stays PID 1, so forward `docker stop` (SIGTERM) / Ctrl-C
# (SIGINT) to both children -- bash does not do this on its own for
# background jobs, and neither gz sim nor camera_gate.py would otherwise see
# the signal at all.
trap 'kill -TERM "${GZ_PID}" "${CAMERA_GATE_PID}" 2>/dev/null || true' TERM INT

# `wait -n` (bash 4.3+; this image ships bash 5.2 via Ubuntu 24.04) blocks
# until whichever of the two exits first, without needing to know in advance
# which one that'll be. `set +e` around it because a non-zero exit here is
# the normal, expected case (either process can legitimately be the one that
# exits first) -- not a script error `set -e` should abort on.
set +e
wait -n "${GZ_PID}" "${CAMERA_GATE_PID}"
FIRST_EXIT=$?
set -e

if kill -0 "${GZ_PID}" 2>/dev/null; then
  # gz sim is still alive, so camera_gate.py must be the one that exited.
  echo "entrypoint.sh: camera_gate.py exited (status ${FIRST_EXIT}) -- camera RTSP mounts and the render gate are gone (no camera video); cameras stay at whichever enable state they were last set to. gz sim keeps running." >&2
  wait "${GZ_PID}"
  exit $?
fi

# gz sim exited -- take camera_gate.py down with it and propagate gz sim's
# real exit status (not FIRST_EXIT, in case of the near-simultaneous-exit
# race where camera_gate.py happened to be reaped by `wait -n` first).
kill "${CAMERA_GATE_PID}" 2>/dev/null || true
wait "${GZ_PID}"
exit $?
