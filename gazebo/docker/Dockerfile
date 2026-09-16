# Headless Gazebo Harmonic + ardupilot_gazebo, built for the "gazebo" compose
# profile (see gazebo/gazebo-sitl-plan.html). Runs `gz sim -s` (server only,
# no GUI) driving ArduPilot SITL over the JSON FDM link, rendering camera
# frames through GstCameraPlugin straight to RTP/UDP.
#
# Ubuntu 24.04 (noble) is the base ONLY because Gazebo Harmonic (the locked
# LTS release) ships apt binaries for jammy/noble but not for a slim/distro-
# less base, and ardupilot_gazebo has to be compiled from source against it
# (no prebuilt package exists for the plugin itself) — see "Build
# ardupilot_gazebo" below for why source build was verified as necessary.
FROM ubuntu:24.04

# ---------------------------------------------------------------------------
# Gazebo's own apt repository (packages.osrfoundation.org). Ubuntu's default
# noble repos do not carry gz-sim8/Harmonic; the OSRF repo is the only source
# for the `gz-harmonic` metapackage (server + `gz` CLI, incl. `gz topic`,
# which MediaMTX's runOnDemand hook shells out to) and for libgz-sim8-dev
# (headers ardupilot_gazebo's CMake needs to compile its plugins against).
# Steps verified against https://gazebosim.org/docs/harmonic/install_ubuntu/.
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        curl ca-certificates gnupg lsb-release \
    && curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
        -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
        > /etc/apt/sources.list.d/gazebo-stable.list \
    && rm -rf /var/lib/apt/lists/*

# System dependencies:
#   - gz-harmonic: the Harmonic server/GUI/CLI metapackage. We only ever run
#     the server (`gz sim -s`), but the metapackage is what installs `gz`
#     itself plus the ogre2 render engine and the sensors/physics systems the
#     world file loads as <plugin> entries.
#   - build-essential, cmake, pkg-config, git: to compile ardupilot_gazebo
#     (upstream ships no binary package — verified against its README, which
#     documents only a from-source cmake build).
#   - libgz-sim8-dev, rapidjson-dev, libopencv-dev, libgstreamer1.0-dev,
#     libgstreamer-plugins-base1.0-dev, gstreamer1.0-plugins-bad,
#     gstreamer1.0-libav, gstreamer1.0-gl: ardupilot_gazebo's own "Ubuntu
#     Harmonic (apt)" dependency list, copied verbatim from its README
#     (https://github.com/ArduPilot/ardupilot_gazebo/blob/main/README.md).
#     libgz-sim8-dev transitively pulls in libgz-common5-dev/libgz-rendering8-
#     dev/libgz-cmake3-dev, which its CMakeLists.txt requires for the
#     (default, no GZ_VERSION set) Harmonic branch.
#   - cppzmq-dev: NOT in ardupilot_gazebo's documented dependency list, but the
#     build fails without it. libgz-sim8-dev pulls gz-transport13, whose
#     gz-transport13-targets.cmake declares CPPZMQ::CPPZMQ in its link
#     interface; libgz-transport13-dev ships the cmake config but not the
#     cppzmq headers that define that IMPORTED target, so cmake aborts at the
#     generate step with "the target was not found" before any compilation
#     starts. cppzmq-dev supplies them (and pulls libzmq3-dev).
#   - gstreamer1.0-plugins-good, gstreamer1.0-plugins-ugly: runtime elements
#     GstCameraPlugin's pipeline actually links (verified by reading
#     src/GstCameraPlugin.cc): rtph264pay + udpsink (good) and x264enc
#     (ugly, since <use_cuda> is false here — no GPU encoder in a software-
#     rendering container). The README's build-dependency list above only
#     covers *-bad (needed to compile), not these runtime elements.
#   - libgl1-mesa-dri, libegl1, mesa-utils: Mesa's llvmpipe (software GL) and
#     EGL. `gz sim --headless-rendering` initialises Ogre2 through EGL/PBuffer
#     instead of GLX/X11 (verified in gz-rendering's Ogre2RenderEngine.cc —
#     CreateContext() branches on the "headless" render-engine param), so no
#     Xvfb/X server is needed; llvmpipe is what actually does the rasterising
#     since Docker Desktop's Windows host exposes no real GPU render node here.
#   - gettext-base: provides `envsubst`, which entrypoint.sh uses to render
#     the model SDF template (the RTP destination and FDM bind port are
#     per-instance, so they can't be baked into the image — see
#     entrypoint.sh).
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        gz-harmonic \
        build-essential cmake pkg-config git \
        libgz-sim8-dev rapidjson-dev \
        cppzmq-dev \
        libopencv-dev \
        libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
        gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl \
        gstreamer1.0-plugins-good gstreamer1.0-plugins-ugly \
        libgl1-mesa-dri libegl1 mesa-utils \
        gettext-base \
        python3-gz-transport13 python3-gz-msgs10 \
    && rm -rf /var/lib/apt/lists/*
# python3-gz-transport13/-msgs10 back camera_gate.py's persistent enable-topic
# publishers. They were present transitively anyway (via the gz dev packages),
# but the gate degrades to a racy per-message `gz topic` fallback without
# them, so they are pinned as an explicit dependency here.

# Explicit even though "harmonic" is ardupilot_gazebo's default (no
# GZ_VERSION set) branch in its CMakeLists.txt — future edits to this
# Dockerfile that add a GZ_VERSION-sensitive step won't silently drift.
ENV GZ_VERSION=harmonic

# ---------------------------------------------------------------------------
# Build ardupilot_gazebo (ArduPilotPlugin + GstCameraPlugin) from source.
# Pinned to a specific commit, not `main`, for reproducible builds — the repo
# publishes no version tags (checked via the GitHub tags API: none exist).
# Bump by re-checking https://github.com/ArduPilot/ardupilot_gazebo/commits/main
# and updating the SHA below.
ARG ARDUPILOT_GAZEBO_REF=082a0fe231f6e63bc8d1598f1cba461d9e2ea7f5

# B1 (see gazebo-sitl-plan.html "Blockers"): upstream GstCameraPlugin only
# gates its GStreamer *encode* thread on <enable_topic> — OnImage() early-
# returns while disabled, but the plugin stays subscribed to the camera's
# image topic forever, so gz-sensors' CameraSensor::Update() never takes its
# "no subscribers, skip Render()" fast path and the sensor renders every
# step regardless of enable state. Measured against the running stack:
# publishing `false` to /cam2/enable left cam2's frame rate unchanged. That
# defeats the "one camera at a time" render gate the whole design depends on
# (two cameras rendering together measured RTF 0.44 vs 0.92-0.95 for one).
# patches/0001-...patch makes the image-topic subscription follow the
# enable state (subscribe on enable, unsubscribe on disable) so a disabled
# camera actually has zero subscribers and gz-sensors stops rendering it.
# Do NOT delete this patch — without it, two-camera SITL runs at ~44% RTF,
# which is disqualifying.
COPY docker/patches/ /opt/ardupilot_gazebo_patches/
RUN git clone https://github.com/ArduPilot/ardupilot_gazebo /opt/ardupilot_gazebo \
    && git -C /opt/ardupilot_gazebo checkout "${ARDUPILOT_GAZEBO_REF}" \
    && git -C /opt/ardupilot_gazebo apply \
        /opt/ardupilot_gazebo_patches/0001-gstcameraplugin-gate-render-on-enable-state.patch \
    && cmake -S /opt/ardupilot_gazebo -B /opt/ardupilot_gazebo/build \
        -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    && cmake --build /opt/ardupilot_gazebo/build -j"$(nproc)"

# Plugin search path (ArduPilotPlugin.so / GstCameraPlugin.so) and the
# resource path for ardupilot_gazebo's own stock worlds/models (our world
# includes model://runway from here). Both documented in ardupilot_gazebo's
# README "Environment Configuration" section. Our own world/model tree is
# appended in the next stage — kept as a separate COPY layer so editing SDF
# doesn't invalidate the (expensive) build layer above.
ENV GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH}
ENV GZ_SIM_RESOURCE_PATH=/opt/ardupilot_gazebo/models:/opt/ardupilot_gazebo/worlds

# ---------------------------------------------------------------------------
# Our own assets. worlds/ is copied verbatim onto the resource path (no
# per-instance values, so no templating needed). models/ holds
# model.sdf.template, not model.sdf — entrypoint.sh renders the template into
# a writable runtime directory, because the RTP host:port and FDM bind port
# are per-instance env vars (NET_PREFIX etc.) that can't be baked into the
# image. Keeping the un-rendered source out of GZ_SIM_RESOURCE_PATH avoids
# `gz sim` finding a model.config with no matching model.sdf next to it.
COPY worlds/ /opt/gazebo/worlds/
COPY models/ /opt/gazebo/models-src/
ENV GZ_SIM_RESOURCE_PATH=/opt/gazebo/worlds:${GZ_SIM_RESOURCE_PATH}

# The render gate (B2, see gazebo-sitl-plan.html "Blockers"). MediaMTX is
# built FROM scratch and execs runOnDemand hooks with no shell, so it cannot
# invoke `gz topic` itself; instead this container polls mediamtx's HTTP API
# and toggles its own cameras. entrypoint.sh runs it alongside `gz sim`.
# python3 is already present transitively (verified: /usr/bin/python3, pulled
# in by the gz-harmonic/dev packages above) and the script is stdlib-only, so
# nothing extra is installed — but it is a transitive dependency, so if a
# future apt change drops python3 this COPY is the thing that breaks.
COPY docker/camera_gate.py /opt/gazebo/camera_gate.py

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# ---------------------------------------------------------------------------
# Defaults for every value the shared compose contract parameterises
# (gazebo/gazebo-sitl-plan.html "Locked decisions" + the sim network's
# NET_PREFIX default of 172.28). docker-compose.yaml overrides these per the
# same env vars used elsewhere in the repo, so a bare `docker run` of this
# image still comes up in a sane single-instance configuration.
ENV GZ_FDM_ADDR=0.0.0.0 \
    GZ_FDM_IN_PORT=9002 \
    GZ_CAM1_HOST=172.28.0.31 \
    GZ_CAM1_RTP_PORT=5600 \
    GZ_CAM2_HOST=172.28.0.32 \
    GZ_CAM2_RTP_PORT=5601 \
    GZ_VERBOSITY=1

ENTRYPOINT ["/entrypoint.sh"]
