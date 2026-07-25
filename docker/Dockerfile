FROM ros:jazzy

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-navigation2 \
      ros-jazzy-nav2-bringup \
      ros-jazzy-rmw-zenoh-cpp \
      ros-jazzy-rviz2 \
      tigervnc-standalone-server \
      novnc \
      websockify \
      python3-pip \
      libegl1 \
      libgl1-mesa-dri \
 && rm -rf /var/lib/apt/lists/*

# rviz2 for a headless Linux container has nowhere to put its window on
# macOS, so it's tunneled through a browser instead: Xvnc (tigervnc) is the
# X server and the VNC server in one process, websockify/novnc bridge that
# to a plain web page. See scripts/rviz.sh and `make rviz`.

# Zenoh instead of DDS: discovery goes through a router over TCP (run by the
# core service, scripts/core.sh), so none of the DDS-in-docker failure modes
# exist — no multicast dependency, no participant-index ceiling, no wedged
# wait-sets.
# Both FastDDS and CycloneDDS were tried first and hit those in this setup.
ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp
ENV ZENOH_ROUTER_CHECK_ATTEMPTS=10

# The simulator lives in here too: the sim service (compose.yaml) runs
# sim/spar_sim on this image, headless. MUJOCO_GL=egl renders on the CPU
# through Mesa's surfaceless EGL path (llvmpipe) -- no GPU in Docker on
# macOS, and none needed for a 320x240 camera. The version is pinned to
# match the host venv (the viewer loads the same MJCF; version skew is the
# new editor-version drift).
RUN pip install --no-cache-dir --break-system-packages mujoco==3.10.0

WORKDIR /ws

# The image holds the *environment* only; the workspace (src, scripts, build,
# install, logs) is bind-mounted by compose so build artifacts and logs land
# on the host. Only the package manifests are copied in — just enough for
# rosdep to install dependencies. The actual colcon build is run by hand,
# in the container, and lands in ./build on the host (see README), so code
# edits need an incremental rebuild, never an image rebuild.
COPY ground/src/spar_ground/package.xml src/spar_ground/
COPY ground/src/spar_bringup/package.xml src/spar_bringup/
RUN . /opt/ros/jazzy/setup.sh \
 && apt-get update && rosdep update --rosdistro jazzy \
 && rosdep install --from-paths src --ignore-src -y --rosdistro jazzy \
 && rm -rf /var/lib/apt/lists/*

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["sleep", "infinity"]
