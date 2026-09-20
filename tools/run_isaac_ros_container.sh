#!/bin/bash
# ==============================================================================
# Launch Script for Isaac ROS (x86_64) Container with GPU & GUI Acceleration
# ==============================================================================

WORKSPACE_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." >/dev/null 2>&1 && pwd )"

# Grant local X11 access for GUI tools like RViz2
xhost +local:root >/dev/null 2>&1 || true

DOCKER_ARGS=()
DOCKER_ARGS+=(--runtime nvidia)
DOCKER_ARGS+=(--privileged)
DOCKER_ARGS+=(--network host)
DOCKER_ARGS+=(--ipc host)
DOCKER_ARGS+=(-e FASTRTPS_DEFAULT_PROFILES_FILE=/usr/local/share/middleware_profiles/rtps_udp_profile.xml)

# GUI / Display forwarding
if [ -n "$DISPLAY" ]; then
    DOCKER_ARGS+=(-e DISPLAY="$DISPLAY")
    DOCKER_ARGS+=(-v /tmp/.X11-unix:/tmp/.X11-unix:rw)
fi

if [ -f "$HOME/.Xauthority" ]; then
    DOCKER_ARGS+=(-v "$HOME/.Xauthority:/root/.Xauthority:rw")
fi

# Workspace mounting
DOCKER_ARGS+=(-v "$WORKSPACE_DIR:/workspaces/isaac_ros-dev")
DOCKER_ARGS+=(-v "$WORKSPACE_DIR:/workspace")
DOCKER_ARGS+=(-w /workspaces/isaac_ros-dev)

IMAGE_NAME="isaac_ros_dev-x86_64:latest"

echo "=================================================================="
echo "🚀 Starting Isaac ROS Dev Container ($IMAGE_NAME)"
echo "📂 Workspace mounted at: /workspaces/isaac_ros-dev"
echo "=================================================================="

if [ -t 0 ]; then
    if [ $# -eq 0 ]; then
        docker run -it --rm "${DOCKER_ARGS[@]}" "$IMAGE_NAME" bash
    else
        docker run -it --rm "${DOCKER_ARGS[@]}" "$IMAGE_NAME" bash -i -c "$*"
    fi
else
    if [ $# -eq 0 ]; then
        docker run --rm "${DOCKER_ARGS[@]}" "$IMAGE_NAME" bash
    else
        docker run --rm "${DOCKER_ARGS[@]}" "$IMAGE_NAME" bash -c "$*"
    fi
fi
