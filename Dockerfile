# ==============================================================================
# Custom ROS 2 Humble Dockerfile for Jetson Orin Nano
# Base Image: Official Jetson ROS Humble Base
# ==============================================================================

FROM ros:humble-ros-base-l4t-r36.5.0

ENV DEBIAN_FRONTEND=noninteractive

# 1. Install System Dependencies & ROS 2 Packages
RUN apt-get update && apt-get install -y \
    python3-pip \
    libtbb-dev \
    ros-humble-cv-bridge \
    ros-humble-tf2-ros \
    ros-humble-sensor-msgs \
    ros-humble-nav-msgs \
    ros-humble-geometry-msgs \
    ros-humble-cartographer-ros \
    && rm -rf /var/lib/apt/lists/*

# 2. Fix TBB Dynamic Library Symlink for OpenCV TBB 2020 compatibility
RUN if [ -f /usr/lib/aarch64-linux-gnu/libtbb.so ] && [ ! -f /usr/lib/aarch64-linux-gnu/libtbb.so.2 ]; then \
        ln -s /usr/lib/aarch64-linux-gnu/libtbb.so /usr/lib/aarch64-linux-gnu/libtbb.so.2; \
    fi

# 3. Install Python Dependencies (INS_python & ROS 2 requirements)
RUN pip3 install --no-cache-dir \
    numpy \
    scipy \
    numba \
    matplotlib \
    pandas \
    tqdm \
    pygeomag \
    pymavlink \
    pyyaml

# 4. Set Workspace Directory
WORKDIR /workspace

# 5. Set Entrypoint and Default Command
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
