# ==============================================================================
# Custom ROS 2 Humble Dockerfile for Jetson Orin Nano
# Base Image: Official Jetson ROS Humble Base
# ==============================================================================

FROM ros:humble-ros-base-l4t-r36.5.0

ENV DEBIAN_FRONTEND=noninteractive

# 1. Install System Dependencies, Build Tools & ROS 2 Packages
RUN apt-get update && apt-get install -y \
    bash-completion \
    libboost-all-dev \
    libtbb-dev \
    python3-pip \
    python3-posix-ipc \
    gstreamer1.0-tools \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    ros-humble-cv-bridge \
    ros-humble-tf2-ros \
    ros-humble-sensor-msgs \
    ros-humble-nav-msgs \
    ros-humble-geometry-msgs \
    ros-humble-stereo-msgs \
    ros-humble-visualization-msgs \
    ros-humble-image-transport \
    ros-humble-camera-info-manager \
    ros-humble-diagnostic-updater \
    ros-humble-cartographer-ros \
    ros-humble-foxglove-msgs \
    ros-humble-magic-enum \
    ros-humble-rviz-common \
    ros-humble-rviz-ogre-vendor \
    ros-humble-rviz-rendering \
    ros-humble-nav2-costmap-2d \
    ros-humble-pcl-conversions \
    ros-humble-pcl-ros \
    && rm -rf /var/lib/apt/lists/*

# 2. glibc 2.34+ Compatibility (librt.so symlink & empty librt.a for CMake link resolution)
RUN ln -sf librt.so.1 /usr/lib/aarch64-linux-gnu/librt.so && \
    ar cr /usr/lib/aarch64-linux-gnu/librt.a

# 3. Fix TBB Dynamic Library Symlink for OpenCV TBB 2020 compatibility
RUN if [ -f /usr/lib/aarch64-linux-gnu/libtbb.so ] && [ ! -f /usr/lib/aarch64-linux-gnu/libtbb.so.2 ]; then \
        ln -s /usr/lib/aarch64-linux-gnu/libtbb.so /usr/lib/aarch64-linux-gnu/libtbb.so.2; \
    fi

# 4. Install Python Dependencies (INS_python & ROS 2 requirements)
RUN pip3 install --no-cache-dir \
    numpy \
    scipy \
    numba \
    matplotlib \
    pandas \
    tqdm \
    pygeomag \
    pymavlink \
    pyyaml \
    netifaces

# 5. Connect /opt/venv with ROS 2 and system python packages
RUN if [ -d /opt/venv ]; then \
        PYTHON_VER=$(python3 -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')") && \
        mkdir -p /opt/venv/lib/python${PYTHON_VER}/site-packages && \
        printf "/opt/ros/humble/install/lib/python${PYTHON_VER}/site-packages\n/usr/lib/python3/dist-packages\n" > /opt/venv/lib/python${PYTHON_VER}/site-packages/ros_humble.pth; \
    fi

# 6. Patch /ros_environment.sh to source dual ROS prefixes (APT + compiled)
RUN sed -i 's|ros_source_env "$ROS_ROOT/install/setup.bash"|ros_source_env "$ROS_ROOT/setup.bash"\n\tros_source_env "$ROS_ROOT/install/setup.bash"|g' /ros_environment.sh

# 7. Configure interactive Bash completion and auto-sourcing
RUN cat << 'EOF' >> /root/.bashrc

# Sourcing ROS environment
if [ -f /ros_environment.sh ]; then
    source /ros_environment.sh > /dev/null 2>&1 || true
fi
if [ -f /workspace/install/setup.bash ]; then
    source /workspace/install/setup.bash > /dev/null 2>&1 || true
fi

# Enable Bash Completion
if [ -f /usr/share/bash-completion/bash_completion ]; then
    . /usr/share/bash-completion/bash_completion
elif [ -f /etc/bash_completion ]; then
    . /etc/bash_completion
fi

# Enable ROS 2 & Colcon Completion
if [ -f /opt/ros/humble/install/share/ros2cli/environment/ros2-argcomplete.bash ]; then
    source /opt/ros/humble/install/share/ros2cli/environment/ros2-argcomplete.bash
fi
eval "$(register-python-argcomplete3 ros2)" 2>/dev/null || true
eval "$(register-python-argcomplete3 colcon)" 2>/dev/null || true
EOF

# 8. Set Workspace Directory
WORKDIR /workspace

# 9. Set Entrypoint and Default Command
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
