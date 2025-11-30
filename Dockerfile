# ============================================================
# 🐋 Mecanumbot ROS2 Humble + Camera + Tools Dockerfile
# ============================================================

FROM osrf/ros:humble-desktop

# ------------------------------------------------------------
# 🧰 Basic environment setup
# ------------------------------------------------------------
ENV DEBIAN_FRONTEND=noninteractive \
    PIP_DISABLE_PIP_VERSION_CHECK=1 \
    PIP_NO_CACHE_DIR=1 \
    PYTHONDONTWRITEBYTECODE=1 \
    TZ=Etc/UTC \
    ROS_DOMAIN_ID=19 \
    ROS_LOCALHOST_ONLY=0 \
    RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# ------------------------------------------------------------
# 📦 System dependencies
# ------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    iputils-ping net-tools curl wget iproute2 dnsutils \
    git vim nano \
    ros-humble-vision-msgs \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-v4l2-camera \
    ros-humble-image-view \
    ros-humble-foxglove-bridge \
    ros-humble-foxglove-msgs \
    ffmpeg libsm6 libxext6 libgl1 libglib2.0-0 \
 && rm -rf /var/lib/apt/lists/*

# ------------------------------------------------------------
# 🧹 Remove conflicting packages
# ------------------------------------------------------------
RUN apt-get purge -y python3-sympy || true && \
    apt-get autoremove -y && rm -rf /var/lib/apt/lists/*

# ------------------------------------------------------------
# 🐍 Python packages
# ------------------------------------------------------------
RUN python3 -m pip install --upgrade pip && \
    python3 -m pip install --no-cache-dir \
        numpy==1.26.4 opencv-python-headless==4.9.0.80 \
        deep-sort-realtime

RUN python3 -m pip install --no-cache-dir \
    --index-url https://download.pytorch.org/whl/cpu \
    torch torchvision torchaudio && \
    python3 -m pip install --no-cache-dir \
    onnxruntime ultralytics && \
    python3 -m pip install --no-cache-dir --force-reinstall \
        numpy==1.26.4 opencv-python-headless==4.9.0.80

# ------------------------------------------------------------
# ⚙️ ROS 2 workspace
# ------------------------------------------------------------
WORKDIR /ws
RUN mkdir -p /ws/src

# Clone Mecanumbot base repo
#RUN git clone https://github.com/Fortuz/mecanumbot.git /ws/src/mecanumbot

# Copy this repo (camera package)
#COPY src/mecanumbot_camera /ws/src/mecanumbot_camera


# Install missing ROS deps
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-dynamixel-sdk \
    ros-humble-turtlebot3-msgs \
 && rm -rf /var/lib/apt/lists/*

# Build
RUN bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install --parallel-workers 1"

# ------------------------------------------------------------
# 🧠 Setup environment
# ------------------------------------------------------------
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /ws/install/setup.bash" >> /root/.bashrc && \
    echo "export ROS_DOMAIN_ID=19" >> /root/.bashrc && \
    echo "export ROS_LOCALHOST_ONLY=0" >> /root/.bashrc && \
    echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> /root/.bashrc && \
    echo "alias start_robot='ros2 launch mecanumbot_bringup robot.launch.py'" >> /root/.bashrc && \
    echo "alias start_camera='ros2 launch mecanumbot_camera camera_and_detectors.launch.py'" >> /root/.bashrc && \
    echo 'echo "✅ ROS 2 Humble + Mecanumbot + Camera ready!"' >> /root/.bashrc

# ------------------------------------------------------------
# 🧱 Keep container alive
# ------------------------------------------------------------
ENTRYPOINT ["/bin/bash"]
CMD ["-c", "source /root/.bashrc && sleep infinity"]

EXPOSE 8765
