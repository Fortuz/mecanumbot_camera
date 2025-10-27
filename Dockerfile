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
    TZ=Etc/UTC

# ------------------------------------------------------------
# 📦 System dependencies (with ping, net-tools, etc.)
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
# 🐍 Python: pip upgrades and key packages
# ------------------------------------------------------------
RUN python3 -m pip install --upgrade pip && \
    python3 -m pip install --no-cache-dir \
        "numpy==1.26.4" "opencv-python-headless==4.9.0.80"
RUN python3 -m pip install --no-cache-dir deep-sort-realtime

# PyTorch (CPU only) + ONNX + Ultralytics
RUN python3 -m pip install --no-cache-dir \
    --index-url https://download.pytorch.org/whl/cpu \
    torch torchvision torchaudio && \
    python3 -m pip install --no-cache-dir \
    onnxruntime ultralytics && \
    python3 -m pip install --no-cache-dir --force-reinstall \
    "numpy==1.26.4" "opencv-python-headless==4.9.0.80"

# ------------------------------------------------------------
# ⚙️  ROS 2 setup and workspace preparation
# ------------------------------------------------------------
WORKDIR /ws

# ------------------------------------------------------------
# 🧠 ROS environment + mecanumbot settings
# ------------------------------------------------------------
# Append all environment setup to .bashrc for convenience
RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc && \
    echo 'source ~/mecanumbot_ws/install/setup.bash' >> /root/.bashrc && \
    echo 'export OPENCR_PORT=/dev/ttyACM0' >> /root/.bashrc && \
    echo 'export OPENCR_MODEL=mecanumbot' >> /root/.bashrc && \
    echo 'export ROS_DOMAIN_ID=19' >> /root/.bashrc && \
    echo 'export LDS_MODEL=LDS-02' >> /root/.bashrc && \
    echo 'export TURTLEBOT3_MODEL=mecanumbot' >> /root/.bashrc && \
    echo 'export ROS_LOCALHOST_ONLY=0' >> /root/.bashrc && \
    echo "alias start_robot='ros2 launch mecanumbot_bringup robot.launch.py'" >> /root/.bashrc && \
    echo "alias start_robot_with_led='ros2 launch mecanumbot_bringup robot.launch.py & ros2 run mecanumbot_led mecanumbot_led_service & wait'" >> /root/.bashrc && \
    echo "alias start_camera='ros2 launch mecanumbot_camera camera_and_detectors.launch.py & wait'" >> /root/.bashrc && \
    echo 'echo "✅ ROS 2 Humble & Mecanumbot environment ready!"' >> /root/.bashrc

# ------------------------------------------------------------
# 🧩 Build mecanumbot workspace
# ------------------------------------------------------------
#RUN mkdir -p ~/mecanumbot_ws/src && \
#    cd ~/mecanumbot_ws/src && \
#    git clone https://github.com/Fortuz/mecanumbot.git && \
#    cd ~/mecanumbot_ws && \
#    /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install --parallel-workers 1" && \
#    echo 'source ~/mecanumbot_ws/install/setup.bash' >> ~/.bashrc
#

# ------------------------------------------------------------
# 📁 ROS 2 workspace layout (add mecanumbot + camera)
# ------------------------------------------------------------
WORKDIR /ws
RUN mkdir -p /ws/src

# Clone the mecanumbot base repository
RUN git clone https://github.com/Fortuz/mecanumbot.git /ws/src/mecanumbot

# Copy this repo (mecanumbot_camera) into the workspace
# Note: this Dockerfile lives inside the repo, so we copy `.`
COPY . /ws/src/mecanumbot_camera

# ------------------------------------------------------------
# 🧠 ROS environment and colcon build
# ------------------------------------------------------------    
# ✅ Install missing ROS 2 dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-dynamixel-sdk \
    ros-humble-turtlebot3-msgs \
 && rm -rf /var/lib/apt/lists/*
RUN bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install --parallel-workers 1"

# Append environment setup to .bashrc for convenience
RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc && \
    echo 'source /ws/install/setup.bash' >> /root/.bashrc && \
    echo 'export ROS_DOMAIN_ID=0' >> /root/.bashrc && \
    echo 'export ROS_LOCALHOST_ONLY=0' >> /root/.bashrc && \
    echo "alias start_robot='ros2 launch mecanumbot_bringup robot.launch.py'" >> /root/.bashrc && \
    echo "alias start_camera='ros2 launch mecanumbot_camera camera_and_detectors.launch.py'" >> /root/.bashrc && \
    echo 'echo \"✅ ROS 2 Humble + Mecanumbot + Camera ready!\"' >> /root/.bashrc

# ------------------------------------------------------------
# 🧱 Keep container alive for dev sessions
# ------------------------------------------------------------
ENTRYPOINT ["/bin/bash"]
CMD ["-c", "source /root/.bashrc && sleep infinity"]

EXPOSE 8765

