FROM osrf/ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive \
    PIP_NO_CACHE_DIR=1 \
    PYTHONDONTWRITEBYTECODE=1 \
    ROS_DOMAIN_ID=19 \
    ROS_LOCALHOST_ONLY=0 \
    RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Base system + ROS dependencies required by mecanumbot_camera
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-colcon-common-extensions \
    git curl wget ca-certificates \
    ros-humble-vision-msgs \
    ros-humble-vision-opencv \
    ros-humble-image-transport \
    ros-humble-cv-bridge \
    ros-humble-rqt-image-view \
    libgl1 libglib2.0-0 \
 && rm -rf /var/lib/apt/lists/*

# Python dependencies (NumPy 1.x, OpenCV, and TensorFlow Lite runtime)
RUN python3 -m pip install --upgrade pip && \
    python3 -m pip install --no-cache-dir \
        numpy==1.26.4 \
        opencv-python-headless==4.9.0.80 \
        tflite-runtime==2.14.0 \
        pyyaml

# Copy package sources into a ROS 2 workspace
WORKDIR /ws/src/mecanumbot_camera
COPY . /ws/src/mecanumbot_camera

# Build mecanumbot_camera
WORKDIR /ws
RUN . /opt/ros/humble/setup.sh && \
    colcon build --packages-select mecanumbot_camera

ENV PYTHONUNBUFFERED=1 \
    ROS_DOMAIN_ID=19 \
    ROS_LOCALHOST_ONLY=0 \
    RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
