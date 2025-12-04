cmake_minimum_required(VERSION 3.8)
project(mecanumbot_camera)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Ensure Python modules are installed
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install executables
install(PROGRAMS
  mecanumbot_camera/ball_tracker_rgb.py
  mecanumbot_camera/people_detector.py
  mecanumbot_camera/overlay_fused.py
  mecanumbot_camera/detection_overlay.py
  mecanumbot_camera/video_publisher.py
  mecanumbot_camera/ball_follower.py
  mecanumbot_camera/behavior_manager.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install configuration files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install model files (ONNX, etc.)
install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
)

# Install resource files
install(DIRECTORY resource/
  DESTINATION share/${PROJECT_NAME}/resource
)

ament_package()
