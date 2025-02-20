cmake_minimum_required(VERSION 3.5)
project(orbslam3)

# You should set the PYTHONPATH to your own python site-packages path
set(ENV{PYTHONPATH} "/opt/ros/humble/lib/python3.10/site-packages")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -g)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED)
find_package(ORB_SLAM3 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


include_directories(
  include
  ${ORB_SLAM3_ROOT_DIR}/include
  ${ORB_SLAM3_ROOT_DIR}/include/CameraModels
  ${ORB_SLAM3_ROOT_DIR}/Thirdparty/Sophus
)

link_directories(
  include
)

add_executable(mono
  src/monocular/mono-slam-node.cpp
)
add_executable(grid
  src/monocular/grid-map.cpp
)
add_executable(pointcloud
  src/monocular/point-cloud.cpp
)
ament_target_dependencies(mono rclcpp sensor_msgs nav_msgs geometry_msgs cv_bridge ORB_SLAM3 Pangolin OpenCV tf2_ros tf2_geometry_msgs )
ament_target_dependencies(grid rclcpp sensor_msgs nav_msgs geometry_msgs cv_bridge ORB_SLAM3 Pangolin OpenCV tf2_ros tf2_geometry_msgs )
ament_target_dependencies(pointcloud rclcpp sensor_msgs nav_msgs geometry_msgs cv_bridge ORB_SLAM3 Pangolin OpenCV tf2_ros tf2_geometry_msgs )

install(TARGETS mono grid pointcloud DESTINATION lib/${PROJECT_NAME})

# Install launch files.
# install(DIRECTORY launch config vocabulary
# DESTINATION share/${PROJECT_NAME}/)

ament_package()
