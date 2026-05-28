cmake_minimum_required(VERSION 3.8)
project(stereo_depth_estimator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Enforce C++20 for modern ROS2 development
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find necessary ROS2 and system dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

# =======================================================
# 1. CREATE TARGET
# =======================================================
add_executable(stereo_depth_node src/stereo_depth_node.cpp)

# =======================================================
# 2. CONFIGURE TARGET
# =======================================================
target_include_directories(stereo_depth_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(stereo_depth_node ${OpenCV_LIBRARIES})

ament_target_dependencies(stereo_depth_node 
  rclcpp sensor_msgs message_filters cv_bridge image_transport
)

# =======================================================
# 3. INSTALL TARGET (Strictly AFTER creation)
# =======================================================
install(TARGETS stereo_depth_node
  DESTINATION lib/${PROJECT_NAME})

# Finalize the ROS2 package
ament_package()