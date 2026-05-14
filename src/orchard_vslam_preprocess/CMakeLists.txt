cmake_minimum_required(VERSION 3.8)
project(orchard_vslam_preprocess)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)

if(DEFINED OpenCV_CONFIG_PATH AND EXISTS "${OpenCV_CONFIG_PATH}")
  set(OpenCV_DIR "${OpenCV_CONFIG_PATH}" CACHE PATH "OpenCV config path from cv_bridge" FORCE)
endif()

find_package(OpenCV REQUIRED COMPONENTS core features2d imgproc)

add_executable(orchard_vslam_preprocess_node
  src/orchard_vslam_preprocess_node.cpp
)

target_compile_features(orchard_vslam_preprocess_node PUBLIC cxx_std_17)

ament_target_dependencies(orchard_vslam_preprocess_node
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
  message_filters
)

target_link_libraries(orchard_vslam_preprocess_node
  ${OpenCV_LIBRARIES}
)

target_include_directories(orchard_vslam_preprocess_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)

install(TARGETS orchard_vslam_preprocess_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
