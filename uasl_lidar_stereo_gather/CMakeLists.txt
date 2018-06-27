cmake_minimum_required(VERSION 2.8.3)
project(uasl_lidar_stereo_gather)

add_definitions(-std=c++11)

if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
endif()

find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2_ros
  tf2_msgs
  rosbag
  sensor_msgs 
  uasl_image_acquisition
  uasl_msg_gather
  cv_bridge
)

catkin_package(
  INCLUDE_DIRS include ${OpenCV_INCLUDE_DIRS}
  LIBRARIES get_data_base
  CATKIN_DEPENDS roscpp tf2_ros tf2_msgs rosbag sensor_msgs uasl_image_acquisition uasl_msg_gather cv_bridge
  DEPENDS OpenCV
)

include_directories(
  include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
)

add_library(get_data_base src/get_data_base.cpp)
target_link_libraries(get_data_base ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

add_executable(get_lidar_stereo_node src/get_lidar_stereo_node.cpp)
target_link_libraries(get_lidar_stereo_node get_data_base)

add_executable(get_lidar_stereo_node_separated src/get_lidar_stereo_node_separated.cpp)
target_link_libraries(get_lidar_stereo_node_separated get_data_base)

add_executable(convert_data_node src/convert_data_node.cpp)
target_link_libraries(convert_data_node ${catkin_LIBRARIES})

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

