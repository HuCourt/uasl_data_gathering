cmake_minimum_required(VERSION 2.8.3)
project(uasl_lidar_stereo_gather)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  sensor_msgs 
  geometry_msgs
  uasl_image_acquisition
  pcl_conversions
  uasl_msg_gather
  velodyne_pointcloud
  cv_bridge
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIR}
  LIBRARIES
  CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs uasl_image_acquisition pcl_conversions uasl_msg_gather velodyne_pointcloud cv_bridge
  DEPENDS EIGEN3
)

include_directories(
  include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(get_lidar_stereo_node src/get_lidar_stereo_node.cpp)
target_link_libraries(get_lidar_stereo_node ${catkin_LIBRARIES})

add_executable(get_lidar_stereo_node_separated src/get_lidar_stereo_node_separated.cpp)
target_link_libraries(get_lidar_stereo_node_separated ${catkin_LIBRARIES})

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

