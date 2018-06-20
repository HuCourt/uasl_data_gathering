cmake_minimum_required(VERSION 2.8.3)
project(uasl_msg_gather)

find_package(catkin REQUIRED COMPONENTS cmake_modules message_generation std_msgs geometry_msgs sensor_msgs)
include_directories(${catkin_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_definitions(--std=c++11x)


add_message_files(
  FILES
  Pointcloud2_stereo.msg
)


generate_messages(DEPENDENCIES std_msgs geometry_msgs sensor_msgs)

catkin_package(
  INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR}
  LIBRARIES
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs sensor_msgs
  DEPENDS Eigen
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
