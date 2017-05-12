cmake_minimum_required(VERSION 2.8.3)
project(skeleton_tracker)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  kdl_conversions
  openni2_camera
  pcl_conversions
  pcl_ros
  roscpp
  roslib
  tf
  message_generation
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_message_files(
   FILES
   user_IDs.msg
)

add_service_files(
   FILES
   skeletonsrv.srv
)

generate_messages(
    DEPENDENCIES geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES skeleton_tracker
  CATKIN_DEPENDS cv_bridge geometry_msgs image_transport kdl_conversions openni2_camera pcl_conversions pcl_ros roscpp roslib tf message_runtime
  DEPENDS system_lib opencv
)

set(OPENNI2_LIB_DIR $ENV{OPENNI2_REDIST})
set(OPENNI2_INCLUDE_DIR $ENV{OPENNI2_INCLUDE})
set(NITE2_LIB_DIR $ENV{NITE2_REDIST64})
set(NITE2_INCLUDE_DIR $ENV{NITE2_INCLUDE})

##message(WARNING ${OpenCV_LIBRARIES})

link_directories(${OPENNI2_LIB_DIR})
link_directories(${NITE2_LIB_DIR})

include_directories(${OPENNI2_INCLUDE_DIR})
include_directories(${NITE2_INCLUDE_DIR})

add_executable(xtion_tracker src/xtion_tracker_node.cpp)
add_dependencies(xtion_tracker skeleton_tracker_gencpp)  # Generate messages first

target_link_libraries(xtion_tracker OpenNI2 NiTE2 ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
add_executable(skeleton_tracker src/skeleton_tracker_node.cpp)

add_dependencies(skeleton_tracker skeleton_tracker_gencpp)  # Generate messages first
target_link_libraries(skeleton_tracker OpenNI2 NiTE2 ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

add_custom_command(TARGET skeleton_tracker POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E create_symlink ${NITE2_LIB_DIR}/NiTE2 "~/.ros/NiTE2"
  )
