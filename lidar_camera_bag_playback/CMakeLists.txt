cmake_minimum_required(VERSION 2.8.3)
project(lidar_camera_bag_playback)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  visualization_msgs
  cv_bridge
  geometry_msgs
  tf2_geometry_msgs
  h264_bag_playback
  dataset_tools
  custom_point_types
  py_trees_msgs
  tf
  tf2
  gmsl_frame_msg
  tf_conversions
  tf2_msgs
  tf2_ros
  lidar_camera_projection
)

# package currently needs opencv
find_package(OpenCV 3 REQUIRED)


###################################
## catkin specific configuration ##
###################################
catkin_package(
  #INCLUDE_DIRS ${PROJECT_NAME}
  #CATKIN_DEPENDS velodyne_correction
)

###########
## Build ##
###########
include_directories(
  include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(${PROJECT_NAME} src/lidar_camera_bag_playback.cpp)

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES})

#############
## Install ##
#############
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
