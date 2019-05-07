cmake_minimum_required(VERSION 2.8.3)
project(rgbd_calibration)

add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Release)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  sensor_msgs
  pcl_conversions
  pcl_ros
  message_generation
)

## System dependencies are found with CMake's conventions
# pcl
set(PCL_DIR "/usr/share/pcl-1.7/PCLConfig.cmake")
find_package( PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  RgbdMsg.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rgbd_calibration
#  CATKIN_DEPENDS roscpp rospy tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)


## Declare a C++ executable
add_executable(rgbd_calibration_node src/rgbd_calib.cpp)
target_link_libraries(rgbd_calibration_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(rgbd_calibration_node  rgbd_calibration_message_cpp)

