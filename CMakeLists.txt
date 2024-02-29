cmake_minimum_required(VERSION 3.0.2)
project(multi_camera_cooperation)

add_compile_options(-std=c++14 -O3 -g)

set(CMAKE_BUILD_TYPE "Release")
set(cv_bridge_DIR ~/catkin_make/src/cv_bridge/cmake)
set(uav_config_DIR /home/nvidia/mulcam_ws/src/uav_config/build)

find_package(OpenCV 4)

message(OpenCV_VERSION '${OpenCV_VERSION}')
message(OpenCV_LIBS '${OpenCV_LIBS}')

SET(CUDA_TOOLKIT_ROOT_DIR "/usr/local/cuda")

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})


find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  rospy
  sensor_msgs
  sensor_msgs
  std_msgs
  visualization_msgs
)
find_package(uav_config REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   sensor_msgs#   sensor_msgs#   std_msgs#   visualization_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS ${catkin_INCLUDE_DIRS}
  CATKIN_DEPENDS roscpp rospy geometry_msgs image_transport #message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ../../devel/include
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${uav_config_INCLUDE_DIRS}
  src
)

# MESSAGE("uav_config_INCLUDE_DIRS": ${uav_config_INCLUDE_DIRS})
# MESSAGE('My OpenCV_INCLUDE_DIRS:'${OpenCV_INCLUDE_DIRS})

# ==================== PnP core library ==================== #
add_library(pnp_ros_lib src/pnp_target_node_ros.cpp)
target_link_libraries(pnp_ros_lib ${OpenCV_LIBS} yaml-cpp)
# ==================== PnP core library ==================== #

# ==================== Executable Program ==================== #

## The main executable program to detect target and estimate it pose
add_executable(multi_camera_cooperation_ros src/multi_camera_cooperation_ros_test.cpp)
target_link_libraries(multi_camera_cooperation_ros pnp_ros_lib ${catkin_LIBRARIES} ${OpenCV_LIBS})

