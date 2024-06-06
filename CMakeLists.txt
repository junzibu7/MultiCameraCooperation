cmake_minimum_required(VERSION 3.0.2)
project(multi_camera_cooperation)

add_compile_options(-std=c++14 -O3 -g)

set(CMAKE_BUILD_TYPE "Release")

# #set(cv_bridge_DIR ~/catkin_make/src/cv_bridge/cmake) #nvidia
# #set(uav_config_DIR /home/nvidia/mulcam_ws/src/uav_config/build)  #nvidia
# set(cv_bridge_DIR /home/hezijia/tool_ws/src/vision_opencv/cv_bridge/cmake)  #laptop
# set(uav_config_DIR /home/hezijia/catkin_ws/src/uav_config/build/catkin_generated/installspace) #laptop
# set(OpenCV_DIR /usr/local/share/opencv4) #laptop

find_package(OpenCV 4.2.0 REQUIRED)

message(OpenCV_VERSION '${OpenCV_VERSION}')
message(OpenCV_LIBS '${OpenCV_LIBS}')

#SET(CUDA_TOOLKIT_ROOT_DIR "/usr/local/cuda")  #nvidia

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})


find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  rospy
  sensor_msgs
  std_msgs
  visualization_msgs
  message_generation
  geometry_msgs
  tf
)
find_package(uav_config REQUIRED)


add_message_files(
        FILES
        landmark.msg
        # camera.msg
)

generate_messages(
        DEPENDENCIES
        geometry_msgs
        std_msgs
)

catkin_package(
  INCLUDE_DIRS ${catkin_INCLUDE_DIRS}
  CATKIN_DEPENDS roscpp rospy geometry_msgs std_msgs image_transport message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ../../devel/include
  include
  include/Filter
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${uav_config_INCLUDE_DIRS}
  src
)

MESSAGE('uav_config_INCLUDE_DIRS:'${uav_config_INCLUDE_DIRS})
MESSAGE('My OpenCV_INCLUDE_DIRS:'${OpenCV_INCLUDE_DIRS})

# ==================== core library ==================== #
add_library(math_tools_lib src/math_tools.cpp)
add_library(ekf_lib src/Filter/extend_kalman.cpp)
add_library(kf_lib src/Filter/kalman_filter.cpp)
add_library(lp_lib src/Filter/lowpass_filter.cpp)
target_link_libraries(ekf_lib kf_lib lp_lib math_tools_lib)


add_library(pnp_ros_lib src/pnp_target_node_ros.cpp)
add_library(landmark_ros_lib src/landmark_extraction_node_ros.cpp)
target_link_libraries(pnp_ros_lib ${OpenCV_LIBS} ${catkin_LIBRARIES} math_tools_lib yaml-cpp)
target_link_libraries(landmark_ros_lib ${OpenCV_LIBS} ${catkin_LIBRARIES} math_tools_lib yaml-cpp)

add_library(pnp_cooperation_ros_lib src/pnp_cooperation_node_ros.cpp)
target_link_libraries(pnp_cooperation_ros_lib ${OpenCV_LIBS} ${catkin_LIBRARIES} math_tools_lib)
# ==================== core library ==================== #

# ==================== Executable Program ==================== #

## The main executable program to detect target and estimate its pose
add_executable(single_cam_process_ros src/single_cam_process.cpp)
target_link_libraries(single_cam_process_ros pnp_ros_lib landmark_ros_lib ${catkin_LIBRARIES} ${OpenCV_LIBS})

## The executable program to broadcast base2servogroup transformation
add_executable(base2servogp_broadcaster_ros src/base2servogp_broadcaster.cpp)
target_link_libraries(base2servogp_broadcaster_ros ${catkin_LIBRARIES})

## The executable program to fuse the pnp estimatetion from multi-cameras
add_executable(multi_cam_cooperation_ros src/multi_cam_cooperation.cpp)
target_link_libraries(multi_cam_cooperation_ros pnp_cooperation_ros_lib ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(cam_pos src/cam_pos.cpp)
target_link_libraries(cam_pos pnp_cooperation_ros_lib ${catkin_LIBRARIES} ${OpenCV_LIBS} ${EIGEN3_LIBRARIES})
