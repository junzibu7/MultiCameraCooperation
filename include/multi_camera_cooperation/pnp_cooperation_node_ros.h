#ifndef PNP_COOPERATION_NODE_ROS_H
#define PNP_COOPERATION_NODE_ROS_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Imu.h>
#include <image_transport/image_transport.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_datatypes.h> 

#include <Eigen/Eigen>
#include <Eigen/Dense>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/video/tracking.hpp>
#include <cv_bridge/cv_bridge.h>

#include <multi_camera_cooperation/colors.h>
#include <multi_camera_cooperation/landmark.h>

#include <cstdlib>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <cstdio>
#include <fstream>
#include <cmath>
#include <ctime>
#include <queue>
#include <vector>
#include <chrono>
#include <sstream>

#include "multi_camera_cooperation/math_tools.h"

#define IMG_COMPRESSED
#define SHOW_ORIGIN_IMG
#define ENABLE_VISUALIZATION

using namespace std;



class PNPCooperationNodeROS;

class PNPCooperationNodeROS
{
public:
//==================== tf  ====================//
    tf::TransformBroadcaster* br_base_to_coopestimation = nullptr;
    tf::Transform base_to_coopestimation;
    tf::TransformListener* lr_base_to_estimationfromcamA = nullptr;
    tf::StampedTransform base_to_estimationfromcamA;
    tf::TransformListener* lr_base_to_estimationfromcamB = nullptr;
    tf::StampedTransform base_to_estimationfromcamB;
    tf::TransformListener* lr_base_to_estimationfromcamC = nullptr;
    tf::StampedTransform base_to_estimationfromcamC;
    tf::TransformListener* lr_base_to_estimationfromcamD = nullptr;
    tf::StampedTransform base_to_estimationfromcamD;
//==================== tf ====================//

//==================== estimation process ====================//
    tf::Vector3 t_coopestimation;
    tf::Quaternion q_coopestimation;
//==================== estimation process ====================//

///------------- function declarations -------------///
    PNPCooperationNodeROS();

/**
 * @brief Load config files, initialize the ros wrapper and related
 * @param nh
 * @param br
 * @param lrA
 * @param lrB
 * @param lrC
 * @param lrD
 */
    void init(ros::NodeHandle &nh, tf::TransformBroadcaster* br, tf::TransformListener* lrA, tf::TransformListener* lrB, tf::TransformListener* lrC, tf::TransformListener* lrD);
 
/**
 * @brief read camera estimation from camA, camB, camC, camD
 */
    void cam_estimation_listener();

/**
* @brief broadcast the camera estimation to the base
 */
    void broadcast_base_to_coopestimation();

/**
* @brief fuse the camera estimation from camA, camB, camC, camD
 */   
    void cam_estimation_fuse();
};



#endif //PNP_COOPERATION_NODE_ROS_H