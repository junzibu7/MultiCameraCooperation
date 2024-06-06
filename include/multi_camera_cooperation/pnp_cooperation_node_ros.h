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
    //======================== tf  =========================//
    tf::TransformBroadcaster* br_base_to_coopestimation = nullptr;
    tf::StampedTransform base_to_coopestimation;
    tf::StampedTransform baseA_to_coopestimation;
    tf::StampedTransform baseB_to_coopestimation;
    tf::StampedTransform baseC_to_coopestimation;
    tf::StampedTransform baseD_to_coopestimation;
    


    tf::StampedTransform base_to_coopestimation_buf;
    tf::TransformListener* lr_base_to_camA = nullptr;
    tf::StampedTransform base_to_camA;
    tf::TransformListener* lr_base_to_camB = nullptr;
    tf::StampedTransform base_to_camB;
    tf::TransformListener* lr_base_to_camC = nullptr;
    tf::StampedTransform base_to_camC;
    tf::TransformListener* lr_base_to_camD = nullptr;
    tf::StampedTransform base_to_camD;
    //======================== tf =========================//

    //==================== estimation process ====================//
    tf::Vector3 t_coopestimation;
    tf::Quaternion q_coopestimation;
    int listenercount = 0;
    bool camAGoodFlag = false;
    bool camBGoodFlag = false;
    bool camCGoodFlag = false;
    bool camDGoodFlag = false;
    //==================== estimation process ====================//

    //==================== transform ====================//
    Eigen::Matrix4Xd T_base_to_coopestimation = Eigen::Matrix4d::Identity();


    Eigen::Matrix3d R_base_coopestimation = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t_base_coopestimation = Eigen::Vector3d::Zero();
    Eigen::Quaterniond q_base_coopestimation = Eigen::Quaterniond::Identity();

    Eigen::Matrix4Xd T_camA_to_estimation = Eigen::Matrix4d::Identity();
    Eigen::Matrix4Xd T_camB_to_estimation = Eigen::Matrix4d::Identity();
    Eigen::Matrix4Xd T_camC_to_estimation = Eigen::Matrix4d::Identity();
    Eigen::Matrix4Xd T_camD_to_estimation = Eigen::Matrix4d::Identity();
    Eigen::Matrix3d R_camA_to_estimation = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d R_camB_to_estimation = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d R_camC_to_estimation = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d R_camD_to_estimation = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t_camA_to_estimation = Eigen::Vector3d::Zero();
    Eigen::Vector3d t_camB_to_estimation = Eigen::Vector3d::Zero();
    Eigen::Vector3d t_camC_to_estimation = Eigen::Vector3d::Zero();
    Eigen::Vector3d t_camD_to_estimation = Eigen::Vector3d::Zero();
    Eigen::Quaterniond q_camA_to_estimation = Eigen::Quaterniond::Identity();
    Eigen::Quaterniond q_camB_to_estimation = Eigen::Quaterniond::Identity();
    Eigen::Quaterniond q_camC_to_estimation = Eigen::Quaterniond::Identity();
    Eigen::Quaterniond q_camD_to_estimation = Eigen::Quaterniond::Identity();
    ros::Time Time_camA_to_estimation;
    ros::Time Time_camB_to_estimation;
    ros::Time Time_camC_to_estimation;
    ros::Time Time_camD_to_estimation;

    Eigen::Matrix4Xd T_base_to_estimationfromcamA = Eigen::Matrix4d::Identity();
    Eigen::Matrix4Xd T_base_to_estimationfromcamB = Eigen::Matrix4d::Identity();
    Eigen::Matrix4Xd T_base_to_estimationfromcamC = Eigen::Matrix4d::Identity();
    Eigen::Matrix4Xd T_base_to_estimationfromcamD = Eigen::Matrix4d::Identity();
    Eigen::Matrix3d R_base_to_estimationfromcamA = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d R_base_to_estimationfromcamB = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d R_base_to_estimationfromcamC = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d R_base_to_estimationfromcamD = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t_base_to_estimationfromcamA = Eigen::Vector3d::Zero();
    Eigen::Vector3d t_base_to_estimationfromcamB = Eigen::Vector3d::Zero();
    Eigen::Vector3d t_base_to_estimationfromcamC = Eigen::Vector3d::Zero();
    Eigen::Vector3d t_base_to_estimationfromcamD = Eigen::Vector3d::Zero();
    Eigen::Quaterniond q_base_to_estimationfromcamA = Eigen::Quaterniond::Identity();
    Eigen::Quaterniond q_base_to_estimationfromcamB = Eigen::Quaterniond::Identity();
    Eigen::Quaterniond q_base_to_estimationfromcamC = Eigen::Quaterniond::Identity();
    Eigen::Quaterniond q_base_to_estimationfromcamD = Eigen::Quaterniond::Identity();

    Eigen::Matrix4Xd T_base_to_camA = Eigen::Matrix4d::Identity();
    Eigen::Matrix4Xd T_base_to_camB = Eigen::Matrix4d::Identity();
    Eigen::Matrix4Xd T_base_to_camC = Eigen::Matrix4d::Identity();
    Eigen::Matrix4Xd T_base_to_camD = Eigen::Matrix4d::Identity();

    Eigen::Matrix3Xd world_to_cameraA_R = Eigen::Matrix3d::Identity();
    Eigen::Matrix3Xd cameraA_to_world_R = Eigen::Matrix3d::Identity();

    Eigen::Matrix3d R_base_to_camA = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d R_base_to_camB = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d R_base_to_camC = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d R_base_to_camD = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t_base_to_camA = Eigen::Vector3d::Zero();
    Eigen::Vector3d t_base_to_camB = Eigen::Vector3d::Zero();
    Eigen::Vector3d t_base_to_camC = Eigen::Vector3d::Zero();
    Eigen::Vector3d t_base_to_camD = Eigen::Vector3d::Zero();
    Eigen::Quaterniond q_base_to_camA = Eigen::Quaterniond::Identity();
    Eigen::Quaterniond q_base_to_camB = Eigen::Quaterniond::Identity();
    Eigen::Quaterniond q_base_to_camC = Eigen::Quaterniond::Identity();
    Eigen::Quaterniond q_base_to_camD = Eigen::Quaterniond::Identity();

    Eigen::Vector3d uav_vicon_pos = Eigen::Vector3d::Zero();
    Eigen::Vector3d frame_vicon_pos = Eigen::Vector3d::Zero();
    
    Eigen::Vector3d vicon_pnp_error = Eigen::Vector3d::Zero();
    //==================== transform ====================//

    //==================== publish & subscribe ====================//
    ros::Publisher pub_base_to_coopestimation;
    ros::Publisher pub_base_to_coopestimation_from_camA;
    ros::Publisher pub_base_to_coopestimation_from_camB;
    ros::Publisher pub_base_to_coopestimation_from_camC;
    ros::Publisher pub_base_to_coopestimation_from_camD;
    ros::Publisher pub_error_camA;
    ros::Publisher pub_error_camD;




    ros::Subscriber sub_camA_to_estimation;
    ros::Subscriber sub_camB_to_estimation;
    ros::Subscriber sub_camC_to_estimation;
    ros::Subscriber sub_camD_to_estimation;
    ros::Subscriber vicon_frame_sub;
    ros::Subscriber uav_vicon_pos_sub;
    

    //===================== message ==============================//
    geometry_msgs::TransformStamped EstimationStamped;
    geometry_msgs::TransformStamped EstimationStampedfromcamA;
    geometry_msgs::TransformStamped EstimationStampedfromcamB;
    geometry_msgs::TransformStamped EstimationStampedfromcamC;
    geometry_msgs::TransformStamped EstimationStampedfromcamD;

    geometry_msgs::TransformStamped error_camA_vector;
    geometry_msgs::TransformStamped error_camD_vector;

    //===================== message ==============================//

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
    void init(ros::NodeHandle &nh, tf::TransformBroadcaster *br, tf::TransformListener *lrA, tf::TransformListener *lrB, tf::TransformListener *lrC, tf::TransformListener *lrD);

    /**
     * @brief camera estimation listen from camA, camB, camC, camD
     */
    void T_base_to_EstimationfromcamA_callback(const geometry_msgs::TransformStamped &base_to_camA);
    void T_base_to_EstimationfromcamB_callback(const geometry_msgs::TransformStamped &base_to_camB);
    void T_base_to_EstimationfromcamC_callback(const geometry_msgs::TransformStamped &base_to_camC);
    void T_base_to_EstimationfromcamD_callback(const geometry_msgs::TransformStamped &base_to_camD);
    void Vicon_pnp_test_frame(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void Vicon_uav_pos(const geometry_msgs::PoseStamped::ConstPtr &msg);


    void toRotationMatrix(float x, float y,float z,float w,Eigen::Matrix3Xd &R);
    /**
     * @brief camera estimation callback from base
     */
    void T_base_to_CoopEstimation();

/**
* @brief broadcast the camera estimation to the base
 */
    void broadcast_base_to_coopestimation(int listenercount);

/**
* @brief fuse the camera estimation from camA, camB, camC, camD
 */   
    void cam_estimation_fuse();
};



#endif //PNP_COOPERATION_NODE_ROS_H