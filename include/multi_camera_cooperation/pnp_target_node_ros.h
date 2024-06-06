#ifndef PNP_TARGET_NODE_ROS_H
#define PNP_TARGET_NODE_ROS_H

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


#include <nav_msgs/Odometry.h>




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

#include "uav_config/read_config_drone.h"
#include "multi_camera_cooperation/math_tools.h"

#define USE_4_Point

using namespace std;

class PnPTargetNodeROS;

class PnPTargetNodeROS
{
public:

//==================== cam attribute ====================//
    std::string cam;
//==================== cam attribute ====================//

//==================== ros ====================//
    ros::Subscriber sub_drone_imu;
    ros::Subscriber sub_drone_vicon_pose;
    ros::Subscriber sub_drone_vio_pose;
    ros::Subscriber sub_marker_pixel;
    


    ros::Publisher pub_drone_vicon_pose;
    ros::Publisher pub_relative_pose_mocap;
    ros::Publisher pub_cam_to_estimation;

    std::string marker_pixel_topic;
    ros::Time stamp;
    // cv_bridge::CvImagePtr cv_ptr_compressed_ir;
    // cv_bridge::CvImageConstPtr cv_ptr_raw_ir;
//==================== ros ====================//

//==================== test ===================//
    double sety = 0.0;
//==================== test ===================//

//==================== tf ====================//
    tf::TransformBroadcaster* br0 = nullptr;
    tf::StampedTransform cam_to_estimation;
//==================== tf ====================//

    ///------------- ir_img process -------------///
    std::string write_image_path; //写入图像到本地用于分析
    bool haveWrite = false;
    std::vector<cv::Point2f> marker_pixels;
    std::vector<cv::Point2f> marker_pixels_buffer;
    std::vector<cv::Point2f> marker_pixels_sorted;
    std::vector<cv::Point2f> marker_pixels_up;
    std::vector<cv::Point2f> marker_pixels_down;
    // //dilate
    // cv::Mat erodeElement = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    // cv::Mat dilateElement = getStructuringElement(cv::MORPH_RECT,cv::Size(3, 3));
    // cv::Mat ir_erode;
    // cv::Mat ir_dilate;
    // std::vector<std::vector<cv::Point>> ir_contours;
    // std::vector<std::vector<cv::Point>> ir_contours_final;

    // ///------------- optical flow -------------///
    // cv::Mat prevImg;
    // cv::Mat nextImg;
    // std::vector<cv::Point2f> prevImgPts;
    // std::vector<cv::Point2f> nextImgPts;
    // std::vector<cv::Point2f> pnpImgPts;
    // std::vector<uchar> status;
    // std::vector<float> err;
    // cv::Rect opti_trust_region = cv::Rect(0,0,640,480);//默认参数，opti Track 目标后，将目标外沿选取更大的一片可信区域，时刻随目标变化而变化;
    // cv::TermCriteria termcrit = cv::TermCriteria(cv::TermCriteria::EPS, 100, 0.001);
    // cv::Size winSize = cv::Size(25, 25);
    // int win_width = 4;//用于对光流追踪后的点，在光点附近win_width的窗口内进一步取中心点坐标

    ///------------- pnp -------------///
    int landmark_num;
    std::vector<cv::Point3f> drone_landmarks_cv;
    cv::Mat cameraMatrix = cv::Mat::zeros(3,3,CV_32FC1);
    std::vector<double> distCoeffs;
    struct Pose{
        Eigen::Vector3d pos;
        Eigen::Vector3d euler;
        Eigen::Quaterniond Quat;
    };
    cv::Vec3d outputRvecRaw, outputTvecRaw;
    bool pnpGoodFlag = false;
    double PnP_time_delay = 0.0;

    ///------------- logical & visualize -------------///
    // float smooth_threshold = 2;
    // bool got_attitude_init = false;
    // bool vision_prepare_ok = false;
    // bool roiGoodFlag = false;
    // bool opticalReadyFlag = false;
    // bool opticalGoodFlag = false;


    ///------------ drone state ----------///
    std::shared_ptr<ConfigParser> uav_config = nullptr;

    Pose drone_pose, drone_pose_vicon_init;
    Pose drone_pose_vicon, base_pose_vicon;
//    Attitude drone_attitude_init;
    Eigen::Quaterniond drone_attitude, drone_attitude_init;
    bool drone_attitude_init_flag = true;
    Eigen::Vector3d drone_acc;
    bool drone_pose_vicon_first_flag = true;
    geometry_msgs::PoseStamped drone_pose_msg;
    Eigen::Vector3d target_pos_in_mocap; //mocap下target的绝对位置，用于检验PnP估计的准确性。
    Eigen::Vector3d target_pos_in_img;
    Eigen::Quaterniond target_q_in_img;
    //T_A_to_B is from A transform to B, also means B pose in A
    Eigen::Matrix4d T_IRLandmark_to_drone = Eigen::Matrix4d::Identity(); //目标飞机drone在landmark目标坐标系下的姿态
    Eigen::Matrix4d T_image_to_markers = Eigen::Matrix4d::Identity(); //landmark目标在camera坐标系下的姿态
    deque<Eigen::Matrix4d> window_T_image_to_markers; // 滑窗实现“低通”滤波
    Eigen::Matrix4d T_camera_to_image = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d T_cam_to_estimation = Eigen::Matrix4d::Identity(); 
    Eigen::Matrix4d T_body_to_drone = Eigen::Matrix4d::Identity(); //drone在body坐标系下的姿态
    Eigen::Matrix3d R_body_to_drone = Eigen::Matrix3d::Identity(); 
    Eigen::Matrix3d R_cam_to_estimation = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t_body_to_drone = Eigen::Vector3d::Zero();

    Eigen::Matrix3Xd world_to_cameraA_R = Eigen::Matrix3d::Identity();
    Eigen::Matrix3Xd cameraA_to_world_R = Eigen::Matrix3d::Identity();

    Eigen::Vector3d uav_vicon_pos = Eigen::Vector3d::Zero();
    Eigen::Vector3d frame_vicon_pos = Eigen::Vector3d::Zero();
    
    Eigen::Vector3d vicon_pnp_error = Eigen::Vector3d::Zero();

    ros::Subscriber vicon_frame_sub;
    ros::Subscriber uav_vicon_pos_sub;

    tf::Vector3 t_cam_to_estimation = tf::Vector3();
    tf::Quaternion q_body_to_drone = tf::Quaternion();
    tf::Quaternion q_cam_to_estimation = tf::Quaternion();
    geometry_msgs::PoseStamped msg_target_pose_from_img;
    geometry_msgs::TransformStamped msg_T_cam_to_estimation;

    ///------------ Complementary filter for calculate position velocity state ----------///
    // std::shared_ptr<ComplementaryFilter> filter;

    ///------------- function declarations -------------///
    PnPTargetNodeROS();

    /**
     * @brief Load config files, initialize the ros wrapper and related
     * @param nh
     * @param br
     */
    // void init(ros::NodeHandle &nh);
    void init(ros::NodeHandle &nh, tf::TransformBroadcaster* br);
 
    /**
     * @brief read image from local path and test
     * @param img_path
     */
    void test_img_local(std::string img_path);

    /**
     * @brief The callback from drone pose of VIO or Mocap.
     * @param msg
     */    
    void drone_vicon_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void drone_vio_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void base_vicon_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void base_vio_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);

    void Vicon_pnp_test_frame(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void Vicon_uav_pos(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void toRotationMatrix(float x, float y,float z,float w,Eigen::Matrix3Xd &R);


    /**
     * @brief The callback from marker_pixels
     * @param msg
     */    
    void ir_marker_pixel_cb(const multi_camera_cooperation::landmark::ConstPtr &msg);

    /**
     * IMU callback from Xsens or Pixhawk
     * @param msg
     */
    void drone_imu_cb(const sensor_msgs::Imu::ConstPtr &msg);

    /**
     * @brief using ir_img and rect ROI to detect infrared landmark, and calculate the relative pose of target in img.
     */
    void landmark_pose_solve();

    /**
     * @brief match IRlandmarks with the shape of T.
     * @param pointsVector
     */
    bool T_shape_identify(vector<cv::Point2f> &pointsVector);
    bool Square_shape_identity(vector<cv::Point2f> &pointsVector);
    bool pnp_process(std::vector<cv::Point2f> &pointsVector);
    std::string Convert(float Num);
    
};

fstream t_cam_to_estimation_file;
fstream q_cam_to_estimation_file;


#endif //PNP_TARGET_NODE_ROS_H