#ifndef PNP_TARGET_ROS_H
#define PNP_TARGET_ROS_H

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
#include <multi_camera_cooperation/Markers.h>

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

#define IMG_COMPRESSED
#define SHOW_ORIGIN_IMG
#define USE_4_Point

using namespace std;

class PnPTargetNodeROS;

class PnPTargetNodeROS
{
public:

//==================== ros ====================//
    ros::Subscriber sub_drone_imu;
    ros::Subscriber sub_drone_vicon_pose;
    ros::Subscriber sub_drone_vio_pose;
    ros::Subscriber sub_ir_img;

    ros::Publisher pub_drone_vicon_pose;
    ros::Publisher pub_ir_img;
    ros::Publisher pub_ir_show_img;
    ros::Publisher pub_ir_binary_img;
    ros::Publisher pub_ir_erode_dilate_img;
    ros::Publisher pub_ir_contours_img;
    ros::Publisher pub_marker_pixel;
    ros::Publisher pub_target_pose_in_body;
    ros::Publisher pub_relative_pose_mocap;
    std::string marker_pixel_topic;
    ros::Time stamp;
    cv_bridge::CvImagePtr cv_ptr_compressed_ir;
    cv_bridge::CvImageConstPtr cv_ptr_raw_ir;
//==================== ros ====================//

    ///------------- ir_img process -------------///
    cv::Mat ir_img;
    cv::Mat ir_img_color_show;
    cv::Mat ir_binary;
    int ir_binary_threshold = 50; //二值化阈值
    std::string write_image_path; //写入图像到本地用于分析
    bool haveWrite = false;
    std::vector<cv::Point2f> marker_pixels;
    std::vector<cv::Point2f> marker_pixels_sorted;
    //dilate
    cv::Mat erodeElement = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::Mat dilateElement = getStructuringElement(cv::MORPH_RECT,cv::Size(3, 3));
    cv::Mat ir_erode;
    cv::Mat ir_dilate;
    std::vector<std::vector<cv::Point>> ir_contours;
    std::vector<std::vector<cv::Point>> ir_contours_final;

    ///------------- optical flow -------------///
    cv::Mat prevImg;
    cv::Mat nextImg;
    std::vector<cv::Point2f> prevImgPts;
    std::vector<cv::Point2f> nextImgPts;
    std::vector<cv::Point2f> pnpImgPts;
    std::vector<uchar> status;
    std::vector<float> err;
    cv::Rect opti_trust_region = cv::Rect(0,0,640,480);//默认参数，opti Track 目标后，将目标外沿选取更大的一片可信区域，时刻随目标变化而变化;
    cv::TermCriteria termcrit = cv::TermCriteria(cv::TermCriteria::EPS, 100, 0.001);
    cv::Size winSize = cv::Size(25, 25);
    int win_width = 4;//用于对光流追踪后的点，在光点附近win_width的窗口内进一步取中心点坐标

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

    ///------------- logical & visualize -------------///
    float smooth_threshold = 2;
    bool got_attitude_init = false;
    bool vision_prepare_ok = false;
    bool roiGoodFlag = false;
    bool opticalReadyFlag = false;
    bool opticalGoodFlag = false;
    bool pnpGoodFlag = false;
    double PnP_time_delay = 0.0;

    ///------------ drone state ----------///
    std::shared_ptr<ConfigParser> uav_config = nullptr;

    Pose drone_pose, drone_pose_vicon_init;
    Pose drone_pose_vicon, body_pose_vicon;
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
    Eigen::Matrix4d T_markers_to_drone = Eigen::Matrix4d::Identity(); //目标飞机drone在landmark目标坐标系下的姿态
    Eigen::Matrix4d T_camera_to_markers = Eigen::Matrix4d::Identity(); //landmark目标在camera坐标系下的姿态
    Eigen::Matrix4d T_body_to_camera = Eigen::Matrix4d::Identity();  
    Eigen::Matrix4d T_body_to_drone = Eigen::Matrix4d::Identity(); //drone在body坐标系下的姿态
    Eigen::Matrix3d R_body_to_drone = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t_body_to_drone = Eigen::Vector3d::Zero();
    Eigen::Quaterniond q_body_to_drone = Eigen::Quaterniond::Identity();
    geometry_msgs::PoseStamped msg_target_pose_from_img;

    ///------------ Complementary filter for calculate position velocity state ----------///
    // std::shared_ptr<ComplementaryFilter> filter;

    ///------------- function declarations -------------///
    PnPTargetNodeROS();

    /**
     * @brief Load config files, initialize the ros wrapper and related
     * @param nh
     */
    void init(ros::NodeHandle &nh);

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

    /**
     * IMU callback from Xsens or Pixhawk
     * @param msg
     */
    void drone_imu_cb(const sensor_msgs::Imu::ConstPtr &msg);

    void ir_compressed_img_cb(const sensor_msgs::CompressedImage::ConstPtr &msg);
    void ir_raw_img_cb(const sensor_msgs::Image::ConstPtr &msg);

    /**
     * @brief using ir_img and rect ROI to detect infrared landmark, and calculate the relative pose of target in img.
     */
    void landmark_pose_solve();

    /**
     * @brief find ir features in img.
     * @param model model1为角点检测方法||model2为二值化方法
     */
    bool ir_img_process(cv::Mat &_ir_img, vector<cv::Point2f> &pointsVector, int model);

    geometry_msgs::Quaternion euler2quaternion(float roll, float pitch, float yaw);
    Eigen::Quaterniond euler2quaternion_eigen(float roll, float pitch, float yaw);
    Eigen::Vector3d  quaternion2euler(float x, float y, float z, float w);
    bool optical_flow(cv::Mat &frame, vector<cv::Point2f> &pointsVector);
    bool extractFeatures(cv::Mat &frame, vector<cv::Point2f> &pointsVector);
    void broadcast_marker_pixel(vector<cv::Point2f> &pointsVector);
    float yaw_esti_pixel_angle_process(vector<cv::Point2f> &pointsVector, vector<cv::Point2f> &pointsVectorNeighbour, ConfigParser &uav, ConfigParser &uav_neighbour);
    bool pnp_process(std::vector<cv::Point2f> &pointsVector);
    bool refine_pixel(std::vector<cv::Point2f> &pts_raw, std::vector<cv::Point2f> &pts_refine, cv::Mat &img);
    std::string Convert(float Num);
    void getEulerAngles(cv::Vec3d &rvec, Eigen::Vector3d &eulerAngles, Eigen::Quaterniond &q);
    
};


#endif //PNP_TARGET_ROS_H