//
// Created by hzj on 24-2-28.
//
#include <multi_camera_fusion/multi_camera_fusion_ros_test.h>

using namespace std;

int main(int argc, char **argv)
{
    ROS_INFO("Initializing MultiCameraFusion ...");
    ros::init(argc, argv, "multi_camera_fusion_ros");
    ros::NodeHandle nh("~");
//    ros::NodeHandle nh;
    ros::Rate rate(30);

    std::cout << "Init MultiCameraFusion" << std::endl;

    ros::spin();
}