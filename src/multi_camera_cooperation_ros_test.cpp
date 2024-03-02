//
// Created by hzj on 24-2-28.
//
#include <multi_camera_cooperation/multi_camera_cooperation_ros_test.h>

using namespace std;

int main(int argc, char **argv)
{
    ROS_INFO("Initializing MultiCameraCooperation ...");
    ros::init(argc, argv, "multi_camera_cooperation_ros");
    ros::NodeHandle nh("~");
//    ros::NodeHandle nh;
    ros::Rate rate(30);

    std::cout << "Init MultiCameraCooperation" << std::endl;

    ros::spin();
}