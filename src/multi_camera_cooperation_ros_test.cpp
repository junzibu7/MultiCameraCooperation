//
// Created by hzj on 24-2-28.
//
// #include "multi_camera_cooperation/multi_camera_cooperation_ros_test.h"
#include "multi_camera_cooperation/pnp_target_node_ros.h"
#include "multi_camera_cooperation/landmark_extraction_node_ros.h"

using namespace std;

PnPTargetNodeROS pnp_manager;
LandmarkExtractionNodeROS landmark_manager;

int main(int argc, char **argv)
{
    ROS_INFO("Initializing MultiCameraCooperation ...");
    ros::init(argc, argv, "multi_camera_cooperation_ros");
    ros::NodeHandle nh("~");
//    ros::NodeHandle nh;
    ros::Rate rate(30);
    tf::TransformBroadcaster* br = new(tf::TransformBroadcaster);

    std::cout << "Init MultiCameraCooperation" << std::endl;
    
    landmark_manager.init(nh);
    pnp_manager.init(nh, br);

    ros::spin();
}