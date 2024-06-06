//
// Created by hzj on 24-3-29.
//
#include <multi_camera_cooperation/pnp_cooperation_node_ros.h>

using namespace std;

PNPCooperationNodeROS pnp_coop_manager;

int main(int argc, char **argv)
{
    ROS_INFO("Initializing MultiCameraCooperation ...");
    ros::init(argc, argv, "multi_cam_cooperation_ros");
    ros::NodeHandle nh("~");
//    ros::NodeHandle nh;
    ros::Rate rate(30);

    tf::TransformBroadcaster* br = new(tf::TransformBroadcaster);
    tf::TransformListener* lrA = new(tf::TransformListener);
    tf::TransformListener* lrB = new(tf::TransformListener);
    tf::TransformListener* lrC = new(tf::TransformListener);
    tf::TransformListener* lrD = new(tf::TransformListener);

    pnp_coop_manager.init(nh, br, lrA, lrB, lrC, lrD);

    while (ros::ok())
    {
        pnp_coop_manager.cam_estimation_fuse();
        ros::spinOnce();
        rate.sleep();
    }
}