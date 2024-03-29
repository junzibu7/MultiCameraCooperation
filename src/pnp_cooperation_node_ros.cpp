//
// Created by hzj on 24-3-29.
//
#include <multi_camera_cooperation/pnp_cooperation_node_ros.h>

PNPCooperationNodeROS::PNPCooperationNodeROS(){
    // empty
}

void PNPCooperationNodeROS::init(ros::NodeHandle &nh, tf::TransformBroadcaster* br, tf::TransformListener* lrA, tf::TransformListener* lrB, tf::TransformListener* lrC, tf::TransformListener* lrD)
{
    br_base_to_coopestimation = br;
    lr_base_to_estimationfromcamA = lrA;
    lr_base_to_estimationfromcamB = lrB;
    lr_base_to_estimationfromcamC = lrC;
    lr_base_to_estimationfromcamD = lrD;
}

void PNPCooperationNodeROS::cam_estimation_listener()
{
    try
    {
        lr_base_to_estimationfromcamA->lookupTransform("base", "EstimationfromcamA", ros::Time(0), base_to_estimationfromcamA);
        lr_base_to_estimationfromcamB->lookupTransform("base", "EstimationfromcamB", ros::Time(0), base_to_estimationfromcamB);
        lr_base_to_estimationfromcamC->lookupTransform("base", "EstimationfromcamC", ros::Time(0), base_to_estimationfromcamC);
        lr_base_to_estimationfromcamD->lookupTransform("base", "EstimationfromcamD", ros::Time(0), base_to_estimationfromcamD);
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
}

void PNPCooperationNodeROS::broadcast_base_to_coopestimation()
{                                    
    base_to_coopestimation.setOrigin(t_coopestimation);
    base_to_coopestimation.setRotation(q_coopestimation);
    br_base_to_coopestimation->sendTransform(tf::StampedTransform(base_to_coopestimation, ros::Time::now(), "base", "CoopEstimation"));
}

void PNPCooperationNodeROS::cam_estimation_fuse()
{
    t_coopestimation = tf::Vector3((base_to_estimationfromcamA.getOrigin().x() + base_to_estimationfromcamB.getOrigin().x() + base_to_estimationfromcamC.getOrigin().x() + base_to_estimationfromcamD.getOrigin().x()) / 4,
                                   (base_to_estimationfromcamA.getOrigin().y() + base_to_estimationfromcamB.getOrigin().y() + base_to_estimationfromcamC.getOrigin().y() + base_to_estimationfromcamD.getOrigin().y()) / 4,
                                   (base_to_estimationfromcamA.getOrigin().z() + base_to_estimationfromcamB.getOrigin().z() + base_to_estimationfromcamC.getOrigin().z() + base_to_estimationfromcamD.getOrigin().z()) / 4);
    q_coopestimation = tf::Quaternion((base_to_estimationfromcamA.getRotation().w() + base_to_estimationfromcamB.getRotation().w() + base_to_estimationfromcamC.getRotation().w() + base_to_estimationfromcamD.getRotation().w()) / 4,
                                      (base_to_estimationfromcamA.getRotation().x() + base_to_estimationfromcamB.getRotation().x() + base_to_estimationfromcamC.getRotation().x() + base_to_estimationfromcamD.getRotation().x()) / 4,
                                      (base_to_estimationfromcamA.getRotation().y() + base_to_estimationfromcamB.getRotation().y() + base_to_estimationfromcamC.getRotation().y() + base_to_estimationfromcamD.getRotation().y()) / 4,
                                      (base_to_estimationfromcamA.getRotation().z() + base_to_estimationfromcamB.getRotation().z() + base_to_estimationfromcamC.getRotation().z() + base_to_estimationfromcamD.getRotation().z()) / 4);
}