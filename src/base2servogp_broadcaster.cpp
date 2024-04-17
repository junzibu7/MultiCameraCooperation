//
// Created by hzj on 24-3-16.
//
#include <math.h>
#include "multi_camera_cooperation/math_tools.h"

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PointStamped.h>

tf::Quaternion q;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "base2servogp_broadcaster");
	ros::NodeHandle nh;

    tf::TransformBroadcaster brA;
    tf::TransformBroadcaster brB;
    tf::TransformBroadcaster brC;
    tf::TransformBroadcaster brD;

    tf::StampedTransform base_to_servogroup12;
    tf::StampedTransform base_to_servogroup34;
    tf::StampedTransform base_to_servogroup56;
    tf::StampedTransform base_to_servogroup78;

    geometry_msgs::TransformStamped Stampedbase_to_servogroup12;
    geometry_msgs::TransformStamped Stampedbase_to_servogroup34;
    geometry_msgs::TransformStamped Stampedbase_to_servogroup56;
    geometry_msgs::TransformStamped Stampedbase_to_servogroup78;

    ros::Publisher pub_base_to_servogroup12 = nh.advertise<geometry_msgs::TransformStamped>("T_base_to_servogroup12", 1);
    ros::Publisher pub_base_to_servogroup34 = nh.advertise<geometry_msgs::TransformStamped>("T_base_to_servogroup34", 1);
    ros::Publisher pub_base_to_servogroup56 = nh.advertise<geometry_msgs::TransformStamped>("T_base_to_servogroup56", 1);
    ros::Publisher pub_base_to_servogroup78 = nh.advertise<geometry_msgs::TransformStamped>("T_base_to_servogroup78", 1);


    ros::Rate rate(30);

    while (ros::ok())
    {
        base_to_servogroup12.setOrigin(tf::Vector3(0.0, 0.0, 0.0));//rightback
    	q.setRPY(0, 0, atan(0.45/0.70));
		base_to_servogroup12.setRotation(q);
        base_to_servogroup12.stamp_ = ros::Time::now();
        base_to_servogroup12.frame_id_ = "base";
        base_to_servogroup12.child_frame_id_ = "servogroup12";
        tf::transformStampedTFToMsg(base_to_servogroup12, Stampedbase_to_servogroup12);
        pub_base_to_servogroup12.publish(Stampedbase_to_servogroup12);
        brA.sendTransform(Stampedbase_to_servogroup12);
        


        base_to_servogroup34.setOrigin(tf::Vector3(1.383 + 0.263, 0.932 + 0.680, 0.0));//leftfront
    	q.setRPY(0, 0, -pi/2);
		base_to_servogroup34.setRotation(q);
        base_to_servogroup34.stamp_ = ros::Time::now();
        base_to_servogroup34.frame_id_ = "base";
        base_to_servogroup34.child_frame_id_ = "servogroup34";
        tf::transformStampedTFToMsg(base_to_servogroup34, Stampedbase_to_servogroup34);
        pub_base_to_servogroup12.publish(Stampedbase_to_servogroup34);
        brA.sendTransform(Stampedbase_to_servogroup34);


        base_to_servogroup56.setOrigin(tf::Vector3(-0.320 + 0.263, 0.742 + 0.680, 0.0));//leftback
    	q.setRPY(0, 0, -atan(0.45/0.70));
		base_to_servogroup56.setRotation(q);
        base_to_servogroup56.stamp_ = ros::Time::now();
        base_to_servogroup56.frame_id_ = "base";
        base_to_servogroup56.child_frame_id_ = "servogroup56";
        tf::transformStampedTFToMsg(base_to_servogroup56, Stampedbase_to_servogroup56);
        pub_base_to_servogroup12.publish(Stampedbase_to_servogroup56);
        brA.sendTransform(Stampedbase_to_servogroup56);
    

        base_to_servogroup78.setOrigin(tf::Vector3(1.35 + 0.263, -0.898 + 0.680, 0.0));//rightfront
    	q.setRPY(0, 0, pi/2);
		base_to_servogroup78.setRotation(q);
        base_to_servogroup78.stamp_ = ros::Time::now();
        base_to_servogroup78.frame_id_ = "base";
        base_to_servogroup78.child_frame_id_ = "servogroup78";
        tf::transformStampedTFToMsg(base_to_servogroup78, Stampedbase_to_servogroup78);
        pub_base_to_servogroup12.publish(Stampedbase_to_servogroup78);
        brA.sendTransform(Stampedbase_to_servogroup78);

        ros::spinOnce();
		rate.sleep();
    }
    ros::Duration(1.0).sleep();
	
	return 0;
    
}