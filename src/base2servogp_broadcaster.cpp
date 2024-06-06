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
        base_to_servogroup12.setOrigin(tf::Vector3(-0.951+1.07,-0.693+0.772,-0.2));//rightback
    	// q.setRPY(0, 0, atan(0.45/0.70));
        q.setRPY(0, 5*pi/180, (27.54+4.227)*pi/180.0);

        // q.setRPY(0, 0, pi/2);
		base_to_servogroup12.setRotation(q);
        base_to_servogroup12.stamp_ = ros::Time::now();
        base_to_servogroup12.frame_id_ = "base";
        base_to_servogroup12.child_frame_id_ = "servogroup12";
        tf::transformStampedTFToMsg(base_to_servogroup12, Stampedbase_to_servogroup12);
        pub_base_to_servogroup12.publish(Stampedbase_to_servogroup12);
        brA.sendTransform(Stampedbase_to_servogroup12);
        
        base_to_servogroup34.setOrigin(tf::Vector3(-0.801+1.07,0.876+0.772,-0.2));//rightfront
    	q.setRPY(0, 0, (-40.1+4.227)*pi/180.0);
		base_to_servogroup34.setRotation(q);
        base_to_servogroup34.stamp_ = ros::Time::now();
        base_to_servogroup34.frame_id_ = "base";
        base_to_servogroup34.child_frame_id_ = "servogroup34";
        tf::transformStampedTFToMsg(base_to_servogroup34, Stampedbase_to_servogroup34);
        pub_base_to_servogroup12.publish(Stampedbase_to_servogroup34);
        brA.sendTransform(Stampedbase_to_servogroup34);


        base_to_servogroup56.setOrigin(tf::Vector3(0.886+1.07,0.778+0.772,-0.2));//leftback
    	q.setRPY(0, -5*pi/180, (213.3+4.227)*pi/180.0);
		base_to_servogroup56.setRotation(q);
        base_to_servogroup56.stamp_ = ros::Time::now();
        base_to_servogroup56.frame_id_ = "base";
        base_to_servogroup56.child_frame_id_ = "servogroup56";
        tf::transformStampedTFToMsg(base_to_servogroup56, Stampedbase_to_servogroup56);
        pub_base_to_servogroup12.publish(Stampedbase_to_servogroup56);
        brA.sendTransform(Stampedbase_to_servogroup56);
    
        base_to_servogroup78.setOrigin(tf::Vector3(0.805+1.07,-0.867+0.772,-0.2));//leftfront
    	q.setRPY(0, 0, (141+4.227)*pi/180.0);
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