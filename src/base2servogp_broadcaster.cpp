//
// Created by hzj on 24-3-16.
//
#include <math.h>

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

    tf::Transform base_to_servogroup12;
    tf::Transform base_to_servogroup34;
    tf::Transform base_to_servogroup56;
    tf::Transform base_to_servogroup78;

    ros::Rate rate(30);

    while (ros::ok())
    {
        base_to_servogroup12.setOrigin(tf::Vector3(0, 0, 0.0));
    	q.setRPY(0, 0, 0);
		base_to_servogroup12.setRotation(q);
		brA.sendTransform(tf::StampedTransform(base_to_servogroup12, ros::Time::now(), "base", "servogroup12"));

        base_to_servogroup34.setOrigin(tf::Vector3(0, 0, 0.0));
    	q.setRPY(0, 0, 0);
		base_to_servogroup34.setRotation(q);
		brB.sendTransform(tf::StampedTransform(base_to_servogroup34, ros::Time::now(), "base", "servogroup34"));

        base_to_servogroup56.setOrigin(tf::Vector3(0, 0, 0.0));
    	q.setRPY(0, 0, 0);
		base_to_servogroup56.setRotation(q);
		brC.sendTransform(tf::StampedTransform(base_to_servogroup56, ros::Time::now(), "base", "servogroup56"));

        base_to_servogroup78.setOrigin(tf::Vector3(0, 0, 0.0));
    	q.setRPY(0, 0, 0);
		base_to_servogroup78.setRotation(q);
		brD.sendTransform(tf::StampedTransform(base_to_servogroup78, ros::Time::now(), "base", "servogroup78"));

        ros::Duration(0.03).sleep();
    }
    ros::Duration(1.0).sleep();
	
	return 0;
    
}