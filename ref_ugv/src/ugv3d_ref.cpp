#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/PoseStamped.h>
#include <sstream>
#include <iostream>
#include <fstream>
#include <sstream>
#include <std_srvs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <std_msgs/Float64.h>


ros::Publisher ugvOdom_pub,laserOdom_pub;


void convert_odom(const nav_msgs::Odometry::ConstPtr &msg)
{

    tf::Quaternion quat_odom;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, quat_odom);
    tf::Transform body2world;
    body2world.setRotation(quat_odom);
    body2world.setOrigin(tf::Vector3(msg->pose.pose.position.x,
                                     msg->pose.pose.position.y,
                                     msg->pose.pose.position.z));
    // configuration on UGV
    tf::Vector3 laser_offset(0,0,0.6);
    tf::Vector3 ugv_offset(-0.4,0,0);
    tf::Vector3 laser_pos = body2world*laser_offset;
    tf::Vector3 ugv_pos = body2world*ugv_offset;

    nav_msgs::Odometry ugv_odom,laser_odom;
    laser_odom.header.frame_id = "world";
    laser_odom.header.stamp = msg->header.stamp;
    laser_odom.pose.pose.orientation = msg->pose.pose.orientation;
    laser_odom.pose.pose.position.x = laser_pos.x();
    laser_odom.pose.pose.position.y = laser_pos.y();
    laser_odom.pose.pose.position.z = laser_pos.z();
    laserOdom_pub.publish(laser_odom);
    ugv_odom.header.frame_id = "world";
    ugv_odom.header.stamp = msg->header.stamp;
    ugv_odom.pose.pose.orientation = msg->pose.pose.orientation;
    ugv_odom.pose.pose.position.x = ugv_pos.x();
    ugv_odom.pose.pose.position.y = ugv_pos.y();
    ugv_odom.pose.pose.position.z = ugv_pos.z();
    ugvOdom_pub.publish(ugv_odom);

}



int main(int argc, char **argv)
{

    ros::init(argc, argv, "msg_rangs");
    ros::NodeHandle nh;
    ros::Subscriber state_sub = nh.subscribe("/odometry", 1, convert_odom);
    ugvOdom_pub = nh.advertise<nav_msgs::Odometry>("/UgvOdomTopic",1);
    laserOdom_pub = nh.advertise<nav_msgs::Odometry>("/LaserOdomTopic",1);

    ros::spin();

    return 0;
}
