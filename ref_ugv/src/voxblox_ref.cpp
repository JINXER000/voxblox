#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <sstream>
#include <iostream>
#include <fstream>
#include <sstream>
#include <std_srvs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <std_msgs/Float64.h>


ros::Publisher ugvOdom_pub,laserOdom_pub, transform_pub;


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
    tf::Vector3 laser_offset(0,0,0.85);
    tf::Vector3 ugv_offset(-0.25,0,0);
    tf::Vector3 laser_pos = body2world*laser_offset;
    tf::Vector3 ugv_pos = body2world*ugv_offset;

    // nav_msgs::Odometry ugv_odom,laser_odom;
    // laser_odom.header.frame_id = "world";
    // laser_odom.header.stamp = msg->header.stamp;
    // laser_odom.pose.pose.orientation = msg->pose.pose.orientation;
    // laser_odom.pose.pose.position.x = laser_pos.x();
    // laser_odom.pose.pose.position.y = laser_pos.y();
    // laser_odom.pose.pose.position.z = laser_pos.z();
    // laserOdom_pub.publish(laser_odom);
    // ugv_odom.header.frame_id = "world";
    // ugv_odom.header.stamp = msg->header.stamp;
    // ugv_odom.pose.pose.orientation = msg->pose.pose.orientation;
    // ugv_odom.pose.pose.position.x = ugv_pos.x();
    // ugv_odom.pose.pose.position.y = ugv_pos.y();
    // ugv_odom.pose.pose.position.z = ugv_pos.z();
    // ugvOdom_pub.publish(ugv_odom);

    geometry_msgs::TransformStamped transform_;
    transform_.header.frame_id = "world";
    transform_.header.stamp = msg->header.stamp;
    transform_.transform.rotation = msg->pose.pose.orientation;
    transform_.transform.translation.x = laser_pos.x();
    transform_.transform.translation.y = laser_pos.y();
    transform_.transform.translation.z = laser_pos.z()*1.7;
    transform_pub.publish(transform_);

}



int main(int argc, char **argv)
{

    ros::init(argc, argv, "msg_rangs");
    ros::NodeHandle nh;
//    ros::Subscriber state_sub = nh.subscribe("/LaserOdomTopic", 1, convert_odom);
//    transform_pub = nh.advertise<geometry_msgs::TransformStamped>("/LaserTransform", 1);
    ros::Subscriber state_sub = nh.subscribe("/iris_0/mavros/local_position/odom", 1, convert_odom);
    transform_pub = nh.advertise<geometry_msgs::TransformStamped>("/iris_transform", 1);

    ros::spin();

    return 0;
}
