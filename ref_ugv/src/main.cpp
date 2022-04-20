#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/PoseStamped.h>
#include <gazebo_msgs/SetModelState.h>
#include <sstream>
#include <iostream>
#include <fstream>
#include <sstream>
#include <std_srvs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <tf/tf.h>
#include <std_msgs/Float64.h>

//#define DIRECTSET
ros::Timer shift_timer;
ros::Publisher pose_pub,cmd_pub,height_pub,odom_pub;
double shift = 0.0;
int cyc = 0;
bool got_ground_truth= false;
void updateShift(const ros::TimerEvent&)
{
  shift = 0;//1.5*sin(2*M_PI*0.004*static_cast<double>(cyc)*0.05)+static_cast<double>(cyc)*0.05*0.01;
  cyc++;
}

void get_state(const nav_msgs::Odometry::ConstPtr &msg)
{
  geometry_msgs::PoseStamped reb;
  std_msgs::Float64 laser_height;
  reb.pose = msg->pose.pose;

  laser_height.data = reb.pose.position.z;
  height_pub.publish(laser_height);

  reb.pose.position.z -= shift;
  reb.header.frame_id = "world";
  reb.header.stamp=msg->header.stamp;
  // pose_pub.publish(reb);

  // pub odom
  nav_msgs::Odometry odom;
  odom.header.frame_id = "world";
  odom.header.stamp=msg->header.stamp;
  odom.pose = msg->pose;
  odom_pub.publish(odom);

  got_ground_truth = true;
}

void get_transform(const geometry_msgs::TransformStamped::ConstPtr &msg)
{


  geometry_msgs::PoseStamped reb;
  std_msgs::Float64 laser_height;
  reb.pose.position.x=msg->transform.translation.x;
  reb.pose.position.y=msg->transform.translation.y;
  reb.pose.position.z=msg->transform.translation.z;
  reb.pose.orientation=msg->transform.rotation;

  // pub odom
  nav_msgs::Odometry odom;
  odom.header.frame_id = "world";
  odom.header.stamp=msg->header.stamp;
  odom.pose.pose = reb.pose;
  odom_pub.publish(odom);
}
void get_state_pos(const geometry_msgs::PoseStamped::ConstPtr &msg)
{

  if(got_ground_truth)
    return;

  geometry_msgs::PoseStamped reb;
  std_msgs::Float64 laser_height;
  reb.pose = msg->pose;

  laser_height.data = reb.pose.position.z;
  height_pub.publish(laser_height);

  reb.pose.position.z -= shift;
  reb.header.frame_id = "world";
  reb.header.stamp=msg->header.stamp;
  // pose_pub.publish(reb);

  // pub odom
  nav_msgs::Odometry odom;
  odom.header.frame_id = "world";
  odom.header.stamp=msg->header.stamp;
  odom.pose.pose = msg->pose;
  odom_pub.publish(odom);
}

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Publisher image_info_pub_;
  int depth_rows,depth_cols;
  float fx,fy,cx,cy;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/depth/image_raw", 1,
                               &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("revised_sensor/image", 1);
    image_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>("revised_sensor/camera_info",1);

    // Load simulated camera Info from param
    nh_.param<float>("/ref/cx", cx, 320);
    nh_.param<float>("/ref/cy", cy, 240);
    nh_.param<float>("/ref/fx", fx, 337.209);
    nh_.param<float>("/ref/fy", fy, 337.209);
    nh_.param<int>("/ref/depth_rows", depth_rows, 480);
    nh_.param<int>("/ref/depth_cols", depth_cols, 640);

  }


  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    // printf("get img");
    for(int i=0;i<cv_ptr->image.rows;i++)
    {
      for (int j=0;j<cv_ptr->image.cols;j++)
      {

        if(std::isnan(cv_ptr->image.at<float>(i,j)))
        {
          cv_ptr->image.at<float>(i,j)=INFINITY;
        }

      }
    }
    image_pub_.publish(cv_ptr->toImageMsg());

    // publish the camera info
    sensor_msgs::CameraInfo camInfo;
    camInfo.height = depth_rows;
    camInfo.width = depth_cols;
    camInfo.K[2] = cx;
    camInfo.K[5] = cy;
    camInfo.K[0] = fx;
    camInfo.K[4] = fy;

    image_info_pub_.publish(camInfo);
  }
};

int main(int argc, char **argv)
{

  ros::init(argc, argv, "msg_rangs");
  ros::NodeHandle nh;
  ros::Subscriber state_sub = nh.subscribe("/ground_truth/state", 1, get_state);
  ros::Subscriber state_pos_sub = nh.subscribe("mavros/local_position/pose", 1, get_state_pos);
  ros::Subscriber transorm_sub = nh.subscribe("/kinect/vrpn_client/estimated_transform", 1, get_transform);
  pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/position/local",1);
  odom_pub = nh.advertise<nav_msgs::Odometry>("odom",1);
  height_pub = nh.advertise<std_msgs::Float64>("/mavros/height/laser",1);
  //ros::Subscriber cmd_sub = nh.subscribe("rt_ref_gen/current_state", 1, get_cmd);
  ImageConverter ic;
#ifndef DIRECTSET
  cmd_pub = nh.advertise<geometry_msgs::PoseStamped>("/command/pose",1);
#else
  cmd_pub = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1);
#endif

  shift_timer = nh.createTimer(ros::Duration(0.05), updateShift);
  ros::spin();

  return 0;
}
