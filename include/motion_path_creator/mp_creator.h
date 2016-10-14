#pragma once

#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>
#include <string>
#include <geometry_msgs/Point32.h>
#include <nav_msgs/Odometry.h>

class mpCreator
{
public:
  mpCreator()
  {
    objSub = n.subscribe<geometry_msgs::Point32>("cloud", 10, &mpCreator::objCallback, this);
    odomSub = n.subscribe<nav_msgs::Odometry>("odometry/filtered", 1000, &mpCreator::odomCallback, this);
  }

  void objCallback(const geometry_msgs::Point32::ConstPtr& in)
  {
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr& in)
  {
    x = in->pose.pose.position.x;
    y = in->pose.pose.position.y;
    const geometry_msgs::Quaternion quat = in->pose.pose.orientation;
    theta = atan2((2 * ((quat.x * quat.w) + (quat.y * quat.z))),
                  ((quat.x * quat.x) + (quat.y * quat.y) - (quat.z * quat.z) - (quat.w * quat.w)));
  }

  ros::NodeHandle n;
  ros::Subscriber objSub, odomSub;

  geometry_msgs::Point32 nextObject;
private:
  float x, y, theta;
};
