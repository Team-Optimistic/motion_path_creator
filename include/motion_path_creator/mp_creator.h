#pragma once

#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>
#include <string>
#include <geometry_msgs/Point32.h>
#include <nav_msgs/Odometry.h>
#define _USE_MATH_DEFINES
#include <math.h>

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
    //The next object we pick up should be the one which is both:
    // close to us, and
    // in our direction of movement (no sense in turning around to get the
    // "technically" closest object)
    //
    // Sort in using std::sort in <algorithm> and comparator function objSortComparator
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr& in)
  {
    x = in->pose.pose.position.x;
    y = in->pose.pose.position.y;
    const geometry_msgs::Quaternion quat = in->pose.pose.orientation;
    theta = atan2((2 * ((quat.x * quat.w) + (quat.y * quat.z))),
                  ((quat.x * quat.x) + (quat.y * quat.y) - (quat.z * quat.z) - (quat.w * quat.w)));

    xVel = in->twist.twist.linear.x;
    yVel = in->twist.twist.linear.y;
  }

  ros::NodeHandle n;
  ros::Subscriber objSub, odomSub;

  geometry_msgs::Point32 nextObject;
private:
  float x, y, theta;
  float xVel, yVel;

  const float angleWeight = 0.25;

  inline const float distanceToPoint(geometry_msgs::Point32 p) const
  {
    return sqrt(pow(p.x - x, 2) * pow(p.y - y, 2));
  }

  inline const float angleToPoint(geometry_msgs::Point32 p) const
  {
    return (atan2(p.y - y, p.x - x) * (180.0 / M_PI)) - theta;
  }

  bool objSortComparator(geometry_msgs::Point32 a, geometry_msgs::Point32 b)
  {
    const float aWeight = distanceToPoint(a) + (angleWeight * angleToPoint(a));
    const float bWeight = distanceToPoint(b) + (angleWeight * angleToPoint(b));
    return aWeight <= bWeight;
  }
};
