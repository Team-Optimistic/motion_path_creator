#pragma once

#include <sensor_msgs/PointCloud.h>
#include <ros/ros.h>
#include <string>
#include <geometry_msgs/Point32.h>
#include <nav_msgs/Odometry.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <algorithm>
#include <iterator>
#include <functional>
#include <vector>

class mpCreator
{
public:
  mpCreator()
  {
    objSub = n.subscribe<sensor_msgs::PointCloud>("cloud", 10, &mpCreator::objCallback, this);
    odomSub = n.subscribe<nav_msgs::Odometry>("odometry/filtered", 1000, &mpCreator::odomCallback, this);
  }

  void objCallback(const sensor_msgs::PointCloud::ConstPtr& in)
  {
    // The next object we pick up should be the one which is both:
    // close to us and in our direction of movement (no sense in turning around
    // to get the "technically" closest object because turning is expensive)
    std::vector<geometry_msgs::Point32> objects = in->points;
    std::sort(std::begin(objects), std::end(objects), std::bind(&mpCreator::objSortComparator, this, std::placeholders::_1, std::placeholders::_2));
    currentObject = objects[0];
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

  inline const geometry_msgs::Point32& getNextObject() const { return currentObject; }

  ros::NodeHandle n;
  ros::Subscriber objSub, odomSub;

  geometry_msgs::Point32 nextObject;
private:
  // Current ekf estimate
  float x, y, theta;
  float xVel, yVel;

  // Current object (or next object) to pursue
  geometry_msgs::Point32 currentObject;

  // Conversion factor from angle to distance
  const float angleWeight = 0.25;

  inline const float distanceToPoint(const geometry_msgs::Point32& p) const
  {
    return sqrt(pow(p.x - x, 2) * pow(p.y - y, 2));
  }

  inline const float angleToPoint(const geometry_msgs::Point32& p) const
  {
    return (atan2(p.y - y, p.x - x) * (180.0 / M_PI)) - theta;
  }

  bool objSortComparator(const geometry_msgs::Point32& a, const geometry_msgs::Point32& b)
  {
    const float aWeight = distanceToPoint(a) + (angleWeight * angleToPoint(a));
    const float bWeight = distanceToPoint(b) + (angleWeight * angleToPoint(b));
    return aWeight <= bWeight;
  }
};
