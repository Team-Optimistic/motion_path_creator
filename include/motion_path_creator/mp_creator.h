#pragma once

#include <ros/ros.h>
#include <string>
#include <geometry_msgs/Point32.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud.h>

class mpCreator
{
public:
  mpCreator();

  /**
  * Callback for a new lidar scan from xv_11
  */
  void scanCallback(const sensor_msgs::PointCloud::ConstPtr& in);

  /**
   * Callback for ekf position estimate
   */
  void odomCallback(const nav_msgs::Odometry::ConstPtr& in);

  /**
   * Callback for robotPOS request for closest object behind robot
   */
  void mpCreator::robotPOSCallback(void); //const geometry_msgs::Point32::ConstPtr& in
private:
  ros::NodeHandle n;
  ros::Publisher mpcPub;
  ros::Subscriber scanSub, odomSub, robotPOSSub;

  // Current ekf estimate
  float x, y, theta;
  float xVel, yVel;

  // Conversion factor from angle to distance
  const float angleWeight = 0.25;

  inline const float distanceToPoint(const geometry_msgs::Point32& p) const;
  inline const float angleToPoint(const geometry_msgs::Point32& p) const;
  bool objSortComparator(const geometry_msgs::Point32& a, const geometry_msgs::Point32& b) const;
};
