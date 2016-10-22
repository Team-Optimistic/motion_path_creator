#pragma once

#include <ros/ros.h>
#include <string>
#include <geometry_msgs/Point32.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Empty.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_broadcaster.h>

class mpCreator
{
public:
  mpCreator();

  /**
  * Callback for a new lidar scan from xv_11
  */
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& in);

  /**
   * Callback for ekf position estimate
   */
  void odomCallback(const nav_msgs::Odometry::ConstPtr& in);

  /**
   * Callback for robotPOS request for closest object behind robot
   */
  void robotPOSCallback(const std_msgs::Empty::ConstPtr& in);
private:
  ros::NodeHandle n;
  ros::Publisher mpcPub;
  ros::Subscriber scanSub, odomSub, robotPOSSub;
  tf::TransformBroadcaster br;
  tf::Transform transform;

  // Current ekf estimate
  float x, y, theta;
  float xVel, yVel;

  // Internal copy of recent point cloud
  sensor_msgs::PointCloud cloud;
  laser_geometry::LaserProjection projector_;

  // Conversion factor from angle to distance
  const float angleWeight = 0.25;

  inline const float distanceToPoint(const geometry_msgs::Point32& p) const;
  inline const float distanceToPoint(const geometry_msgs::Point32& p, const geometry_msgs::Point32& from) const;

  inline const float angleToPoint(const geometry_msgs::Point32& p) const;
  inline const float angleToPoint(const geometry_msgs::Point32& p, const geometry_msgs::Point32& from) const;

  inline const float getCost(const geometry_msgs::Point32& p, int angleOffset  = 0) const;

  bool objSortComparator(const geometry_msgs::Point32& a, const geometry_msgs::Point32& b) const;
  bool invObjSortComparator(const geometry_msgs::Point32& a, const geometry_msgs::Point32& b) const;

  bool sortByAngle(const geometry_msgs::Point32& a, const geometry_msgs::Point32& b) const;
  bool sortByDistance(const geometry_msgs::Point32& a, const geometry_msgs::Point32& b) const;
};
