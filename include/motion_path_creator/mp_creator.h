#pragma once

#include <ros/ros.h>
#include <string>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Empty.h>
#ifdef MPC_USE_FAKE_TRANSFORM
  #include <tf/transform_broadcaster.h>
#endif

class mpCreator
{
public:
  mpCreator();

  /**
  * Callback for a new field scan
  */
  void objectCallback(const sensor_msgs::PointCloud2::ConstPtr& in);

  /**
   * Callback for ekf position estimate
   */
  void odomCallback(const nav_msgs::Odometry::ConstPtr& in);

  /**
   * Callback for robotPOS request for closest object behind robot
   */
  void robotPOSCallback(const std_msgs::Empty::ConstPtr& in);

  /**
   * Callback for moving to a point
   */
  void moveToPointCallback(const geometry_msgs::PoseStamped::ConstPtr& in);
private:
  ros::NodeHandle n;
  ros::Publisher mpcPub;
  ros::Subscriber objectSub, odomSub, robotPOSSub, moveToPointSub;

  #ifdef MPC_USE_FAKE_TRANSFORM
    tf::TransformBroadcaster br;
    tf::Transform transform;
  #endif

  // Current ekf estimate
  float x, y, theta;
  float xVel, yVel;

  // Internal copy of recent point cloud
  sensor_msgs::PointCloud cloud;

  // Conversion factor from angle to distance
  const float angleWeight = 0.25;

  //PI
  static constexpr float PI_F = 3.14159265358979f;

  inline const float distanceToPoint(const geometry_msgs::Point32& p) const;
  inline const float distanceToPoint(const geometry_msgs::Point32& p, const geometry_msgs::Point32& from) const;

  inline const float angleToPoint(const geometry_msgs::Point32& p) const;
  inline const float angleToPoint(const geometry_msgs::Point32& p, const geometry_msgs::Point32& from) const;

  inline const float getCost(const geometry_msgs::Point32& p, int angleOffset = 0) const;

  bool sortByCost(const geometry_msgs::Point32& a, const geometry_msgs::Point32& b) const;
  bool sortByCost_Behind(const geometry_msgs::Point32& a, const geometry_msgs::Point32& b) const;

  bool sortByAngle(const geometry_msgs::Point32& a, const geometry_msgs::Point32& b) const;
  bool sortByDistance(const geometry_msgs::Point32& a, const geometry_msgs::Point32& b) const;
};
