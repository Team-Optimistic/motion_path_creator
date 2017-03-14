#pragma once

#include <ros/ros.h>
#include <string>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Empty.h>

class mpCreator
{
public:
  mpCreator();

  /**
  * Callback for a new list of stars on the field
  */
  void smallObjsCallback(const sensor_msgs::PointCloud2::ConstPtr& in);

  /**
  * Callback for a new list of cubes on the field
  */
  void bigObjsCallback(const sensor_msgs::PointCloud2::ConstPtr& in);

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

  /**
   * Callback for moving to a point in a custom way
   */
  void customMoveCallback(const geometry_msgs::PoseStamped::ConstPtr& in);

  inline const sensor_msgs::PointCloud& getSmallObjs() const { return smallObjects; }
  inline const sensor_msgs::PointCloud& getBigObjs() const { return bigObjects; }
  inline const geometry_msgs::Point32& getCoords() const { return coords; }
private:
  ros::NodeHandle n;
  ros::Publisher mpcPub;
  ros::Subscriber smallObjsSub, bigObjsSub, odomSub, robotPOSSub, moveToPointSub, customMoveSub;

  //Internal copy of recent objects
  sensor_msgs::PointCloud smallObjects, bigObjects;

  //Current ekf estimate, z is theta
  geometry_msgs::Point32 coords;
};
