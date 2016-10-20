#define _USE_MATH_DEFINES
#include <math.h>
#include <algorithm>
#include <iterator>
#include <functional>
#include <vector>

#include "motion_path_creator/mp_creator.h"

mpCreator::mpCreator()
{
  mpcPub = n.advertise<geometry_msgs::Point32>("mpc/nextObject", 1000);
}

/**
* Callback for a new lidar scan from xv_11
*/
void mpCreator::scanCallback(const sensor_msgs::LaserScan::ConstPtr& in)
{
  //Convert interal copy of recent laser scan into point cloud
  projector_.projectLaser(*in, cloud);

  // The next object we pick up should be the one which is both:
  // close to us and in our direction of movement (no sense in turning around
  // to get the "technically" closest object because turning is expensive)
  std::vector<geometry_msgs::Point32> objects = cloud.points;
  std::sort(std::begin(objects), std::end(objects), std::bind(&mpCreator::objSortComparator, this, std::placeholders::_1, std::placeholders::_2));
  mpcPub.publish(objects[0]);
}

/**
 * Callback for ekf position estimate
 */
void mpCreator::odomCallback(const nav_msgs::Odometry::ConstPtr& in)
{
  x = in->pose.pose.position.x;
  y = in->pose.pose.position.y;
  const geometry_msgs::Quaternion quat = in->pose.pose.orientation;
  theta = atan2((2 * ((quat.x * quat.w) + (quat.y * quat.z))),
                ((quat.x * quat.x) + (quat.y * quat.y) - (quat.z * quat.z) - (quat.w * quat.w)));

  xVel = in->twist.twist.linear.x;
  yVel = in->twist.twist.linear.y;
}

/**
 * Callback for robotPOS request for closest object behind robot
 */
void mpCreator::robotPOSCallback(const std_msgs::Empty::ConstPtr& in)
{
  std::vector<geometry_msgs::Point32> objects = cloud.points;
  std::sort(std::begin(objects), std::end(objects), std::bind(&mpCreator::invObjSortComparator, this, std::placeholders::_1, std::placeholders::_2));
  mpcPub.publish(objects[0]);
}

/**
 * Computes the distance from the robot to a point
 * @param  p The other point
 * @return   The distance from the robot to the point
 */
inline const float mpCreator::distanceToPoint(const geometry_msgs::Point32& p) const
{
  return sqrt(pow(p.x - x, 2) * pow(p.y - y, 2));
}

/**
 * Computes the angle from the robot to a point
 * @param  p The other point
 * @return   The angle from the robot to the point
 */
inline const float mpCreator::angleToPoint(const geometry_msgs::Point32& p) const
{
  return (atan2(p.y - y, p.x - x) * (180.0 / M_PI)) - theta;
}

/**
 * std::sort comparator function, returns whichever point has less cost
 * @param  a Point a
 * @param  b Point b
 * @return   Point with less cost
 */
bool mpCreator::objSortComparator(const geometry_msgs::Point32& a, const geometry_msgs::Point32& b) const
{
  const float aWeight = distanceToPoint(a) + (angleWeight * angleToPoint(a));
  const float bWeight = distanceToPoint(b) + (angleWeight * angleToPoint(b));
  return aWeight <= bWeight;
}

/**
 * std::sort comparator function, returns whichever point has less cost
 * @param  a Point a
 * @param  b Point b
 * @return   Point with less cost
 */
bool mpCreator::invObjSortComparator(const geometry_msgs::Point32& a, const geometry_msgs::Point32& b) const
{
  const float aWeight = distanceToPoint(a) + (angleWeight * (180 - angleToPoint(a)));
  const float bWeight = distanceToPoint(b) + (angleWeight * (180 - angleToPoint(b)));
  return aWeight <= bWeight;
}
