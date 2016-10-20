#define _USE_MATH_DEFINES
#include <math.h>
#include <algorithm>
#include <iterator>
#include <functional>
#include <vector>

#include "mp_creator.h"

mpCreator::mpCreator()
{
  mpcPub = n.advertise<geometry_msgs::Point32>("mpc/nextObject");
  scanSub = n.subscribe<sensor_msgs::LaserScan>("scan", 1000, &mpCreator::objCallback, this);
  odomSub = n.subscribe<nav_msgs::Odometry>("odometry/filtered", 1000, &mpCreator::odomCallback, this);
  robotPOSSub = n.subscribe<void>("robotPOS/pickedUpObject", 1000, &mpCreator::robotPOSCallback, this);
}

/**
* Callback for a new lidar scan from xv_11
*/
void mpCreator::scanCallback(const sensor_msgs::PointCloud::ConstPtr& in)
{
  // The next object we pick up should be the one which is both:
  // close to us and in our direction of movement (no sense in turning around
  // to get the "technically" closest object because turning is expensive)
  std::vector<geometry_msgs::Point32> objects = in->points;
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
void mpCreator::robotPOSCallback()
{

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