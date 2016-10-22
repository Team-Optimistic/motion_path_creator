#define _USE_MATH_DEFINES
#include <math.h>
#include <algorithm>
#include <iterator>
#include <functional>
#include <vector>

#include "motion_path_creator/mp_creator.h"

mpCreator::mpCreator():
  x(0),
  y(0),
  theta(0),
  xVel(0),
  yVel(0)
{
  mpcPub = n.advertise<geometry_msgs::Point32>("mpc/nextObject", 1000);
  temp = n.advertise<sensor_msgs::PointCloud>("temp/temp", 1000);
  scanSub = n.subscribe<sensor_msgs::LaserScan>("xv/scan", 1000, &mpCreator::scanCallback, this);
  odomSub = n.subscribe<nav_msgs::Odometry>("odometry/filtered", 1000, &mpCreator::odomCallback, this);
  robotPOSSub = n.subscribe<std_msgs::Empty>("robotPOS/spcRequest", 1000, &mpCreator::robotPOSCallback, this);

  geometry_msgs::Point32 a;
  a.x = 1;
  a.y = 6;

  geometry_msgs::Point32 b;
  b.x = 4;
  b.y = 7;

  geometry_msgs::Point32 c;
  c.x = 5;
  c.y = 6;

  geometry_msgs::Point32 d;
  d.x = 7;
  d.y = 3;

  geometry_msgs::Point32 e;
  e.x = 8;
  e.y = 4;

  std::vector<geometry_msgs::Point32> points;
  points.push_back(a);
  points.push_back(b);
  points.push_back(c);
  points.push_back(e);
  points.push_back(d);

  cloud.points = points;
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
 * Computes the distance from the a point to a point
 * @param  p The other point
 * @return   The distance from the robot to the point
 */
inline const float mpCreator::distanceToPoint(const geometry_msgs::Point32& p) const
{
  geometry_msgs::Point32 robot;
  robot.x = x;
  robot.y = y;
  return distanceToPoint(p, robot);
}

/**
 * Computes the distance from the a point to a point
 * @param  p    The other point
 * @param  from The point to measure from
 * @return      The distance from the robot to the point
 */
inline const float mpCreator::distanceToPoint(const geometry_msgs::Point32& p, const geometry_msgs::Point32& from) const
{
  return sqrt(pow(p.x - from.x, 2) * pow(p.y - from.y, 2));
}

/**
 * Computes the angle from the a point to a point
 * @param  p The other point
 * @return   The angle from the robot to the point
 */
inline const float mpCreator::angleToPoint(const geometry_msgs::Point32& p) const
{
  geometry_msgs::Point32 robot;
  robot.x = x;
  robot.y = y;
  return angleToPoint(p, robot);
}

/**
 * Computes the angle from the a point to a point
 * @param  p    The other point
 * @param  from The point to measure from
 * @return      The angle from the robot to the point
 */
inline const float mpCreator::angleToPoint(const geometry_msgs::Point32& p, const geometry_msgs::Point32& from) const
{
  return (atan2(p.y - from.y, p.x - from.x) * (180.0 / M_PI)) - theta;
}

/**
 * Computes the cost of an object
 * @param  p           The object
 * @param  angleOffset Artifically rotate the robot to change the cost
 * @return             The cost of the object
 */
inline const float mpCreator::getCost(const geometry_msgs::Point32& p, int angleOffset) const
{
  return distanceToPoint(p) + (angleWeight * (angleOffset - angleToPoint(p)));
}

/**
 * std::sort comparator function, returns whichever point has less cost
 * @param  a Point a
 * @param  b Point b
 * @return   Point with less cost
 */
bool mpCreator::objSortComparator(const geometry_msgs::Point32& a, const geometry_msgs::Point32& b) const
{
  return getCost(a) <= getCost(b);
}

/**
 * std::sort comparator function, returns whichever point has less cost
 * @param  a Point a
 * @param  b Point b
 * @return   Point with less cost
 */
bool mpCreator::invObjSortComparator(const geometry_msgs::Point32& a, const geometry_msgs::Point32& b) const
{
  return getCost(a, 180) <= getCost(b, 180);
}

bool mpCreator::sortByAngle(const geometry_msgs::Point32& a, const geometry_msgs::Point32& b) const
{
  return angleToPoint(a) <= angleToPoint(b);
}

bool mpCreator::sortByDistance(const geometry_msgs::Point32& a, const geometry_msgs::Point32& b) const
{
  return distanceToPoint(a) <= distanceToPoint(b);
}

void mpCreator::johnAlgorithm()
{
  // Find the items from a corner alongside the wall in polar coordinates. Then
  // sort the items by the theta angle then by radius.  All the items along the
  // wall are gotten immediately from one end to the other and then the robot
  // will work its way around the edge.  You could add in a filter to at first
  // skip stars that don't have a neighbor with a certain radius, and then
  // include them later. That should give a fast path that should decrease the
  // driving needed.

  std::vector<geometry_msgs::Point32> objects = cloud.points;

  geometry_msgs::Point32 corner;
  corner.x = 0;
  corner.y = 0;

  //Split objects into sections by theta
  std::vector<std::vector<geometry_msgs::Point32>> zones(36);

  for (auto&& p : objects)
  {
    zones[(int)(round(angleToPoint(p, corner) / 5.0))].push_back(p);
  }

  for (auto&& v : zones)
  {
    std::cout << "V:" << std::endl;
    for (auto&& p : v)
    {
      std::cout << "P:" << p.x << "," << p.y << ";" << std::endl;
     }
  }

  //Sort each list by radius
  for (auto&& v : zones)
  {
    std::sort(std::begin(v), std::end(v), std::bind(&mpCreator::sortByDistance, this, corner, std::placeholders::_2));
  }

  for (auto&& v : zones)
  {
    std::cout << "V:" << std::endl;
    for (auto&& p : v)
    {
      std::cout << "P:" << p.x << "," << p.y << ";" << std::endl;
     }
  }

  static int seq = 0;
  sensor_msgs::PointCloud objCloud;
  objCloud.points = objects;
  objCloud.header.seq = seq++;
  objCloud.header.stamp = ros::Time::now();
  objCloud.header.frame_id = "/world";

  transform.setRotation(tf::Quaternion(0,0,0,1));
  transform.setOrigin(tf::Vector3(0,0,0));
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/world", "myFrame"));

  temp.publish(objCloud);
}
