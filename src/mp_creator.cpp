#include <cmath>
#include <algorithm>
#include <iterator>
#include <functional>
#include <vector>
#include <sensor_msgs/point_cloud_conversion.h>
#include <limits>

#include "motion_path_creator/mp_creator.h"

mpCreator::mpCreator():
  x(0),
  y(0),
  theta(0),
  xVel(0),
  yVel(0)
{
  mpcPub = n.advertise<sensor_msgs::PointCloud2>("nextObjects", 1000);
  objectSub = n.subscribe<sensor_msgs::PointCloud2>("objectList", 1000, &mpCreator::objectCallback, this);
  odomSub = n.subscribe<nav_msgs::Odometry>("odometry/filtered", 1000, &mpCreator::odomCallback, this);
  robotPOSSub = n.subscribe<std_msgs::Empty>("spcRequest", 1000, &mpCreator::robotPOSCallback, this);
  moveToPointSub = n.subscribe<geometry_msgs::PoseStamped>("move_base_simple/goal", 1000, &mpCreator::moveToPointCallback, this);

  #ifdef MPC_USE_FAKE_TRANSFORM
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
  #endif
}

/**
* Callback for a new field scan
*/
void mpCreator::objectCallback(const sensor_msgs::PointCloud2::ConstPtr& in)
{
  //Convert interal copy of recent laser scan into point cloud
  sensor_msgs::convertPointCloud2ToPointCloud(*in, cloud);

  //RYAN ALGORITHM
  const int listCount = 5; //Number of lists: one for the center and each wall
  std::vector<geometry_msgs::Point32> lists[listCount];

  //Initialize lists
  for (int i = 0; i < listCount; i++)
  {
    lists[i] = std::vector<geometry_msgs::Point32>();
  }

  //Index of each list in lists
  const int objects = 0, rightWallObjects = 1, leftWallObjects = 2, southWallObjects = 3, fenceObjects = 4;

  //Only consider objects that aren't too close to the field wall, unless there
  //are enough of them to only go after them
  const int intakeLength = 18;
  int rightCounter = 0, leftCounter = 0, southCounter = 0, fenceCounter = 0;
  bool shouldSort = true; //Don't sort if we're using objects on the wall

  for (auto&& p : cloud.points)
  {
    //Check if object is too close to the right wall
    if (p.x >= 144 - intakeLength)
    {
      lists[rightWallObjects].push_back(p);
      rightCounter++;
    }
    //Check if boject is too close to the left wall
    else if (p.x <= intakeLength)
    {
      lists[leftWallObjects].push_back(p);
      leftCounter++;
    }
    //Check if the object is too close to the south wall
    else if (p.y <= intakeLength)
    {
      lists[southWallObjects].push_back(p);
      southCounter++;
    }
    //Check if the object is too close to the fence
    else if (p.y >= 72 - intakeLength)
    {
      lists[fenceObjects].push_back(p);
      fenceCounter++;
    }
    else
    {
      lists[objects].push_back(p);
    }
  }

  //Sort objects by relative cost
  //Start lowest cost as a big number that won't happen normally (0xCCCCCCC)
  constexpr int bigNumber = 214748364;
  int lowestCost = bigNumber, lowestCostIndex = 0;
  int lowestCostBackup = bigNumber, lowestCostIndexBackup = 0;
  int temp = 0;

  //Iterate over each list
  for (int i = 0; i < listCount; i++)
  {
    //Sort list by cost
    std::sort(std::begin(lists[i]), std::end(lists[i]), std::bind(&mpCreator::sortByCost, this, std::placeholders::_1, std::placeholders::_2));

    //Get cost of list
    temp = 0;
    for (auto&& p : lists[i])
    {
      temp += getCost(p);
    }

    //Save list if it has lowest cost yet
    //Save list into backups if it doesn't have enough objects
    if (lists[i].size() < 3 && temp < lowestCostBackup)
    {
      lowestCostBackup = temp;
      lowestCostIndexBackup = i;
    }
    //If size is fine, save into priority
    else if (temp < lowestCost)
    {
      lowestCost = temp;
      lowestCostIndex = i;
    }
  }

  //Publish cheapest list
  sensor_msgs::PointCloud cloudSorted;

  //If there are no priority lists, use backup list
  if (lowestCost == bigNumber)
  {
    cloudSorted.points = lists[lowestCostIndexBackup];
  }
  else if (lowestCostBackup == bigNumber)
  {
    cloudSorted.points = lists[lowestCostIndex];
  }
  //If there are no lists at all, publish nothing
  else
  {
    cloudSorted.points = std::vector<geometry_msgs::Point32>();
    ROS_INFO("No objects to get");
  }

  //Convert into PointCloud2
  sensor_msgs::PointCloud2 out;
  sensor_msgs::convertPointCloudToPointCloud2(cloudSorted, out);
  mpcPub.publish(out);

  //---------------------------------------------------------------------------

  //JOHN ALGORITHM

  // // Find the items from a corner alongside the wall in polar coordinates. Then
  // // sort the items by the theta angle then by radius.  All the items along the
  // // wall are gotten immediately from one end to the other and then the robot
  // // will work its way around the edge.  You could add in a filter to at first
  // // skip stars that don't have a neighbor with a certain radius, and then
  // // include them later. That should give a fast path that should decrease the
  // // driving needed.
  //
  // std::vector<geometry_msgs::Point32> objects = cloud.points;
  //
  // geometry_msgs::Point32 corner;
  // corner.x = 0;
  // corner.y = 0;
  //
  // //Split objects into sections by theta
  // std::vector<std::vector<geometry_msgs::Point32>> zones(36);
  //
  // for (auto&& p : objects)
  // {
  //   zones[(int)(std::round(angleToPoint(p, corner) / 5.0))].push_back(p);
  // }
  //
  // // for (auto&& v : zones)
  // // {
  // //   std::cout << "V:" << std::endl;
  // //   for (auto&& p : v)
  // //   {
  // //     std::cout << "P:" << p.x << "," << p.y << ";" << std::endl;
  // //    }
  // // }
  //
  // //Sort each list by radius
  // for (auto&& v : zones)
  // {
  //   std::sort(std::begin(v), std::end(v), std::bind(&mpCreator::sortByDistance, this, corner, std::placeholders::_2));
  // }
  //
  // // for (auto&& v : zones)
  // // {
  // //   std::cout << "V:" << std::endl;
  // //   for (auto&& p : v)
  // //   {
  // //     std::cout << "P:" << p.x << "," << p.y << ";" << std::endl;
  // //    }
  // // }
  //
  // // Add a start mode that has a path for the first few that works its way there
  // // and then switch modes. Also build in a chaining functionality so stars
  // // within x inches are automatically considered priority targets. That way the
  // // robot will get into a cluster or line and work along it.
  // // Build it in a layered fsm:
  //   // Driving/Sensing
  //   // Get Adjacent Stars
  //   // Move to Adjacent Star
  //   // Move to Next Best Star (sorted on field)
  //
  // sensor_msgs::PointCloud2 out;
  // sensor_msgs::convertPointCloudToPointCloud2(objCloud, out);
  // mpcPub.publish(out);

  // Fake transform for rviz
  #ifdef MPC_USE_FAKE_TRANSFORM
    static int seq = 0;
    sensor_msgs::PointCloud objCloud;
    objCloud.points = objects;
    objCloud.header.seq = seq++;
    objCloud.header.stamp = ros::Time::now();
    objCloud.header.frame_id = "/world";

    transform.setRotation(tf::Quaternion(0,0,0,1));
    transform.setOrigin(tf::Vector3(0,0,0));
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/world", "myFrame"));
  #endif
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
  std::sort(std::begin(objects), std::end(objects), std::bind(&mpCreator::sortByCost_Behind, this, std::placeholders::_1, std::placeholders::_2));
  mpcPub.publish(objects[0]);
}

void mpCreator::moveToPointCallback(const geometry_msgs::PoseStamped::ConstPtr& in)
{
  sensor_msgs::PointCloud out;

  std::vector<geometry_msgs::Point32> outVector(1);
  geometry_msgs::Point32 outPoint;
  outPoint.x = in->pose.position.x;
  outPoint.y = in->pose.position.y;
  outPoint.z = 0;
  outVector[0] = outPoint;
  out.points = outVector;

  sensor_msgs::PointCloud2 outFinal;
  sensor_msgs::convertPointCloudToPointCloud2(out, outFinal);

  ROS_INFO("moving to nav goal (%1.2f,%1.2f)", outPoint.x, outPoint.y);

  mpcPub.publish(outFinal);
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
  return (atan2(p.y - from.y, p.x - from.x) * (180.0 / PI_F)) - theta;
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
bool mpCreator::sortByCost(const geometry_msgs::Point32& a, const geometry_msgs::Point32& b) const
{
  return getCost(a) <= getCost(b);
}

/**
 * std::sort comparator function, returns whichever point has less cost
 * @param  a Point a
 * @param  b Point b
 * @return   Point with less cost
 */
bool mpCreator::sortByCost_Behind(const geometry_msgs::Point32& a, const geometry_msgs::Point32& b) const
{
  return getCost(a, 180) <= getCost(b, 180);
}

/**
 * Computes which points has lesser angle
 * @param  a Point A
 * @param  b Point B
 * @return   The point with a lesser angle
 */
bool mpCreator::sortByAngle(const geometry_msgs::Point32& a, const geometry_msgs::Point32& b) const
{
  return angleToPoint(a) <= angleToPoint(b);
}

/**
 * Compute which point has a lesser distance
 * @param  a Point A
 * @param  b Point B
 * @return   The point with a lesser distance
 */
bool mpCreator::sortByDistance(const geometry_msgs::Point32& a, const geometry_msgs::Point32& b) const
{
  return distanceToPoint(a) <= distanceToPoint(b);
}
