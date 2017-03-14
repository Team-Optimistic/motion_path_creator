#include <cmath>
#include <algorithm>
#include <iterator>
#include <functional>
#include <vector>
#include <sensor_msgs/point_cloud_conversion.h>
#include <limits>

#include "motion_path_creator/mp_creator.h"

mpCreator::mpCreator()
{
  mpcPub = n.advertise<sensor_msgs::PointCloud2>("mpc/nextObjects", 10);
  smallObjsSub = n.subscribe<sensor_msgs::PointCloud2>("goat/small_objects", 10, &mpCreator::smallObjsCallback, this);
  bigObjsSub = n.subscribe<sensor_msgs::PointCloud2>("goat/big_objects", 10, &mpCreator::bigObjsCallback, this);
  odomSub = n.subscribe<nav_msgs::Odometry>("odometry/filtered", 10, &mpCreator::odomCallback, this);
  moveToPointSub = n.subscribe<geometry_msgs::PoseStamped>("move_base_simple/goal", 10, &mpCreator::moveToPointCallback, this);
  customMoveSub = n.subscribe<geometry_msgs::PoseStamped>("customMove", 10, &mpCreator::customMoveCallback, this);

  coords.x = coords.y = coords.z = 0;
}

/**
* Callback for a new list of small objects on the field
*/
void mpCreator::smallObjsCallback(const sensor_msgs::PointCloud2::ConstPtr& in)
{
  //Convert interal copy of recent laser scan into point cloud
  sensor_msgs::convertPointCloud2ToPointCloud(*in, smallObjects);
}

/**
* Callback for a new list of big objects on the field
*/
void mpCreator::bigObjsCallback(const sensor_msgs::PointCloud2::ConstPtr& in)
{
  //Convert interal copy of recent laser scan into point cloud
  sensor_msgs::convertPointCloud2ToPointCloud(*in, bigObjects);
}

/**
 * Callback for ekf position estimate
 */
void mpCreator::odomCallback(const nav_msgs::Odometry::ConstPtr& in)
{
  coords.x = in->pose.pose.position.x;
  coords.y = in->pose.pose.position.y;
  const geometry_msgs::Quaternion quat = in->pose.pose.orientation;
  coords.z = atan2((2 * ((quat.x * quat.w) + (quat.y * quat.z))),
                  ((quat.x * quat.x) + (quat.y * quat.y) - (quat.z * quat.z) - (quat.w * quat.w)));
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

  ROS_INFO("moving to nav goal (%1.2f,%1.2f), type: %d", outPoint.x, outPoint.y, (int)outPoint.z);

  mpcPub.publish(outFinal);
}

void mpCreator::customMoveCallback(const geometry_msgs::PoseStamped::ConstPtr& in)
{
  sensor_msgs::PointCloud out;

  std::vector<geometry_msgs::Point32> outVector(1);
  geometry_msgs::Point32 outPoint;
  outPoint.x = in->pose.position.x;
  outPoint.y = in->pose.position.y;
  outPoint.z = in->pose.position.z;
  outVector[0] = outPoint;
  out.points = outVector;

  sensor_msgs::PointCloud2 outFinal;
  sensor_msgs::convertPointCloudToPointCloud2(out, outFinal);

  ROS_INFO("moving to custom nav goal (%1.2f,%1.2f), type: %d", outPoint.x, outPoint.y, (int)outPoint.z);

  mpcPub.publish(outFinal);
}
