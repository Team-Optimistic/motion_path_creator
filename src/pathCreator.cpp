#include <ros/ros.h>
#include <string>
#define _USE_MATH_DEFINES
#include <math.h>
#include <vector>
#include <limits>
#include <sensor_msgs/point_cloud_conversion.h>
#include <algorithm>
#include <iterator>

#include "motion_path_creator/mp_creator.h"

inline const float distanceToPoint(const geometry_msgs::Point32& from, const geometry_msgs::Point32& to);
inline const float angleToPoint(const geometry_msgs::Point32& from, const geometry_msgs::Point32& to);
inline const float getTypeCost(const geometry_msgs::Point32& obj);
inline const float getCost(const geometry_msgs::Point32& robot, const geometry_msgs::Point32& object, const float objectCost);
bool sortByCost(const geometry_msgs::Point32& robot, const geometry_msgs::Point32& obj1, const float obj1Cost, const geometry_msgs::Point32& obj2, const float obj2Cost);
bool sortByDistance(const geometry_msgs::Point32& robot, const geometry_msgs::Point32& obj1, const float obj1Cost, const geometry_msgs::Point32& obj2, const float obj2Cost);
bool sortByAngle(const geometry_msgs::Point32& robot, const geometry_msgs::Point32& obj1, const float obj1Cost, const geometry_msgs::Point32& obj2, const float obj2Cost);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "motion_path_creator");
  static mpCreator mpc;

  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<sensor_msgs::PointCloud2>("mpc/nextObjects", 1000);

  while (ros::ok())
  {
    ros::spinOnce(); //Callbacks

    std::vector<geometry_msgs::Point32> objList;

    //Add small objects
    for (auto&& obj : mpc.getSmallObjs().points)
      objList.push_back(obj);

    //Add big objects
    for (auto&& obj : mpc.getBigObjs().points)
      objList.push_back(obj);

    //Sort by cost
    std::sort(objList.begin(), objList.end(), [](geometry_msgs::Point32 a, geometry_msgs::Point32 b) {
      return sortByCost(mpc.getCoords(), a, getTypeCost(a), b, getTypeCost(b));
    });

    //Convert objList to PointCloud2
    sensor_msgs::PointCloud2 out;
    sensor_msgs::PointCloud temp;
    std::copy(objList.begin(), objList.end(), std::begin(temp.points));
    sensor_msgs::convertPointCloudToPointCloud2(temp, out);
    pub.publish(out);
  }

  return 0;
}

/**
 * Computes the distance from the a point to a point
 * @param  from The point to measure from (point 1)
 * @param  to   The point to measure to (point 2)
 * @return      The distance between the points
 */
inline const float distanceToPoint(const geometry_msgs::Point32& from, const geometry_msgs::Point32& to)
{
  return sqrt(pow(to.x - from.x, 2) * pow(to.y - from.y, 2));
}

/**
 * Computes the angle between two points
 * @param  from The point to measure from (point 1)
 * @param  to   The point to measure to (point 2)
 * @return      The angle between the points
 */
inline const float angleToPoint(const geometry_msgs::Point32& from, const geometry_msgs::Point32& to)
{
  return atan2(to.y - from.y, to.x - from.x) * (180.0 / M_PI);
}

/**
 * Returns the cost for the type of an object
 * @param  obj Object to use
 * @return     Type cost of object
 */
inline const float getTypeCost(const geometry_msgs::Point32& obj)
{
  constexpr float smallObjectCost = 1/1.5, bigObjectCost = 1/4;

  switch (int(obj.z))
  {
    case 1:
      return smallObjectCost;

    case 2:
      return bigObjectCost;

    default:
      return std::numeric_limits<int>::max();
  }
}

/**
 * Computes the cost to pick up an object
 * @param  robot      Where the robot is
 * @param  object     Where the object is
 * @param  objectCost The cost of the object type
 * @return            The cost of the object
 */
inline const float getCost(const geometry_msgs::Point32& robot, const geometry_msgs::Point32& object, const float objectCost)
{
  constexpr float moveCost = 1, turnCost = 2;
  return (distanceToPoint(robot, object) * moveCost + (angleToPoint(robot, object) - robot.z) * turnCost) * objectCost;
}

/**
 * Computes which object has lesser cost
 * @param  robot    Where the robot is
 * @param  obj1     First object
 * @param  obj1Cost Cost of first object's type
 * @param  obj2     Second object
 * @param  obj2Cost Cost of second object's type
 * @return          Whether the first object has lower cost than the second
 */
bool sortByCost(const geometry_msgs::Point32& robot, const geometry_msgs::Point32& obj1, const float obj1Cost, const geometry_msgs::Point32& obj2, const float obj2Cost)
{
  return getCost(robot, obj1, obj1Cost) <= getCost(robot, obj2, obj2Cost);
}

/**
 * Computes which object has lesser distance
 * @param  robot    Where the robot is
 * @param  obj1     First object
 * @param  obj1Cost Cost of first object's type
 * @param  obj2     Second object
 * @param  obj2Cost Cost of second object's type
 * @return          Whether the first object is closer than the second
 */
bool sortByDistance(const geometry_msgs::Point32& robot, const geometry_msgs::Point32& obj1, const float obj1Cost, const geometry_msgs::Point32& obj2, const float obj2Cost)
{
  return distanceToPoint(robot, obj1) <= distanceToPoint(robot, obj2);
}

/**
 * Computes which object has lesser angle
 * @param  robot    Where the robot is
 * @param  obj1     First object
 * @param  obj1Cost Cost of first object's type
 * @param  obj2     Second object
 * @param  obj2Cost Cost of second object's type
 * @return          Whether the first object is closer than the second
 */
bool sortByAngle(const geometry_msgs::Point32& robot, const geometry_msgs::Point32& obj1, const float obj1Cost, const geometry_msgs::Point32& obj2, const float obj2Cost)
{
  return angleToPoint(robot, obj1) <= angleToPoint(robot, obj2);
}
