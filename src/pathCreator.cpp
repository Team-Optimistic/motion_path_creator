#include <ros/ros.h>
#include <string>
#define _USE_MATH_DEFINES
#include <math.h>
#include <vector>
#include <limits>
#include <sensor_msgs/point_cloud_conversion.h>
#include <algorithm>
#include <iterator>
#include <tf/tf.h>
#include <nav_msgs/Path.h>

#include "motion_path_creator/mp_creator.h"

//Prototypes
inline const float distanceToPoint(const geometry_msgs::Point32& from, const geometry_msgs::Point32& to);
inline const float angleToPoint(const geometry_msgs::Point32& from, const geometry_msgs::Point32& to);
inline const float getTypeCost(const geometry_msgs::Point32& obj);
inline const float getCost(const geometry_msgs::Point32& robot, const geometry_msgs::Point32& object, const float objectCost);
bool sortByCost(const geometry_msgs::Point32& robot, const geometry_msgs::Point32& obj1, const float obj1Cost, const geometry_msgs::Point32& obj2, const float obj2Cost);
bool sortByDistance(const geometry_msgs::Point32& robot, const geometry_msgs::Point32& obj1, const float obj1Cost, const geometry_msgs::Point32& obj2, const float obj2Cost);
bool sortByAngle(const geometry_msgs::Point32& robot, const geometry_msgs::Point32& obj1, const float obj1Cost, const geometry_msgs::Point32& obj2, const float obj2Cost);
void publishObjects(const int numObjs, const std::vector<geometry_msgs::Point32> objs, const ros::Publisher pub);

struct ObjTypes
{
  enum types
  {
    small = 1,
    big = 2
  };
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "motion_path_creator");
  static mpCreator mpc;

  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<sensor_msgs::PointCloud>("mpc/nextObjects", 10),
  pathPub = n.advertise<nav_msgs::Path>("mpc/path", 1);
  ros::Rate rate(100.0);//loop at 100HZ

  while (ros::ok())
  {
    rate.sleep();//ensure even the new msg check maxes at 100 hz
    ros::spinOnce(); //Callbacks
    
    if(!mpc.isNewMessage())
      continue;

    std::vector<geometry_msgs::Point32> objList, finalObjList;
    geometry_msgs::Point32 coords = mpc.getCoords();

    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "/field";
    pose.pose.position.x = coords.x;
    pose.pose.position.y = coords.y;
    pose.pose.position.z = 0;
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(coords.z);

    //Generate path
    nav_msgs::Path path;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = "/field";
    path.poses.push_back(pose); // starting position

    //Add small objects
    for (auto&& obj : mpc.getSmallObjs().points)
      objList.push_back(obj);

    //Add big objects
    for (auto&& obj : mpc.getBigObjs().points)
      objList.push_back(obj);

    //Loop until we have enough objects
    int objCount = 0;
    while (objCount < 3 && objList.size() > 0)
    {
      //calculate costs with position
      std::sort(objList.begin(), objList.end(), [coords](geometry_msgs::Point32 a, geometry_msgs::Point32 b) {
        return sortByCost(coords, a, getTypeCost(a), b, getTypeCost(b));
      });

      //Add cheapest element to final list
      finalObjList.push_back(objList.front());
      objCount++; //We just added a new object so increment

      pose.header.stamp = ros::Time::now();
      pose.pose.position.x = objList.front().x;
      pose.pose.position.y = objList.front().y;
      pose.pose.orientation = tf::createQuaternionMsgFromYaw(angleToPoint(coords, objList.front()));
      pose.header.seq = objList.front().z;
      path.poses.push_back(pose); //goal location

      //If we have a big object
      if (objList.front().z == ObjTypes::big)
        break;

      objList.front().z = angleToPoint(coords,objList.front()); //puts point in same format as coords
      coords = objList.front(); //Move robot to that object's position
      objList.erase(objList.begin()); //Remove object from list so we don't consider it again
    }

    if(finalObjList.size() > 0)
      publishObjects(objCount, finalObjList, pub);

    //Publish path
    pathPub.publish(path);
  }
  return 0;
}

/**
* Publishes a number of objects
* @param numObjs Number of objects to publish
* @param objs    Object vector to read from
* @param pub     Publisher to publish with
*/
void publishObjects(const int numObjs, const std::vector<geometry_msgs::Point32> objs, const ros::Publisher pub)
{
  //Convert objList to PointCloud
  sensor_msgs::PointCloud temp;
  temp.header.stamp = ros::Time::now();
  temp.header.frame_id = "/field";
  temp.points = objs;
  pub.publish(temp);
  ROS_INFO("mpc: sent %d objects", temp.points.size());
}

/**
* Computes the distance from the a point to a point
* @param  from The point to measure from (point 1)
* @param  to   The point to measure to (point 2)
* @return      The distance between the points
*/
inline const float distanceToPoint(const geometry_msgs::Point32& from, const geometry_msgs::Point32& to)
{
  return sqrt(pow(to.x - from.x, 2) + pow(to.y - from.y, 2));
}

/**
* Computes the angle between two points
* @param  from The point to measure from (point 1)
* @param  to   The point to measure to (point 2)
* @return      The angle between the points
*/
inline const float angleToPoint(const geometry_msgs::Point32& from, const geometry_msgs::Point32& to)
{
  return atan2(to.y - from.y, to.x - from.x);
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
    case ObjTypes::small:
    return smallObjectCost;

    case ObjTypes::big:
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
  constexpr float moveCost = 1, turnCost = 1/60;

  float turnDistance = (angleToPoint(robot, object) - robot.z) * (180.0 / M_PI);

  turnDistance = turnDistance > 180 ? turnDistance - 360 : turnDistance;
  turnDistance = turnDistance < -180 ? turnDistance + 360 : turnDistance;
  turnDistance = turnDistance < 0 ? turnDistance * -1 : turnDistance;

  const float straightDistance = distanceToPoint(robot, object);

  return (straightDistance * moveCost + turnDistance * turnCost) * objectCost;
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
