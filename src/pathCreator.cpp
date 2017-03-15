#include <ros/ros.h>
#include <string>
#define _USE_MATH_DEFINES
#include <math.h>
#include <vector>
#include <limits>
#include <sensor_msgs/point_cloud_conversion.h>
#include <algorithm>
#include <iterator>
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
  ros::Publisher pub = n.advertise<sensor_msgs::PointCloud2>("mpc/nextObjects", 10),
  pathPub = n.advertise<nav_msgs::Path>("mpc/path", 10);

  while (ros::ok())
  {
    ros::spinOnce(); //Callbacks

    std::vector<geometry_msgs::Point32> objList, finalObjList;
    geometry_msgs::Point32 coords = mpc.getCoords();

    //Add small objects
    for (auto&& obj : mpc.getSmallObjs().points)
      objList.push_back(obj);

    //Add big objects
    for (auto&& obj : mpc.getBigObjs().points)
      objList.push_back(obj);
    if(objList.size() != 0)
    {
    //Publish if we find a big object first
      if ((objList.front()).z == ObjTypes::big)
      {
       publishObjects(1, objList, pub);
     }
    //Else, we need to keep computing
     else
     {
       finalObjList.push_back(objList.front());
       coords = objList.front();
       objList.erase(objList.begin());

      //Loop until we have enough objects
       int objCount;
       for (objCount = 1; objCount <= 3;)
       {
        //If there are no objects left, publish what we have
         if (objList.size() == 0)
         {
           publishObjects(objCount, finalObjList, pub);
           break;
         }

        //Recalculate costs with new position
         std::sort(objList.begin(), objList.end(), [coords](geometry_msgs::Point32 a, geometry_msgs::Point32 b) {
          return sortByCost(coords, a, getTypeCost(a), b, getTypeCost(b));
        });

        //Add cheapest element to final list and remove it from overall list
        finalObjList.push_back(objList.front());
        objCount++; //We just added a new object so increment

        //If we have a small object followed by a big object
        if (objCount == 2 && (objList.front()).z == ObjTypes::big)
        {
          publishObjects(2, finalObjList, pub);
          break;
        }

        coords = objList.front(); //Move robot to that object's positoin
        objList.erase(objList.begin()); //Remove object from list so we don't consider it again
      }
      if(objCount ==3)
        publishObjects(3, finalObjList, pub);
    }
  }

    //Generate path
  nav_msgs::Path path;
  path.header.stamp = ros::Time::now();
  path.header.frame_id = "/field";
  path.poses.reserve(finalObjList.size());
  for (int i = 0; i < finalObjList.size(); i++)
  {
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "/field";
    pose.pose.position.x = finalObjList.at(i).x;
    pose.pose.position.y = finalObjList.at(i).y;
    pose.pose.position.z = 0;
    pose.pose.orientation.x = 0;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = 0;
    pose.pose.orientation.w = 1;
    path.poses.push_back(pose);
  }

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
  //Convert objList to PointCloud2
  ROS_INFO("mpc: publishing \n");

  sensor_msgs::PointCloud2 out;
  sensor_msgs::PointCloud temp;
  temp.points.reserve(numObjs);
  std::copy(objs.begin(), objs.begin() + numObjs, temp.points.begin());
  sensor_msgs::convertPointCloudToPointCloud2(temp, out);
  pub.publish(out);

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
