#include <ros/ros.h>
#include <string>
#include <geometry_msgs/Point32.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Empty.h>

#include "motion_path_creator/mp_creator.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "motion_path_creator");
  mpCreator mpc();

  ros::NodeHandle n;
  ros::Subscriber scanSub, odomSub, robotPOSSub;

  scanSub = n.subscribe<sensor_msgs::LaserScan>("scan", 1000, &mpCreator::scanCallback, &mpc);
  odomSub = n.subscribe<nav_msgs::Odometry>("odometry/filtered", 1000, &mpCreator::odomCallback, &mpc);
  robotPOSSub = n.subscribe<std_msgs::Empty>("robotPOS/spcRequest", 1000, &mpCreator::robotPOSCallback, &mpc);

  ros::spin();

  return 0;
}
