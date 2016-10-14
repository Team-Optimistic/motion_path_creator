#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>

#include "motion_path_creator/mp_creator.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "motion_path_creator");
  ros::NodeHandle n;

  mpCreator<sensor_msgs::PointCloud2> mpc(n, "cloud", 10);

  //ros::Subscriber goatLidarSub = n.subscribe<sensor_msgs::PointCloud2>("cloud", 10, &mpCreator::callback, &mpc);

  return 0;
}
