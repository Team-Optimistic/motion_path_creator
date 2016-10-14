#pragma once

#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>
#include <string>

class mpCreator
{
public:
  mpCreator()
  {
    sub = n.subscribe<sensor_msgs::PointCloud2>("cloud", 10, &mpCreator::callback, this);
  }

  void callback(const sensor_msgs::PointCloud2::ConstPtr& in)
  {

  }

  ros::NodeHandle n;
  ros::Subscriber sub;
};
