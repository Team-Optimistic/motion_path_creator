#pragma once

#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>
#include <string>

template <class T>
class mpCreator
{
public:
  mpCreator(ros::NodeHandle& n, const std::string& topic, const int bufferSize)
  {
    sub = n.subscribe<T>(topic, bufferSize, &mpCreator::callback, this);
  }

  void callback(const sensor_msgs::PointCloud2::ConstPtr& in)
  {
    
  }

  ros::Subscriber sub;
};
