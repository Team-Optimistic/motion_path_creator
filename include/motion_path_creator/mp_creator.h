#pragma once

#include <sensor_msgs/PointCloud2.h>

class mpCreator
{
public:
  mpCreator();

  void callback(const sensor_msgs::PointCloud2& in);
};
