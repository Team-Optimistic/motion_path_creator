#include <ros/ros.h>
#include <string>

#include "motion_path_creator/mp_creator.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "motion_path_creator");
  mpCreator mpc();

  return 0;
}
