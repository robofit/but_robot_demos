/*
 *  Copyright (C) 2013 FIT VUTBR, Tomas Goldmann
 * 
 *  xgoldm03@stud.fit.vutbr.cz
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
 *
 *  ROS driver node for the Velodyne 3D LIDARs (GPS packet).
 */

#include <ros/ros.h>
#include "driver.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "velodyne_gps_driver_node");
  ros::NodeHandle node;
  ros::NodeHandle private_nh("~");

  // start the driver
  velodyne_gps_driver::VelodyneGPSDriver dvr(node, private_nh);

  // loop until shut down or end of file
  while(ros::ok() && dvr.poll())
  {
    ros::spinOnce();
  }

  return 0;
}
