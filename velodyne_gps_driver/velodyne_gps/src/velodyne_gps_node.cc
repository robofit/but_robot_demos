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
 * Rose node for convert RAW position data to sensor_msgs::NavSatFix
 *  
 */

#include <ros/ros.h>
#include <velodyne_gps/nmea_convertor.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "velodyne_gps_node");
  ros::NodeHandle node;
  ros::NodeHandle private_nh("~");

  
  //NMEAConvertor: RAW position data -> sensor_msgs::NavSatFix
  velodyne_gps::NMEAConvertor NMEAConvertor(private_nh, node);

  ros::spin();

  return 0;
}
