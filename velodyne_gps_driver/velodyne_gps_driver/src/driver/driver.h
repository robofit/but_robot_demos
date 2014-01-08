/*
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2013 FIT VUTBR, Tomas Goldmannn
 *
 *  License: Modified BSD Software License Agreement
 *  This file is forked from velodyne_driver and modifed for position packet
 * 
 *  $Id$
 */




/** \file
 *
 *  ROS driver interface for the Velodyne 3D LIDARs position packet
 */



#ifndef _VELODYNE_DRIVER_H_
#define _VELODYNE_DRIVER_H_ 1

#include <string>
#include <ros/ros.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

#include <velodyne_gps_driver/input.h>

namespace velodyne_gps_driver
{

class VelodyneGPSDriver
{
public:

  VelodyneGPSDriver(ros::NodeHandle node,
                 ros::NodeHandle private_nh);
  ~VelodyneGPSDriver() {}

  bool poll(void);

private:

  // configuration parameters
  struct
  {
    std::string frame_id;            ///< tf frame ID
    std::string model;               ///< device model name
    int    npackets;                 ///< number of packets to collect
    double rpm;                      ///< device rotation rate (RPMs)
  } config_;

  boost::shared_ptr<Input> input_;
  ros::Publisher output_;

};

} // namespace velodyne_driver

#endif // _VELODYNE_DRIVER_H_
