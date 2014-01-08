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
 *  Driver for getting position packet from Velodyne HDL32E
 */

#include <string>
#include <cmath>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <velodyne_gps_driver/VelodyneGPSScan.h>

#include "driver.h"

namespace velodyne_gps_driver
{

  
  
VelodyneGPSDriver::VelodyneGPSDriver(ros::NodeHandle node,
                               ros::NodeHandle private_nh)
{
  // use private node handle to get parameters
  private_nh.param("frame_id", config_.frame_id, std::string("velodyne"));
  std::string tf_prefix = tf::getPrefixParam(private_nh);
  ROS_DEBUG_STREAM("tf_prefix: " << tf_prefix);
  config_.frame_id = tf::resolve(tf_prefix, config_.frame_id);

  std::string dump_file;
  int packet_rate=1;
  private_nh.param("pcap", dump_file, std::string(""));
  private_nh.param("packet_rate", packet_rate, 1);


  // open Velodyne input device or file
  if (dump_file != "")
  {
    input_.reset(new velodyne_gps_driver::InputPCAP(private_nh,packet_rate,dump_file));
  }
  else
  {
    input_.reset(new velodyne_gps_driver::InputSocket(private_nh));
  }

  // raw data (from position packet) output topic
  output_ = node.advertise<velodyne_gps_driver::VelodyneGPSPacket>("velodyne_gps_packet_raw", 10);
}



/** poll the device
 *
 *  @returns true unless end of file reacheds
 */
bool VelodyneGPSDriver::poll(void)
{
  // Allocate a new shared pointer for zero-copy sharing with other nodelets.
  velodyne_gps_driver::VelodyneGPSPacketPtr packet(new velodyne_gps_driver::VelodyneGPSPacket);

  while (true)
  {
      
    // keep reading until full packet received
    int rc = input_->getPacket(packet);
    if (rc == 0) break;       // got a full packet?
    if (rc < 0) return false; // end of file reached?


  }

  packet->header.stamp = ros::Time();
  packet->header.frame_id = config_.frame_id;
  output_.publish(packet);

  // notify diagnostics that a message has been published, updating
  // its status


  return true;
}

} 
