/*
 *  Copyright (C) 2013 FIT VUTBR, Tomas Goldmann
 * 
 *  xgoldm03@stud.fit.vutbr.cz
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

#ifndef CONVERTOR_H_
#define CONVERTOR_H_


#include <unistd.h>
#include <stdio.h>
#include <pcap.h>

#include <ros/ros.h>
#include <string>
#include "std_msgs/String.h"
#include <velodyne_gps_driver/VelodyneGPSPacket.h>


using namespace std;
namespace velodyne_gps
{        
    typedef struct raw_packet
    {
	uint8_t zero[14];
	uint8_t gyro1[8]; 
	uint8_t gyro2[8]; 
	uint8_t gyro3[8]; 
	uint8_t zero2[160]; 
	uint8_t t[4]; 
	uint8_t zero3[4];
	uint8_t NMEA[72];
	uint8_t zero4[234];
    } raw_packet_t;
    
  
    class NMEAConvertor
	{

	public:

		    NMEAConvertor(ros::NodeHandle private_nh,ros::NodeHandle nh);
		    ~NMEAConvertor();


	protected:


		    ros::Publisher pub;
		    ros::Subscriber sub;

		    void GPSRAWCallback(const velodyne_gps_driver::VelodyneGPSPacket::ConstPtr& msg);

		    ros::NodeHandle nh_;
	private:
		    double string_to_float(string str);
		    int string_to_int(string str);
		    int getNMEAChecksum(string str);
		    int hexstring_to_int(string str);

	};
}

#endif