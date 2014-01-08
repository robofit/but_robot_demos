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
 *  NMEAConvertor: RAW position data -> sensor_msgs::NavSatFix
 *  
 */

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <velodyne_gps/nmea_convertor.h>
#include <math.h>

#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>

#include <sstream>
#include <string>
#include <ctime>

using namespace std;
namespace velodyne_gps
{

    NMEAConvertor::NMEAConvertor(ros::NodeHandle private_nh,ros::NodeHandle nh) {

      pub = nh.advertise<sensor_msgs::NavSatFix>("velodyne_gps_nav_msg", 1000);
      sub  = nh.subscribe("velodyne_gps_packet_raw", 1000, &NMEAConvertor::GPSRAWCallback, this);
    }

    NMEAConvertor::~NMEAConvertor() {


    }

    
    /**
     * Message accept callback
     * 
     */
    void NMEAConvertor::GPSRAWCallback(const velodyne_gps_driver::VelodyneGPSPacket::ConstPtr& msg) {

     
      //pointer to message data
      const raw_packet_t *raw = (const raw_packet_t *) &msg->data[0];
      const char * nmea_string_c= (const char *)  raw->NMEA;
      const char * gyro1= (const char *)  raw->gyro1;
      string nmea_type,utc_time, status, latitude_str,latitude_d_str, longitude_str, longitude_d_str, speed, track, dateformat, magnetic, magnetic_d, mode,checksum;

      float latitude,longitude,altitude;
      int time_in_seconds=0;
      
      
      //define messages
      sensor_msgs::NavSatFix nav_sat_fix_msg;
      sensor_msgs::NavSatStatus nav_sat_status_msg;
      
      
      stringstream stream(nmea_string_c);
      string nmea_string(nmea_string_c);

      
      getline(stream, nmea_type, ',') ;
	
	if(nmea_type != "$GPRMC")
	{
	  ;
	}
	else
	{
	  
	  ROS_INFO_ONCE("Getting first NMEA GPRMC message.....");
	  
	 
	  
	  //Parse
	  msg->header.stamp;
	  nav_sat_fix_msg.header.stamp= msg->stamp;
	  nav_sat_fix_msg.header.frame_id= msg->header.frame_id;
	  
	  getline(stream, utc_time, ',') ;
	  getline(stream, status, ',') ;
	  getline(stream, latitude_str, ',') ;
	  getline(stream, latitude_d_str, ',') ;
	  getline(stream, longitude_str, ',') ;
	  getline(stream, longitude_d_str, ',') ;
	  getline(stream, speed, ',') ;
	  getline(stream, track, ',') ;
	  getline(stream, dateformat, ',') ;
	  getline(stream, magnetic, ',') ;
	  getline(stream, magnetic_d, ',') ;
	  getline(stream, mode, '*') ;
	  getline(stream, checksum, '\r') ;
	  
	  
	  //With checksum is problem, because compared values are different
	  //cout << nmea_string <<endl;
	  //cout << getNMEAChecksum(nmea_string) <<endl;
	  //cout << hexstring_to_int(checksum) <<endl;
	  
	  //if(getNMEAChecksum(nmea_string)==hexstring_to_int(checksum))
	  //{
	  //}

	  
	  //convert latitude and longitude
	  latitude=string_to_float(latitude_str.substr(0,2))+string_to_float(latitude_str.substr(2))/60;
	  longitude=string_to_float(longitude_str.substr(0,3))+string_to_float(longitude_str.substr(3))/60;
	  

	  
	  //Time convert
	  struct tm when = {0};

	  when.tm_hour = string_to_int(latitude_str.substr(0,2));
	  when.tm_min = string_to_int(latitude_str.substr(2,2));
	  when.tm_sec = string_to_int(latitude_str.substr(4,2));

	  time_t converted;
	  converted = mktime(&when);
	  
	  
	  //Set longitude and latitude direction
	  if(longitude_d_str=="W")
	  {
	    longitude=-longitude;
	  }
	    
	  //
	  if(latitude_d_str=="S")
	  {
	    latitude=-latitude;
	  }
	  
	  altitude=NAN;
	  
	  
	  nav_sat_status_msg.service = nav_sat_status_msg.SERVICE_GPS;
	  
	  //If data are valid
	  if (status=="A")
	  {
	    nav_sat_status_msg.status=nav_sat_status_msg.STATUS_FIX;
	    nav_sat_fix_msg.status=nav_sat_status_msg;
	    
	  }
	  else
	  {
	    nav_sat_status_msg.status=nav_sat_status_msg.STATUS_NO_FIX;
	    nav_sat_fix_msg.status=nav_sat_status_msg;
	  }
	  
	  //TIME from GPS
	  ros::Time time=ros::Time(converted);
	  nav_sat_fix_msg.position_covariance_type=nav_sat_fix_msg.COVARIANCE_TYPE_UNKNOWN;


	  //Set position value to output message
	  nav_sat_fix_msg.latitude=latitude;
	  nav_sat_fix_msg.longitude=longitude;
	  nav_sat_fix_msg.altitude=altitude;

	  
	  
	  //Publish
	  pub.publish(nav_sat_fix_msg);
	}
    }
    
    
    /**
     * Convert string to float
     * 
     */
    double NMEAConvertor::string_to_float(string str)
    {
      istringstream buffer(str);
      float temp;
      buffer >> temp;
      return temp;
    }
    
    
    /**
     * Convert string to int
     * 
     */
    int NMEAConvertor::string_to_int(string str)
    {
      istringstream buffer(str);
      float temp;
      buffer >> temp;
      return temp;
    }
    
    /**
     * Convert hex string to int
     * 
     */
    int NMEAConvertor::hexstring_to_int(string str)
    {
      std::stringstream ss;
      ss << std::hex << str;


      int x;
      ss>>x;
      return x;
    }
    
    
    /**
     * 
     * NMEA checksum
     * 
     */
    
    int NMEAConvertor::getNMEAChecksum(string str)
    {
      int i=0;
      int XOR=0;
      int c=0;
      

      for (XOR = 0, i = 0; i < str.length(); i++) {
	c = (unsigned char)str.at(i);
	if (c == '*') break;
	if (c != '$') XOR ^= c;
      }
      return XOR;
    }
        
}