/**
 * Author: Dagmar Prokopova
 * File: kinect_repeater.h
 * Description: Header file for kinect_repeater
 * Bachelor's thesis, 2013/2014
 *
 */


#ifndef KINECT_REPEATER_H_
#define KINECT_REPEATER_H_

#include <ros/ros.h>
#include <std_msgs/Float64.h>


namespace ball_picker {

  class KinectRepeater 
  {
    public:
      KinectRepeater(ros::NodeHandle& nh);
      ~KinectRepeater();

    protected:

      ros::NodeHandle nh;
      ros::Publisher kinect_pub;
      ros::Subscriber kinect_sub;

      std_msgs::Float64 kinect_msg;

      void repeaterCallback(const std_msgs::Float64& msg);

   };

}


#endif
