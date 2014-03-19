/**
 * Author: Dagmar Prokopova
 * File: twister.h
 * Description: Header file for twister
 * Bachelor's thesis, 2013/2014
 *
 */


#ifndef TWISTER_H_
#define TWISTER_H_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <ball_picker/FlowCommands.h>
#include <ball_picker/FlowControl.h>


namespace ball_picker {

  class Twister 
  {
    public:
      Twister(ros::NodeHandle& nh);
      ~Twister();

    protected:

      int32_t constflowid;

      ros::NodeHandle nh;
      ros::Subscriber control_sub;
      ros::Publisher twist_pub;

      ros::ServiceClient control_client;

      void controlCallback(const ball_picker::FlowCommands& msg);

   };

}


#endif
