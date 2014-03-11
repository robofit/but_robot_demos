/**
 * Author: Dagmar Prokopova
 * File: controller.h
 * Description: Header file for controller
 * Bachelor's thesis, 2013/2014
 *
 */


#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include <ros/ros.h>
#include <ball_picker/FlowControl.h>
#include <ball_picker/FlowCommands.h>

namespace ball_picker {

  class Controller 
  {
    public:
      Controller(ros::NodeHandle& node);
      ~Controller();

    protected:

      ros::NodeHandle nh;
      ros::ServiceServer service_server;

      int32_t state;
      ros::Publisher control_pub;

      bool controlflow(ball_picker::FlowControl::Request &req, ball_picker::FlowControl::Response &res);
   };


}



#endif
