/**
 * Author: Dagmar Prokopova
 * File: manipulator.h
 * Description: Header file for manipulator
 * Bachelor's thesis, 2013/2014
 *
 */


#ifndef MANIPULATOR_H_
#define MANIPULATOR_H_

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose.h>
#include <std_srvs/Empty.h>

#include <ball_picker/FlowCommands.h>
#include <ball_picker/FlowControl.h>



namespace ball_picker {

  class Manipulator 
  {
    public:
      Manipulator(ros::NodeHandle& nh);
      ~Manipulator();

    protected:

      int32_t constflowid;
      bool goal_recieved;


      ros::NodeHandle nh;
      ros::Subscriber goal_sub;
      ros::Subscriber control_sub;

      ros::ServiceClient control_client;
      ros::ServiceClient clear_globalmap_client;
      ros::ServiceClient clear_localmap_client;

      moveit::planning_interface::MoveGroup arm;
      geometry_msgs::Pose goal;

      ros::Publisher right_finger_pub;
      ros::Publisher left_finger_pub;

      void controlCallback(const ball_picker::FlowCommands& msg);
      void goalCoordinatesCallback(const geometry_msgs::Pose& msg);

   };

}


#endif
