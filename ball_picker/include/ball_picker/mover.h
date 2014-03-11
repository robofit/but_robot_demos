/**
 * Author: Dagmar Prokopova
 * File: mover.h
 * Description: Header file for mover
 * Bachelor's thesis, 2013/2014
 *
 */


#ifndef MOVER_H_
#define MOVER_H_

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <ball_picker/GoalCoords.h>
#include <ball_picker/FlowCommands.h>
#include <ball_picker/FlowControl.h>


namespace ball_picker {

  class Mover 
  {
    public:
      Mover(ros::NodeHandle& nh);
      ~Mover();

    protected:

      int32_t constflowid;
      bool goal_recieved;

      ros::NodeHandle nh;
      ros::Subscriber goal_sub;
      ros::Subscriber control_sub;

      ros::ServiceClient control_client;

      actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> action_client;
      move_base_msgs::MoveBaseGoal goal;

      void goalCoordinatesCallback(const ball_picker::GoalCoords& msg);
      void controlCallback(const ball_picker::FlowCommands& msg);


   };

}


#endif
