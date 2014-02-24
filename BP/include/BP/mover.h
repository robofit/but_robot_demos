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

#include <BP/GoalCoords.h>


namespace BP {

  class Mover 
  {
    public:
      Mover(ros::NodeHandle& nh);
      ~Mover();

    protected:

      ros::NodeHandle nh;
      ros::Subscriber goal_sub;

      actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> action_client;
      move_base_msgs::MoveBaseGoal goal;

      void goalCoordinatesCallback(const BP::GoalCoords& msg);

   };

}


#endif
