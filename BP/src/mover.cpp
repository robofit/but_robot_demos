/**
 * Author: Dagmar Prokopova
 * File: mover.cpp
 * Description: Gives command for moving
 * Bachelor's thesis, 2013/2014
 *
 */

#include "BP/mover.h"

using namespace BP;
using namespace std;



/**
 * constructor of mover
 * establishes subscription and initializes action client
 */
Mover::Mover(ros::NodeHandle& node): nh(node), action_client("move_base", true)
{
  //define subscription on goal coordinates
  goal_sub = nh.subscribe("goal_coords", 1, &Mover::goalCoordinatesCallback, this);

  ROS_INFO_STREAM("Mover initialized.");
}


/**
 * destructor of mover
 */
Mover::~Mover() {}


/**
 * callback function called after new message appears in the topic with goal coordinates
 */
void Mover::goalCoordinatesCallback(const BP::GoalCoords& msg)
{
  ROS_INFO_ONCE("Goal coordinates received.");

  //wait for the action server to come up
  if(!action_client.waitForServer(ros::Duration(10.0)))
  {
    ROS_WARN_THROTTLE(1.0, "Move base server not available!");
    return;
  }


  //fill the goal for the robot
  goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = msg.x;
  goal.target_pose.pose.position.y = msg.y;
  goal.target_pose.pose.position.z = msg.z;

  goal.target_pose.pose.orientation.x = 0.0;
  goal.target_pose.pose.orientation.y = 0.0;
  goal.target_pose.pose.orientation.z = 0.0;
  goal.target_pose.pose.orientation.w = 1.0;

  //send the goal to the robot
  ROS_INFO("Sending goal");
  action_client.sendGoal(goal);
  action_client.waitForResult(ros::Duration(10.0));

  if(action_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("The base moved to the target.");
  }
  else
  {
    ROS_INFO("The base failed to move to the target.");

    //cancel all goals
    action_client.cancelAllGoals();
  }

}



int main (int argc, char** argv)
{
  ROS_INFO_STREAM("Mover started.");

  //ROS initialization
  ros::init(argc, argv, "mover");

  //create node
  ros::NodeHandle n;

  //create detector object
  Mover m(n);

  //keep the node alive
  ros::spin();

  return 0;
}
