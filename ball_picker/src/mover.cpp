/**
 * Author: Dagmar Prokopova
 * File: mover.cpp
 * Description: Gives command for moving
 * Bachelor's thesis, 2013/2014
 *
 */

#include "ball_picker/mover.h"

using namespace ball_picker;
using namespace std;



/**
 * constructor of mover
 * establishes subscription and initializes action client
 */
Mover::Mover(ros::NodeHandle& node): nh(node), action_client("move_base", true)
{

  constflowid = ball_picker::FlowCommands::MOVETOBALL;
  goal_recieved = false;

  //define subscription on goal coordinates
  goal_sub = nh.subscribe("goal_coords", 1, &Mover::goalCoordinatesCallback, this);

  control_sub = nh.subscribe("flow_commands", 1, &Mover::controlCallback, this);

  control_client = nh.serviceClient<FlowControl>("/flow_control");

  ROS_INFO_STREAM("Mover initialized.");
}


/**
 * destructor of mover
 */
Mover::~Mover() {}


/**
 * callback function called after new message appears in the topic with goal coordinates
 */
void Mover::goalCoordinatesCallback(const geometry_msgs::Pose& msg)
{
  ROS_INFO("Goal coordinates received.");

  //fill the goal for the robot
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = msg.position.x;
  goal.target_pose.pose.position.y = msg.position.y;
  goal.target_pose.pose.position.z = msg.position.z;

  goal.target_pose.pose.orientation.x = msg.orientation.x;
  goal.target_pose.pose.orientation.y = msg.orientation.y;
  goal.target_pose.pose.orientation.z = msg.orientation.z;
  goal.target_pose.pose.orientation.w = msg.orientation.w;

  goal_recieved = true;
}


/**
 * callback on incoming control command
 */
void Mover::controlCallback(const ball_picker::FlowCommands& msg)
{

  if ((msg.flowid == ball_picker::FlowCommands::MOVETOBALL) ||
      (msg.flowid == ball_picker::FlowCommands::MOVETOHAND))
  {

    constflowid = msg.flowid;

    ROS_INFO("Starting the move process.");

    //wait for the action server to come up
    if(!action_client.waitForServer(ros::Duration(5.0)))
    {
      ROS_WARN_THROTTLE(1.0, "Move base server not available!");
      return;
    }


    if (!control_client.waitForExistence(ros::Duration(0.5)))
    {
      ROS_WARN_THROTTLE(1.0, "Control service not available.");
      return;
    }

    if (!goal_recieved)
    {
      ROS_ERROR("Goal coordinates not recieved.");
      return;
    }

    //send the goal to the robot
    ROS_INFO("Sending goal");
    action_client.sendGoal(goal);
    action_client.waitForResult(ros::Duration(30.0));

    FlowControl srv;
    srv.request.flowcmd.flowid = constflowid;

    if(action_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("The base moved to the target.");
      srv.request.state = true;
      goal_recieved = false;
    }
    else
    {
      ROS_INFO("The base failed to move to the target.");
      srv.request.state = false;

      //cancel all goals
      action_client.cancelAllGoals();
    }

    //call the control service
    if (!control_client.call(srv))
    {
      ROS_ERROR("Error on calling control service!");
      return;
    }

  }
}


int main (int argc, char** argv)
{
  ROS_INFO_STREAM("Mover started.");

  //ROS initialization
  ros::init(argc, argv, "mover");

  //create node
  ros::NodeHandle n;

  //create mover object
  Mover m(n);

  //keep the node alive
  ros::spin();

  return 0;
}
