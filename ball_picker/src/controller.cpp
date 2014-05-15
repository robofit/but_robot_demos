/**
 * Author: Dagmar Prokopova
 * File: controller.cpp
 * Description: State automata which controlls the flow of the application.
 * Bachelor's thesis, 2013/2014
 *
 */

#include "ball_picker/controller.h"

using namespace ball_picker;
using namespace std;



/**
 * constructor of controller
 * establishes service and publisher for control
 */
Controller::Controller(ros::NodeHandle& node): nh(node), state(ball_picker::FlowCommands::INITPREPARE)
{
  //advertise service for checking the state
  service_server = nh.advertiseService("flow_control", &Controller::controlflow, this);

  //establish publisher for control topic
  control_pub = nh.advertise<ball_picker::FlowCommands>("flow_commands", 1);

  ROS_INFO_STREAM("Controller initialized.");


  ROS_INFO_STREAM("Starting the loop.");

  //publish the initial state into control topic
  ball_picker::FlowCommands msg;
  msg.flowid = state;

  while(control_pub.getNumSubscribers() < SUBSCRIBERS)
    ROS_INFO_THROTTLE(1.0,"Controller: Waiting for all nodes to be subscribed.");

  control_pub.publish(msg);
  ros::spinOnce();



}


/**
 * destructor of controller
 */
Controller::~Controller() {}


/**
 * service which keeps, check and sets the state of the controller
 */
bool Controller::controlflow(ball_picker::FlowControl::Request &req, ball_picker::FlowControl::Response &res)
{
  ROS_INFO("Controller: Request recieved.");

  //check if the goal was completed and set new state
  if (req.flowcmd.flowid != ball_picker::FlowCommands::QUERY)
  {

    if (req.flowcmd.flowid != state)
    {
      ROS_ERROR("Invalid state.");
      return false;
    }

    switch(state)
    {
      case ball_picker::FlowCommands::INITPREPARE:
	if (req.state)
//          state = ball_picker::FlowCommands::SEARCHBALL;
          state = ball_picker::FlowCommands::CHECKBALL;
	else
	  state = ball_picker::FlowCommands::INITPREPARE;
	break;
      case ball_picker::FlowCommands::SEARCHBALL:
        if (req.state)
          state = ball_picker::FlowCommands::MOVETOBALL;
        else
          state = ball_picker::FlowCommands::TURNBALL;
        break;
      case ball_picker::FlowCommands::TURNBALL:
        if (req.state)
          state = ball_picker::FlowCommands::SEARCHBALL;
        else
          state = ball_picker::FlowCommands::TURNBALL;
        break;
      case ball_picker::FlowCommands::MOVETOBALL:
        if (req.state)
          state = ball_picker::FlowCommands::CHECKBALL;
        else
          state = ball_picker::FlowCommands::MOVETOBALL;
        break;
      case ball_picker::FlowCommands::CHECKBALL:
	if (req.state)
	  state = ball_picker::FlowCommands::PICKBALL;
	else
//	  state = ball_picker::FlowCommands::SEARCHBALL;
          state = ball_picker::FlowCommands::CHECKBALL;
	break;
      case ball_picker::FlowCommands::PICKBALL:
        if (req.state)
//          state = ball_picker::FlowCommands::SEARCHHAND;
          state = ball_picker::FlowCommands::INITPREPARE;
        else
          state = ball_picker::FlowCommands::CHECKBALL;
        break;
      case ball_picker::FlowCommands::SEARCHHAND:
	if (req.state)
	  state = ball_picker::FlowCommands::MOVETOHAND;
	else
	  state = ball_picker::FlowCommands::TURNHAND;
	break;
      case ball_picker::FlowCommands::TURNHAND:
	if (req.state)
	  state = ball_picker::FlowCommands::SEARCHHAND;
	else
	  state = ball_picker::FlowCommands::TURNHAND;
	break;
      case ball_picker::FlowCommands::MOVETOHAND:
        if (req.state)
          state = ball_picker::FlowCommands::CHECKHAND;
        else
          state = ball_picker::FlowCommands::MOVETOHAND;
        break;
      case ball_picker::FlowCommands::CHECKHAND:
	if (req.state)
	  state = ball_picker::FlowCommands::PLACEBALL;
	else
	  state = ball_picker::FlowCommands::SEARCHHAND;
	break;
      case ball_picker::FlowCommands::PLACEBALL:
        if (req.state)
          state = ball_picker::FlowCommands::INITPREPARE;
        else
          state = ball_picker::FlowCommands::DROPBALL;
        break;
      case ball_picker::FlowCommands::DROPBALL:
	if (req.state)
	  state = ball_picker::FlowCommands::INITPREPARE;
	else
	  state = ball_picker::FlowCommands::DROPBALL;
	break;
    }
 
    //publish the changed state into control topic
    ball_picker::FlowCommands msg;
    msg.flowid = state;

    while(control_pub.getNumSubscribers() < SUBSCRIBERS)
      ROS_INFO_THROTTLE(1.0,"Controller: Waiting for all nodes to be subscribed.");

    control_pub.publish(msg);
    ros::spinOnce();
  }

  //fill the response with new state
  res.flowcmd.flowid = state;

  ROS_INFO_STREAM_ONCE("Response of controll service set.");
  return true;
}


int main (int argc, char** argv)
{
  ROS_INFO_STREAM("Controller started.");

  //ROS initialization
  ros::init(argc, argv, "controller");

  //create node
  ros::NodeHandle n;

  //create controller object
  Controller c(n);

  //keep the node alive
  ros::spin();

  return 0;
}
