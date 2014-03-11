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
Controller::Controller(ros::NodeHandle& node): nh(node), state(ball_picker::FlowCommands::SEARCH)
{
  //advertise service for checking the state
  service_server = nh.advertiseService("flow_control", &Controller::controlflow, this);

  //establish publisher for control topic
  control_pub = nh.advertise<ball_picker::FlowCommands>("flow_commands", 1);

  ROS_INFO_STREAM("Controller initialized.");
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
      case ball_picker::FlowCommands::SEARCH:
        if (req.state)
          state = ball_picker::FlowCommands::MOVETOBALL;
        else
          state = ball_picker::FlowCommands::TURN;
        break;
      case ball_picker::FlowCommands::TURN:
        if (req.state)
          state = ball_picker::FlowCommands::SEARCH;
        else
          state = ball_picker::FlowCommands::TURN;
        break;
      case ball_picker::FlowCommands::MOVETOBALL:
        if (req.state)
          state = ball_picker::FlowCommands::PICKBALL;
        else
          state = ball_picker::FlowCommands::MOVETOBALL;
        break;
      case ball_picker::FlowCommands::PICKBALL:
        if (req.state)
          state = ball_picker::FlowCommands::MOVETOSTORAGE;
        else
          state = ball_picker::FlowCommands::PICKBALL;
        break;
      case ball_picker::FlowCommands::MOVETOSTORAGE:
        if (req.state)
          state = ball_picker::FlowCommands::DROPBALL;
        else
          state = ball_picker::FlowCommands::MOVETOSTORAGE;
        break;
      case ball_picker::FlowCommands::DROPBALL:
        if (req.state)
          state = ball_picker::FlowCommands::SEARCH;
        else
          state = ball_picker::FlowCommands::DROPBALL;
        break;
    }
 
    //publish the changed state into control topic
    ball_picker::FlowCommands msg;
    msg.flowid = state;
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

  //create detector object
  Controller c(n);

  //keep the node alive
  ros::spin();

  return 0;
}
