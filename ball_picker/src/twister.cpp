/**
 * Author: Dagmar Prokopova
 * File: twister.cpp
 * Description: Gives command for turning around to look for balls
 * Bachelor's thesis, 2013/2014
 *
 */

#include "ball_picker/twister.h"

using namespace ball_picker;
using namespace std;



/**
 * constructor of twister
 * establishes subscription and publisher
 */
Twister::Twister(ros::NodeHandle& node): nh(node)
{

  constflowid = ball_picker::FlowCommands::TURN;

  twist_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  control_sub = nh.subscribe("flow_commands", 1, &Twister::controlCallback, this);

  control_client = nh.serviceClient<FlowControl>("/flow_control");

  ROS_INFO_STREAM("Twister initialized.");
}


/**
 * destructor of twister
 */
Twister::~Twister() {}


/**
 * callback on incoming control command
 */
void Twister::controlCallback(const ball_picker::FlowCommands& msg)
{

  ROS_INFO("Control command recieved.");

  if (msg.flowid == constflowid)
  {
    ROS_INFO("Starting to turn around.");

    if (!control_client.waitForExistence(ros::Duration(0.5)))
    {
      ROS_WARN_THROTTLE(1.0, "Control service not available.");
      return;
    }

    geometry_msgs::Twist vel;

    vel.angular.z = 1.0;

    //send the turn command to the robot
    ROS_INFO("Sending the command to turn around.");
    twist_pub.publish(vel);

    ros::spinOnce();
 
    FlowControl srv;
    srv.request.flowcmd.flowid = constflowid;
    srv.request.state = true;

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
  ROS_INFO_STREAM("Twister started.");

  //ROS initialization
  ros::init(argc, argv, "twister");

  //create node
  ros::NodeHandle n;

  //create twister object
  Twister m(n);

  //keep the node alive
  ros::spin();

  return 0;
}
