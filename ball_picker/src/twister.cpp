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

  constflowid = ball_picker::FlowCommands::TURNBALL;

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

  if ((msg.flowid == ball_picker::FlowCommands::TURNBALL) ||
      (msg.flowid == ball_picker::FlowCommands::TURNHAND))
  {

    constflowid = msg.flowid;
    
    ROS_INFO("Starting to turn around.");

    if (!control_client.waitForExistence(ros::Duration(0.5)))
    {
      ROS_WARN_THROTTLE(1.0, "Control service not available.");
      return;
    }

    tfl.waitForTransform("base_link", "odom", ros::Time(0), ros::Duration(1.0));

    tf::StampedTransform start_transform;
    tf::StampedTransform current_transform;

    tfl.lookupTransform("base_link", "odom", ros::Time(0), start_transform);


    geometry_msgs::Twist vel;

    vel.angular.z = 50.0;

    ros::Rate rate(5.0);
    bool done = false;

    while(!done && nh.ok())
    {
      //send the turn command to the robot
      ROS_INFO_THROTTLE(2.0, "Sending the command to turn around.");
      twist_pub.publish(vel);

      ros::spinOnce();
      rate.sleep();


      try
      {
        tfl.lookupTransform("base_link", "odom", ros::Time(0), current_transform);
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("TF exception:\n%s", ex.what());
	return;
      }

      tf::Transform relative_transform = start_transform.inverse() * current_transform;
      tf::Vector3 actual_turn_axis = relative_transform.getRotation().getAxis();
      double angle_turned = relative_transform.getRotation().getAngle();

      if (fabs(angle_turned) < 1.0e-2)
        continue;

      tf::Vector3 desired_turn_axis(0,0,1);
      if (actual_turn_axis.dot(desired_turn_axis) < 0)
        angle_turned = 2 * PI - angle_turned;

      if (angle_turned > PI/4)
      {
        done = true;
        ROS_INFO("Turn complete.");
      }

    }


    ros::Duration(4.0).sleep();

    FlowControl srv;
    srv.request.flowcmd.flowid = constflowid;

    if (done)
      srv.request.state = true;
    else
      srv.request.state = false;

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
  Twister t(n);

  //keep the node alive
  ros::spin();

  return 0;
}
