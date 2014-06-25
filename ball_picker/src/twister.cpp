/**
 * Author: Dagmar Prokopova
 * File: twister.cpp
 * Description: Gives command for turning around in order to find some object
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
  constflowid = 0;

  //initialize watchdog
  watchdog = false;
  timer = nh.createWallTimer(ros::WallDuration(25.0), &Twister::timerCallback, this, false, false);

  //prepare the publisher for /cmd_vel topic
  twist_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  //subscribe to the flow_commands topick
  control_sub = nh.subscribe("flow_commands", 1, &Twister::controlCallback, this);

  //initialize control client
  control_client = nh.serviceClient<FlowControl>("/flow_control");

  ROS_INFO_STREAM("Twister initialized.");
}


/**
 * destructor of twister
 */
Twister::~Twister() {}


/**
 * timeout for rotation, will run out if the rotation takes too long
 */
void Twister::timerCallback(const ros::WallTimerEvent& event)
{
  timer.stop();
  ROS_INFO("Twister: Watchdog for the rotation timed out.");
  watchdog = true;
}



/**
 * callback on incoming control command
 */
void Twister::controlCallback(const ball_picker::FlowCommands& msg)
{

  if ((msg.flowid == ball_picker::FlowCommands::TURNBALL) ||
      (msg.flowid == ball_picker::FlowCommands::TURNHAND))
  {

    constflowid = msg.flowid;
    
    ROS_INFO("Twister: Starting to turn around.");

    //check availability of control client 
    if (!control_client.waitForExistence(ros::Duration(0.5)))
    {
      ROS_WARN_THROTTLE(1.0, "Control service not available.");
      return;
    }

    //check availability of transform from map to base_link
    if(!tfl.waitForTransform("base_link", "map", ros::Time(0), ros::Duration(1.0)))
    {
      ROS_WARN_THROTTLE(1.0, "Twister: Transform from map to base_link not available.");
      return;
    }

    tf::StampedTransform init_transform;
    tf::StampedTransform current_transform;

    //get initial transformation
    try
    {
      tfl.lookupTransform("base_link", "map", ros::Time(0), init_transform);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("TF exception:\n%s", ex.what());
      return;
    }


    geometry_msgs::Twist vel;

    vel.angular.z = 3.0;
    ros::Rate rate(15.0);

    bool done = false;
    watchdog = false;
    timer.start();


    while(!done && nh.ok())
    {
      if (watchdog)
        break;

      //send the turn command to the robot
      ROS_INFO_THROTTLE(2.0, "Twister: Sending the command to turn around.");
      twist_pub.publish(vel);

      ros::spinOnce();
      rate.sleep();

      //get current transform
      try
      {
        tfl.lookupTransform("base_link", "map", ros::Time(0), current_transform);
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("TF exception:\n%s", ex.what());
	return;
      }

      //get the relative angle
      tf::Transform relative_transform = init_transform.inverse() * current_transform;
      double angle_turned = relative_transform.getRotation().getAngle();

      //angle too small, no reason to do anything
      if (fabs(angle_turned) < 1.0e-2)
        continue;

      //desired angle reached
      if (angle_turned > ANGLE)
      {
        done = true;
        timer.stop();
        vel.angular.z = 0.0;
        ROS_INFO("Twister: Turn complete.");
      }
      else
      {
        //XXX this is only for run in simulator
        vel.angular.z = pow((ANGLE-angle_turned), 3)*5;
      }

    }

    //XXX this is needed only for run in simulator
    //wait for everything to be still
    ros::Duration(4.0).sleep();


    //set the response for the controller
    FlowControl srv;
    srv.request.flowcmd.flowid = constflowid;

    if (done)
      srv.request.state = true;
    else
      srv.request.state = false;

    //call the control service
    if (!control_client.call(srv))
    {
      ROS_ERROR("Twister: Error on calling control service!");
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
