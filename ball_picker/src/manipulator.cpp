/**
 * Author: Dagmar Prokopova
 * File: maniuplator.cpp
 * Description: Picks and places the ball
 * Bachelor's thesis, 2013/2014
 *
 */

#include "ball_picker/manipulator.h"

using namespace ball_picker;
using namespace std;



/**
 * constructor of manipulator
 * establishes subscription and publisher
 */
Manipulator::Manipulator(ros::NodeHandle& node): nh(node), arm("arm")
{

  constflowid = ball_picker::FlowCommands::INITPREPARE;
  goal_recieved = false;

  goal_sub = nh.subscribe("goal_coords", 1, &Manipulator::goalCoordinatesCallback, this); 

  control_sub = nh.subscribe("flow_commands", 1, &Manipulator::controlCallback, this);

  control_client = nh.serviceClient<FlowControl>("/flow_control");
  clear_globalmap_client = nh.serviceClient<std_srvs::Empty>("/move_base/global_costmap/ball_picker_layer/ball_picker_clear_costmaps");
  clear_localmap_client = nh.serviceClient<std_srvs::Empty>("/move_base/local_costmap/ball_picker_layer/ball_picker_clear_costmaps");


  right_finger_pub = nh.advertise<std_msgs::Float64>("right_finger_command", 1); 
  left_finger_pub = nh.advertise<std_msgs::Float64>("left_finger_command", 1);

  ROS_INFO_STREAM("Manipulator initialized.");

}


/**
 * destructor of manipulator
 */
Manipulator::~Manipulator() {}


/**
 * callback function called after new message appears in the topic with goal coordinates
 */
void Manipulator::goalCoordinatesCallback(const geometry_msgs::Pose& msg)
{
  ROS_INFO("Goal coordinates received.");

  goal = msg;
  goal_recieved = true;
}




/**
 * callback on incoming control command
 */
void Manipulator::controlCallback(const ball_picker::FlowCommands& msg)
{
  if ((msg.flowid == ball_picker::FlowCommands::INITPREPARE) ||
      (msg.flowid == ball_picker::FlowCommands::PICKBALL) ||
      (msg.flowid == ball_picker::FlowCommands::PLACEBALL) ||
      (msg.flowid == ball_picker::FlowCommands::DROPBALL))
  {

    constflowid = msg.flowid;

    ROS_INFO("Starting the manipulation.");

    if (!control_client.waitForExistence(ros::Duration(0.5)))
    {
      ROS_WARN_THROTTLE(1.0, "Control service not available.");
      return;
    }

    if(!clear_globalmap_client.waitForExistence(ros::Duration(5.0)))
    {
      ROS_WARN_THROTTLE(1.0, "Clearing server for global costmap not available!");
      return;
    }

    if(!clear_localmap_client.waitForExistence(ros::Duration(5.0)))
    {
      ROS_WARN_THROTTLE(1.0, "Clearing server for local costmap not available!");
      return;
    }



    std_msgs::Float64 gripvalue;


    switch(constflowid)
    {

      case ball_picker::FlowCommands::INITPREPARE:
        ROS_INFO("Getting manipulator into initial state.");
        arm.setNamedTarget("cobra");
        gripvalue.data = 0.0;
        break;

      case ball_picker::FlowCommands::DROPBALL:
        ROS_INFO("Dropping the ball on the floor.");
        arm.setNamedTarget("drop");
        gripvalue.data = 0.5;
        break;

      case ball_picker::FlowCommands::PICKBALL:
        ROS_INFO("Trying to pick the ball.");
        if (!goal_recieved)
        {
          ROS_ERROR("Goal coordinates not recieved.");
	  return;
        }


	//LADICI vypisy - smazat
        cout << "x: " << goal.position.x << endl;
        cout << "y: " << goal.position.y << endl;
        cout << "z: " << goal.position.z << endl;


	//open the gripper
	gripvalue.data = 0.5;
        left_finger_pub.publish(gripvalue);
        gripvalue.data = -gripvalue.data;
        right_finger_pub.publish(gripvalue);
        ros::spinOnce();

        arm.setApproximateJointValueTarget(goal, arm.getEndEffectorLink());
	gripvalue.data = 0.035;
	break;

      case ball_picker::FlowCommands::PLACEBALL:
	ROS_INFO("Trying to place the ball into the hand.");
	if (!goal_recieved)
	{
 	  ROS_ERROR("Goal coordinates not recieved");
	  return;
	}
	arm.setApproximateJointValueTarget(goal, arm.getEndEffectorLink());
	gripvalue.data = 0.5;
	break;
    }


    FlowControl srv;
    srv.request.flowcmd.flowid = constflowid;

    if (arm.asyncMove())
      srv.request.state = true;
    else
      srv.request.state = false;

    ros::Duration(2.0).sleep();

    left_finger_pub.publish(gripvalue);
    gripvalue.data = -gripvalue.data;
    right_finger_pub.publish(gripvalue);

    ros::spinOnce();

    //if we are picking the ball and everything went well, get into the default position
    if ((constflowid == ball_picker::FlowCommands::PICKBALL) && (srv.request.state == true))
    {
      ros::Duration(0.4).sleep();
      arm.setNamedTarget("cobra");
      if (arm.asyncMove())
        srv.request.state = true;
      else
        srv.request.state = false;

      //clear the costmaps
      std_srvs::Empty emptysrv;
      clear_globalmap_client.call(emptysrv);
      clear_localmap_client.call(emptysrv);

      ros::Duration(2.0).sleep();
    }
    else
      ros::Duration(1.0).sleep();


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
  ROS_INFO_STREAM("Manipulator started.");

  //ROS initialization
  ros::init(argc, argv, "manipulator");

  //create node
  ros::NodeHandle n;
  
  //create initializer object
  Manipulator mp(n);

  //keep the node alive
  ros::spin();

  return 0;
}
