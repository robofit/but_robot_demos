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
Manipulator::Manipulator(ros::NodeHandle& node): nh(node)
{

  constflowid = ball_picker::FlowCommands::INITPREPARE;
  goal_recieved = false;
  angle_recieved = false;
  goal_coordinates_recieved = false;


  goal_coord_sub = nh.subscribe("goal_coords", 1, &Manipulator::goalCoordinatesCallback, this);
  goal_sub = nh.subscribe("goal_distance", 1, &Manipulator::goalDistanceCallback, this); 
  angle_sub = nh.subscribe("angle", 1, &Manipulator::angleCallback, this);

  control_sub = nh.subscribe("flow_commands", 1, &Manipulator::controlCallback, this);

  control_client = nh.serviceClient<FlowControl>("/flow_control");
  clear_globalmap_client = nh.serviceClient<std_srvs::Empty>("/move_base/global_costmap/ball_picker_layer/ball_picker_clear_costmaps");
  clear_localmap_client = nh.serviceClient<std_srvs::Empty>("/move_base/local_costmap/ball_picker_layer/ball_picker_clear_costmaps");


  right_finger_pub = nh.advertise<std_msgs::Float64>("right_finger_command", 1); 
  left_finger_pub = nh.advertise<std_msgs::Float64>("left_finger_command", 1);
  shoulder_pan_pub = nh.advertise<std_msgs::Float64>("shoulder_pan_command", 1);
  shoulder_pitch_pub = nh.advertise<std_msgs::Float64>("shoulder_pitch_command", 1);  
  elbow_flex_pub = nh.advertise<std_msgs::Float64>("elbow_flex_command", 1);
  wrist_roll_pub = nh.advertise<std_msgs::Float64>("writs_roll_command", 1);


  ROS_INFO_STREAM("Manipulator initialized.");

}


/**
 * destructor of manipulator
 */
Manipulator::~Manipulator() {}


/**
 * callback function called after new message appears in the topic with goal distance
 */
void Manipulator::goalDistanceCallback(const std_msgs::Float64& msg)
{
  ROS_INFO("Goal distance received.");

  goal = msg;
  goal_recieved = true;
}

/**
 * callback function called after new message appears in the topic with angle value
 */
void Manipulator::angleCallback(const std_msgs::Float64& msg)
{
  ROS_INFO("Angle received.");

  angle = msg;
  angle_recieved = true;
}

/**
 * callback function called after new message appears in the topic with goal coordinates
 */
void Manipulator::goalCoordinatesCallback(const geometry_msgs::Pose& msg)
{
  ROS_INFO("Goal coordinates received.");

  goal_height = msg.position.z;
  goal_coordinates_recieved = true;
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
    std_msgs::Float64 panvalue;
    std_msgs::Float64 pitchvalue;
    std_msgs::Float64 elbowvalue;
    std_msgs::Float64 wristvalue;

    gripvalue.data = 0.0;
    panvalue.data = 0.0;
    pitchvalue.data = 0.0;
    elbowvalue.data = 0.0;
    wristvalue.data = 0.0;

    double temp, alpha, beta;

    FlowControl srv;
    srv.request.flowcmd.flowid = constflowid;
    srv.request.state = true;   

    switch(constflowid)
    {

      case ball_picker::FlowCommands::INITPREPARE:
        ROS_INFO("Getting manipulator into initial state.");
        panvalue.data = 0.0;
        pitchvalue.data = 2.12;
        elbowvalue.data = -2.1;
        wristvalue.data = 0.0;
        gripvalue.data = 0.0;

        shoulder_pan_pub.publish(panvalue);
        shoulder_pitch_pub.publish(pitchvalue);
        elbow_flex_pub.publish(elbowvalue);
        wrist_roll_pub.publish(wristvalue);
        right_finger_pub.publish(gripvalue);
        left_finger_pub.publish(gripvalue);
        ros::spinOnce();
        ros::Duration(1.5).sleep();

        break;

      case ball_picker::FlowCommands::DROPBALL:
        ROS_INFO("Dropping the ball on the floor.");
        panvalue.data = 0.0;
        pitchvalue.data = 0.15;
        elbowvalue.data = -1.5;
        wristvalue.data = 0.0;

        shoulder_pan_pub.publish(panvalue);
        shoulder_pitch_pub.publish(pitchvalue);
        elbow_flex_pub.publish(elbowvalue);
        wrist_roll_pub.publish(wristvalue);
        ros::spinOnce();
        ros::Duration(2.0).sleep();

        gripvalue.data = 0.5;
        left_finger_pub.publish(gripvalue);
        gripvalue.data = -gripvalue.data;
        right_finger_pub.publish(gripvalue);
        ros::spinOnce();
        ros::Duration(0.5).sleep();
       
        break;

      case ball_picker::FlowCommands::PICKBALL:
        ROS_INFO("Trying to pick the ball.");
        if (!goal_recieved)
        {
          ROS_ERROR("Goal distance not recieved.");
	  return;
        }

        if (!angle_recieved)
        {
          ROS_ERROR("Angle not recieved.");
	  return;
        }

        if (!goal_coordinates_recieved)
        {
          ROS_ERROR("Goal coordinates not recieved");
          return;
        }

	//LADICI vypisy - smazat
        cout << "distance: " << goal.data << endl;
        cout << "height: " << goal_height << endl;

        if (goal_height <= ARMHEIGHT)
        {
          temp = sqrt((ARMHEIGHT-goal_height)*(ARMHEIGHT-goal_height) + goal.data*goal.data);
          alpha = ((acos((temp*temp + SHOULDER1*SHOULDER1 - SHOULDER2*SHOULDER2)/(2*temp*SHOULDER1))) + (atan(goal.data/(ARMHEIGHT - goal_height)))) - PI/2;
        }
        else
        {
          temp = sqrt((goal_height-ARMHEIGHT)*(goal_height-ARMHEIGHT) + goal.data*goal.data);
          alpha = ((acos((temp*temp + SHOULDER1*SHOULDER1 - SHOULDER2*SHOULDER2)/(2*temp*SHOULDER1))) + (atan(goal.data/(goal_height - ARMHEIGHT))));
        }

        beta = acos((SHOULDER1*SHOULDER1 + SHOULDER2*SHOULDER2 - temp*temp)/(2*SHOULDER2*SHOULDER1));
        wristvalue.data = 0.0;

        //LADICI vypisy - smazat
        cout << "alpha: " << alpha << endl;
        cout << "beta: " << beta << endl;

        //if ((alpha > 1.9) || (alpha < 0.0) || (beta > 1.9) || (beta < 1.88) || (alpha != alpha) || (beta != beta))
        if ((alpha != alpha) || (beta != beta))
        {
          ROS_WARN("Invalid position of manipulator. Cannot reach the ball.");
          srv.request.state = false;
          break;
        }



	//open the gripper
	gripvalue.data = 0.5;
        left_finger_pub.publish(gripvalue);
        gripvalue.data = -gripvalue.data;
        right_finger_pub.publish(gripvalue);
        ros::spinOnce();
        ros::Duration(0.2).sleep();

        //get the arm to the ball
        panvalue.data = angle.data;
        pitchvalue.data = alpha;
        elbowvalue.data = -beta;

        shoulder_pan_pub.publish(panvalue);
        shoulder_pitch_pub.publish(pitchvalue);
        elbow_flex_pub.publish(elbowvalue);
        wrist_roll_pub.publish(wristvalue);
        ros::spinOnce();
        ros::Duration(2.0).sleep();

        //close the gripper
	gripvalue.data = 0.035;
        left_finger_pub.publish(gripvalue);
        gripvalue.data = -gripvalue.data;
        right_finger_pub.publish(gripvalue);
        ros::spinOnce();
        ros::Duration(0.5).sleep();

        //get the arm with picked ball into initial pose
        panvalue.data = 0.0;
        pitchvalue.data = 2.12;
        elbowvalue.data = -2.1;
        wristvalue.data = 0.0;

        shoulder_pan_pub.publish(panvalue);
        shoulder_pitch_pub.publish(pitchvalue);
        elbow_flex_pub.publish(elbowvalue);
        wrist_roll_pub.publish(wristvalue);
        ros::spinOnce();
        ros::Duration(2.0).sleep();

	break;

      case ball_picker::FlowCommands::PLACEBALL:
	ROS_INFO("Trying to place the ball into the hand.");
	if (!goal_recieved)
	{
 	  ROS_ERROR("Goal distance not recieved");
	  return;
	}
        
        if (!angle_recieved)
        {
          ROS_ERROR("Angle not recieved.");
          return;
        }

        if (!goal_coordinates_recieved)
        {
          ROS_ERROR("Goal coordinates not recieved");
          return;
        }

	//LADICI vypisy - smazat
        cout << "distance: " << goal.data << endl;


        if (goal_height <= ARMHEIGHT)
        {
          temp = sqrt((ARMHEIGHT-goal_height)*(ARMHEIGHT-goal_height) + goal.data*goal.data);
          alpha = ((acos((temp*temp + SHOULDER1*SHOULDER1 - SHOULDER2*SHOULDER2)/(2*temp*SHOULDER1))) + (atan(goal.data/(ARMHEIGHT - goal_height)))) - PI/2;
        }
        else
        {
          temp = sqrt((goal_height-ARMHEIGHT)*(goal_height-ARMHEIGHT) + goal.data*goal.data);
          alpha = ((acos((temp*temp + SHOULDER1*SHOULDER1 - SHOULDER2*SHOULDER2)/(2*temp*SHOULDER1))) + (atan(goal.data/(goal_height - ARMHEIGHT))));
        }


        beta = -acos((SHOULDER1*SHOULDER1 + SHOULDER2*SHOULDER2 - temp*temp)/(2*SHOULDER2*SHOULDER1));
        wristvalue.data = 0.0;

        //LADICI vypisy - smazat
        cout << "alpha: " << alpha << endl;
        cout << "beta: " << beta << endl;

        if ((alpha > 1.9) || (alpha < 0.0) || (beta > 1.9) || (beta < 1.88) || (alpha != alpha) || (beta != beta))
        {
          ROS_WARN("Invalid position of manipulator. Cannot reach the hand.");
          srv.request.state = false;
          break;
        }




        //get the arm to the ball
        panvalue.data = angle.data;
        pitchvalue.data = alpha;
        elbowvalue.data = -beta;
        wristvalue.data = 0.0;

        shoulder_pan_pub.publish(panvalue);
        shoulder_pitch_pub.publish(pitchvalue);
        elbow_flex_pub.publish(elbowvalue);
        wrist_roll_pub.publish(wristvalue);
        ros::spinOnce();
        ros::Duration(2.0).sleep();

        gripvalue.data = 0.5;
        left_finger_pub.publish(gripvalue);
        gripvalue.data = -gripvalue.data;
        right_finger_pub.publish(gripvalue);
        ros::spinOnce();
        ros::Duration(0.5).sleep();
 
	break;
    }

    //if we picked the ball, lets remove the ball from the costmap 
    if ((constflowid == ball_picker::FlowCommands::PICKBALL) && (srv.request.state == true))
    {
      //clear the costmaps
      std_srvs::Empty emptysrv;
      clear_globalmap_client.call(emptysrv);
      clear_localmap_client.call(emptysrv);
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
