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
 * establishes subscriptions and publishers
 */
Manipulator::Manipulator(ros::NodeHandle& node): nh(node)
{

  constflowid = 0;
  goal_recieved = false;
  angle_recieved = false;

  //define subscription for goal and ganle income
  goal_coord_sub = nh.subscribe("goal_coords", 1, &Manipulator::goalCoordinatesCallback, this);
  angle_sub = nh.subscribe("goal_angle", 1, &Manipulator::angleCallback, this);

  //define subscription for flow command
  control_sub = nh.subscribe("flow_commands", 1, &Manipulator::controlCallback, this);

  //prepare control client
  control_client = nh.serviceClient<FlowControl>("/flow_control");

  //prepare services for clearing costmaps
  clear_globalmap_client = nh.serviceClient<std_srvs::Empty>("/move_base/global_costmap/ball_picker_layer/ball_picker_clear_costmaps");
  clear_localmap_client = nh.serviceClient<std_srvs::Empty>("/move_base/local_costmap/ball_picker_layer/ball_picker_clear_costmaps");

  //prepare publishers for separate parts of arm
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
 * callback function called after new message appears in the topic with angle value
 */
void Manipulator::angleCallback(const std_msgs::Float64& msg)
{
  ROS_INFO("Manipulator: Angle received.");

  goal_angle = msg.data;
  angle_recieved = true;
}

/**
 * callback function called after new message appears in the topic with goal coordinates
 */
void Manipulator::goalCoordinatesCallback(const geometry_msgs::Pose& msg)
{
  if ((constflowid == ball_picker::FlowCommands::CHECKBALL) || (constflowid == ball_picker::FlowCommands::CHECKHAND))
  {
    ROS_INFO("Manipulator: Goal coordinates received.");

    //find out some lenghts
    while ((!getLength( "arm_shoulder_pitch_link", "arm_elbow_flex_link", &shoulder1)) ||
      (!getLength( "arm_elbow_flex_link", "arm_wrist_roll_link", &shoulder2)) ||
      (!getLength( "arm_shoulder_pan_link", "arm_shoulder_pitch_link", &fixedshoulder)))
    {
      ROS_WARN_THROTTLE(1.0,"Cannot get arm parameters.");
    }

    //shoulder2 is actually longer
    shoulder2 = shoulder2 + GRIPPERLENGTH;

    //count the flat distance
    goal_distance = sqrt(msg.position.x*msg.position.x + msg.position.y*msg.position.y);

    //we must consider the fixed part of arm    
    goal_distance = goal_distance - fixedshoulder;
  
    goal_recieved = true;
  }
}


/**
 * callback on incoming control command
 */
void Manipulator::controlCallback(const ball_picker::FlowCommands& msg)
{
   constflowid = msg.flowid;

   if ((msg.flowid == ball_picker::FlowCommands::INITPREPARE) ||
      (msg.flowid == ball_picker::FlowCommands::PICKBALL) ||
      (msg.flowid == ball_picker::FlowCommands::PLACEBALL) ||
      (msg.flowid == ball_picker::FlowCommands::DROPBALL))
  {


    ROS_INFO("Manipulator: Starting the manipulation.");

    //check if controller is up
    if (!control_client.waitForExistence(ros::Duration(0.5)))
    {
      ROS_WARN_THROTTLE(1.0, "Manipulator: Control service not available.");
      return;
    }

    //check if we will be able to clean the global costmap
    if(!clear_globalmap_client.waitForExistence(ros::Duration(5.0)))
    {
      ROS_WARN_THROTTLE(1.0, "Manipulator: Clearing server for global costmap not available!");
      return;
    }

    //check if we will be able to clean the local costmap
    if(!clear_localmap_client.waitForExistence(ros::Duration(5.0)))
    {
      ROS_WARN_THROTTLE(1.0, "Manipulator: Clearing server for local costmap not available!");
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


    FlowControl srv;
    srv.request.flowcmd.flowid = constflowid;
    srv.request.state = true;   

    double shoulder_pitch_angle;
    double elbow_flex_angle;

    switch(constflowid)
    {

      case ball_picker::FlowCommands::INITPREPARE:
        ROS_INFO("Manipulator: Getting manipulator into initial state.");
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
        ROS_INFO("Manipulator: Dropping the ball on the floor.");
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
        ROS_INFO("Manipulator: Trying to pick the ball.");
        if (!angle_recieved)
        {
          ROS_ERROR("Manipulator: Angle not recieved.");
	  return;
        }

        if (!goal_recieved)
        {
          ROS_ERROR("Manipulator: Goal coordinates not recieved");
          return;
        }

        //count the angles
        getAngles(&shoulder_pitch_angle, &elbow_flex_angle);

        //check for NAN values which mean the arm cant reach the position.
        if ((shoulder_pitch_angle != shoulder_pitch_angle) || (elbow_flex_angle != elbow_flex_angle))
        {
          ROS_WARN("Manipulator: Invalid position of manipulator. Cannot reach the ball or hand.");
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
        panvalue.data = goal_angle;
        pitchvalue.data = shoulder_pitch_angle;
        elbowvalue.data = -elbow_flex_angle;
        wristvalue.data = 0.0;

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
	ROS_INFO("Manipulator: Trying to place the ball into the hand.");

	if (!goal_recieved)
	{
 	  ROS_ERROR("Manipulator: Goal distance not recieved");
	  return;
	}
        
        if (!angle_recieved)
        {
          ROS_ERROR("Manipulator: Angle not recieved.");
          return;
        }
      
        //count the angles 
        getAngles(&shoulder_pitch_angle, &elbow_flex_angle);

        //check for NAN values which mean the arm cant reach the position.
        if ((shoulder_pitch_angle != shoulder_pitch_angle) || (elbow_flex_angle != elbow_flex_angle))
        {
          ROS_WARN("Manipulator: Invalid position of manipulator. Cannot reach the hand.");
          srv.request.state = false;
          break;
        }


        //get the arm to the hand
        panvalue.data = goal_angle;
        pitchvalue.data = shoulder_pitch_angle;
        elbowvalue.data = -elbow_flex_angle;
        wristvalue.data = 0.0;

        shoulder_pan_pub.publish(panvalue);
        shoulder_pitch_pub.publish(pitchvalue);
        elbow_flex_pub.publish(elbowvalue);
        wrist_roll_pub.publish(wristvalue);
        ros::spinOnce();
        ros::Duration(2.0).sleep();

        //open the gripper
        gripvalue.data = 0.5;
        left_finger_pub.publish(gripvalue);
        gripvalue.data = -gripvalue.data;
        right_finger_pub.publish(gripvalue);
        ros::spinOnce();
        ros::Duration(0.5).sleep();
 
	break;
    }

    //if we picked the ball, lets remove the balls from the costmap 
    if ((constflowid == ball_picker::FlowCommands::PICKBALL) && (srv.request.state == true))
    {
      //clear the costmaps
      std_srvs::Empty emptysrv;
      clear_globalmap_client.call(emptysrv);
      clear_localmap_client.call(emptysrv);
    }

 
    //clean and prepare for next use
    goal_recieved = false;
    angle_recieved = false;

    //call the control service
    if (!control_client.call(srv))
    {
      ROS_ERROR("Manipulator: Error on calling control service!");
      return;
    }

  }
}

/**
 * Counts angles of shoulder_pitch_joint and elbow_flex_joint for picking detected balls.
 */
void Manipulator::getAngles(double * pitch, double * elbow)
{
  double alpha = atan(goal_height/goal_distance);

  //check whether we are above the base of arm or below
  if (goal_height < 0)
    alpha = -alpha;

  //count the real distance and angles
  double real_distance = sqrt(goal_height*goal_height + goal_distance*goal_distance);
  *pitch = acos((real_distance*real_distance + shoulder1*shoulder1 - shoulder2*shoulder2)/(2*real_distance*shoulder1)) + alpha;
  *elbow = acos((shoulder1*shoulder1 + shoulder2*shoulder2 - real_distance*real_distance)/(2*shoulder2*shoulder1));
}


/**
 * Function for counting distance between two coordinate frames, used for getting size parameters of arm.
 */
bool Manipulator::getLength(const string source, const string dest, double * length)
{

  tf::StampedTransform robot_transform;
  tf::TransformListener tfl;

  if (!tfl.waitForTransform(dest, source, ros::Time(0), ros::Duration(1.0)))
  {
    ROS_WARN_THROTTLE(1.0, "Manipulator: Transform not available!");
    return false;
  }

  try
  {
    tfl.lookupTransform(dest, source, ros::Time(0), robot_transform);
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR("TF exception:\n%s", ex.what());
    return false;
  }

  //count the length
  *length = sqrt(
    robot_transform.getOrigin().x()*robot_transform.getOrigin().x() + 
    robot_transform.getOrigin().y()*robot_transform.getOrigin().y() +
    robot_transform.getOrigin().z()*robot_transform.getOrigin().z()               );

  return true;
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
