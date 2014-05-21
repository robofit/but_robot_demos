/**
 * Author: Dagmar Prokopova
 * File: manipulator.h
 * Description: Header file for manipulator
 * Bachelor's thesis, 2013/2014
 *
 */


#ifndef MANIPULATOR_H_
#define MANIPULATOR_H_

#include <ros/ros.h>

#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>

#include <ball_picker/FlowCommands.h>
#include <ball_picker/FlowControl.h>

#define PI 3.14159
#define GRIPPERLENGTH 0.15

namespace ball_picker {

  class Manipulator 
  {
    public:
      Manipulator(ros::NodeHandle& nh);
      ~Manipulator();

    protected:

      int32_t constflowid;
      bool goal_recieved;
      bool angle_recieved;

      ros::NodeHandle nh;
      ros::Subscriber angle_sub;
      ros::Subscriber goal_coord_sub;
      ros::Subscriber control_sub;

      ros::ServiceClient control_client;
      ros::ServiceClient clear_globalmap_client;
      ros::ServiceClient clear_localmap_client;

      double goal_angle;
      double goal_height;
      double goal_distance;

      double shoulder1;
      double shoulder2;
      double armheight;
      double fixedshoulder;

      ros::Publisher right_finger_pub;
      ros::Publisher left_finger_pub;
      ros::Publisher shoulder_pan_pub;
      ros::Publisher shoulder_pitch_pub;
      ros::Publisher elbow_flex_pub;
      ros::Publisher wrist_roll_pub;

      void controlCallback(const ball_picker::FlowCommands& msg);
      void angleCallback(const std_msgs::Float64& msg);
      void goalCoordinatesCallback(const geometry_msgs::Pose& msg);
      
      void getAngles(double* pitch, double* elbow);
      bool getLength(const std::string source, const std::string dest, double * length);
   };

}


#endif
