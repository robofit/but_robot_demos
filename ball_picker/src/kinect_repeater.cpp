/**
 * Author: Dagmar Prokopova
 * File: kinect_repeater.cpp
 * Description: Controlls the direction of kinect in real environment
 * Bachelor's thesis, 2013/2014
 *
 */

#include "ball_picker/kinect_repeater.h"

using namespace ball_picker;
using namespace std;



/**
 * constructor of KinectRepeater
 * establishes subscription and publisher
 */
KinectRepeater::KinectRepeater(ros::NodeHandle& node): nh(node)
{

  kinect_pub = nh.advertise<std_msgs::Float64>("kinect_command", 1);
  kinect_sub = nh.subscribe("kinect_value", 1, &KinectRepeater::repeaterCallback, this);

  kinect_msg.data = 0.0;

  ROS_INFO_STREAM("KinectRepeater initialized.");

  ros::Rate rate(30.0);

  while(nh.ok())
  {
    kinect_pub.publish(kinect_msg);
    ros::spinOnce();
    rate.sleep();
  }


}


/**
 * destructor of repeater
 */
KinectRepeater::~KinectRepeater() {}


/**
 * callback on incoming new value to be repeated
 */
void KinectRepeater::repeaterCallback(const std_msgs::Float64& msg)
{
  kinect_msg.data = msg.data;
}


int main (int argc, char** argv)
{
  ROS_INFO_STREAM("KinectRepeater started.");

  //ROS initialization
  ros::init(argc, argv, "kinectrepeater");

  //create node
  ros::NodeHandle n;

  //create kinectrepeater object
  KinectRepeater kr(n);

  //keep the node alive
  ros::spin();

  return 0;
}
