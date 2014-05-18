/**
 * Author: Dagmar Prokopova
 * File: positioner.h
 * Description: Header file for positioner
 * Bachelor's thesis, 2013/2014
 *
 */


#ifndef POSITIONER_H_
#define POSITIONER_H_

#include <string>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float64.h>
 
#include <ball_picker/DetectObjects.h>
#include <ball_picker/FlowCommands.h>
#include <ball_picker/FlowControl.h>
#include <ball_picker/Detections.h>

#define PI 3.14159265
#define SPACE 0.45
#define FALSEANGLE 0.46

namespace ball_picker {


  class Positioner 
  {
    public:
      Positioner(ros::NodeHandle& nh);
      ~Positioner();

      struct DetCoords
      {
        cv::Point3f point;
        unsigned count;
      };


    protected:

      int32_t constflowid;

      ros::NodeHandle nh;
      ros::ServiceClient ball_detection_client;
      ros::ServiceClient hand_detection_client;
      ros::ServiceClient control_client;
      ros::WallTimer timer;
      ros::WallTimer limit;

      std::vector<DetCoords> detected_balls;
      std::vector<DetCoords> detected_hands;

      image_transport::ImageTransport it;
      image_transport::Subscriber depth_sub;
      
      image_geometry::PinholeCameraModel cam_model;

      ros::Subscriber cam_info_sub;
      ros::Subscriber control_sub;

      cv_bridge::CvImagePtr depth_img;

      tf::TransformListener tfl;

      ros::Publisher goal_pub;
      ros::Publisher angle_pub;
      ros::Publisher costmap_pub;
      ros::Publisher kinect_pub;


      double space;
      double falseangle;
      std::string transformframe;

      float kinectposition;

      void timerCallback(const ros::WallTimerEvent& event);
      void limitCallback(const ros::WallTimerEvent& event);
      void depthImageCallback(const sensor_msgs::ImageConstPtr& msg);
      void camInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg);
      void controlCallback(const ball_picker::FlowCommands& msg);
      
      bool cameraTo3D(int center_x, int center_y, cv::Point3f* point3d);
      geometry_msgs::Point getDistance(double a, double b);
   };


}



#endif
