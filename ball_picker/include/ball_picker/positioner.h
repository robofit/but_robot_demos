/**
 * Author: Dagmar Prokopova
 * File: positioner.h
 * Description: Header file for positioner
 * Bachelor's thesis, 2013/2014
 *
 */


#ifndef POSITIONER_H_
#define POSITIONER_H_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_listener.h>

#include <ball_picker/DetectBalls.h>
#include <ball_picker/GoalCoords.h>
#include <ball_picker/FlowCommands.h>
#include <ball_picker/FlowControl.h>

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
      ros::ServiceClient detection_client;
      ros::ServiceClient control_client;
      ros::WallTimer timer;
      ros::WallTimer limit;

      std::vector<DetCoords> detected_balls;

      image_transport::ImageTransport it;
      image_transport::Subscriber depth_sub;
      
      image_geometry::PinholeCameraModel cam_model;

      ros::Subscriber cam_info_sub;
      ros::Subscriber control_sub;

      cv_bridge::CvImagePtr depth_img;

      tf::TransformListener tfl;

      ros::Publisher goal_pub;

      void timerCallback(const ros::WallTimerEvent& event);
      void limitCallback(const ros::WallTimerEvent& event);
      void depthImageCallback(const sensor_msgs::ImageConstPtr& msg);
      void camInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg);
      void controlCallback(const ball_picker::FlowCommands& msg);
      
      bool cameraTo3D(int center_x, int center_y, cv::Point3f* point3d);

   };


}



#endif
