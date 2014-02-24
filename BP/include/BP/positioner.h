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

#include <BP/DetectBalls.h>
#include <BP/GoalCoords.h>

namespace BP {

  class Positioner 
  {
    public:
      Positioner(ros::NodeHandle& nh);
      ~Positioner();

    protected:

      ros::NodeHandle nh;
      ros::ServiceClient service_client;
      ros::Timer timer;

      image_transport::ImageTransport it;
      image_transport::Subscriber depth_sub;
      
      image_geometry::PinholeCameraModel cam_model;

      ros::Subscriber cam_info_sub;

      cv_bridge::CvImagePtr depth_img;

      tf::TransformListener tfl;

      ros::Publisher goal_pub;

      void timerCallback(const ros::TimerEvent& event);
      void depthImageCallback(const sensor_msgs::ImageConstPtr& msg);
      void camInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg);
      
      bool cameraTo3D(int center_x, int center_y, cv::Point3f* point3d);

   };


}



#endif
