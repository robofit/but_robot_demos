/**
 * Author: Dagmar Prokopova
 * File: detector.h
 * Description: Header file for detector
 * Bachelor's thesis, 2013/2014
 *
 */


#ifndef DETECTOR_H_
#define DETECTOR_H_

#include "opencv2/opencv.hpp"

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <ball_picker/DetectObjects.h>
#include <ball_picker/Detections.h>

namespace ball_picker {

  class Detector 
  {
    public:
      Detector(ros::NodeHandle& node);
      ~Detector();

    protected:

      cv_bridge::CvImagePtr rgb_img;
      bool image_recieved_flag;

      CvHaarClassifierCascade * hand_cascade;

      cv::Point center;
      int radius;
      CvRect * rect;
      bool ball_detection_ready_flag;
      bool hand_detection_ready_flag;

      ros::NodeHandle nh;
      image_transport::ImageTransport it;

      image_transport::Subscriber rgb_sub;
      ros::ServiceServer detect_ball_srv;
      ros::ServiceServer detect_hand_srv;

      void rgbImageCallback(const sensor_msgs::ImageConstPtr& msg);
      bool detectBalls(ball_picker::DetectObjects::Request &req, ball_picker::DetectObjects::Response &res);
      bool detectHands(ball_picker::DetectObjects::Request &req, ball_picker::DetectObjects::Response &res);
      void cvOpen(const CvArr *source, CvArr *dest, IplConvKernel *element);
      void cvClose(const CvArr *source, CvArr *dest, IplConvKernel *element);


   };


}



#endif
