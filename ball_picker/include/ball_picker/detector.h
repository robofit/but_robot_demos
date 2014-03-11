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

#include <ball_picker/DetectBalls.h>
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

      cv::Point center;
      int radius;
      bool detection_ready_flag;

      ros::NodeHandle nh;
      image_transport::ImageTransport it;

      image_transport::Subscriber rgb_sub;
      ros::ServiceServer service_server;

      void rgbImageCallback(const sensor_msgs::ImageConstPtr& msg);
      bool detect(ball_picker::DetectBalls::Request &req, ball_picker::DetectBalls::Response &res);
      void cvOpen(const CvArr *source, CvArr *dest, IplConvKernel *element);
      void cvClose(const CvArr *source, CvArr *dest, IplConvKernel *element);


   };


}



#endif
