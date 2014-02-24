/**
 * Author: Dagmar Prokopova
 * File: detector.cpp
 * Description: Detection of tennis balls
 * Bachelor's thesis, 2013/2014
 *
 */

#include "BP/detector.h"

using namespace BP;
using namespace std;
using namespace cv;


/**
 * constructor of detector
 * establishes subscription and service for detections
 */
Detector::Detector(ros::NodeHandle& node): nh(node), it(node)
{
  //define subscription on rgb frames from kinect
  rgb_sub = it.subscribe("kinect_data", 10, &Detector::rgbImageCallback, this);

  //create a service for getting detections and register its callback function
  service_server = nh.advertiseService("detect_balls", &Detector::detect, this);

  image_recieved_flag = false;
  detection_ready_flag = false;

  ROS_INFO_STREAM("Detector initialized.");
}


/**
 * destructor of detector
 */
Detector::~Detector() {}


/**
 * callback function called after new message appears in the kinect topic
 * transforms obtained data into CV and saves into frame_rgb variable
 */
void Detector::rgbImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  ROS_INFO_ONCE("RGB image from kinect received.");

  try
  {
    rgb_img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  image_recieved_flag = true;



  //visualize the input and the detection
  IplImage img = rgb_img->image;
  IplImage * imgptr = &img;

  if (detection_ready_flag)
  {
    cvCircle(imgptr, center, 3, Scalar(0,255,0), -1, 8, 0 );
    cvCircle(imgptr, center, radius, Scalar(0,0,255), 3, 8, 0 );
  }

  cvShowImage("Original rgb image from kinect", imgptr);
  waitKey(10);

}


/**
 * service which detects tennis balls and fills data into the response
 */
bool Detector::detect(BP::DetectBalls::Request &req, BP::DetectBalls::Response &res)
{
  //check if any image frame is prepared for detection
  if (!image_recieved_flag)
    return false;

  IplImage img = rgb_img->image;

  //get the size of the image frame
  Size size = cvGetSize(&img);

  //transform into HSV (hue-saturation-value) model
  IplImage *hsv = cvCreateImage(size, IPL_DEPTH_8U, 3);
  cvCvtColor(&img, hsv, CV_BGR2HSV);

  //define tresholds and extract specified colors (yellow) from the image
  CvMat *mask = cvCreateMat(size.height, size.width, CV_8UC1);
  cvInRangeS(hsv, cvScalar(0.11*256, 0.30*256, 0.30*256, 0), cvScalar(0.16*256, 1.00*256, 1.00*256, 0), mask);

  //make it rougher by erode and dilate
  IplConvKernel *se21 = cvCreateStructuringElementEx(21, 21, 10, 10, CV_SHAPE_RECT, NULL);
  IplConvKernel *se11 = cvCreateStructuringElementEx(11, 11, 5, 5,  CV_SHAPE_RECT, NULL);
  cvClose(mask, mask, se21);
  cvOpen(mask, mask, se11);

  //prepare 8bit single channel grayscale image as an input for hough detection
  IplImage *hough_in = cvCreateImage(size, 8, 1);
  cvCopy(mask, hough_in, NULL);

  //make it smoother
  cvSmooth(hough_in, hough_in, CV_GAUSSIAN, 15, 15);

  //detect circles with Hough transformation
  CvMemStorage* storage = cvCreateMemStorage(0);
  CvSeq* circles = cvHoughCircles(hough_in, storage, CV_HOUGH_GRADIENT, 2, hough_in->height/10, 120, 65, 0, 0);

  //fill the response with type
  res.detections.type = BP::Detections::BALL;


  detection_ready_flag = false;

  //fill the response with centers of balls
  for(int i = 0; i < (int)circles->total; i++)
  {
    float *p = (float*)cvGetSeqElem(circles, i);

    BP::PointOfInterest det;
    det.x = cvRound(p[0]);
    det.y = cvRound(p[1]);

    center.x = cvRound(p[0]);
    center.y = cvRound(p[1]);
    radius = cvRound(p[2]);

    res.detections.ballcenters.push_back(det);

    detection_ready_flag = true;
  }

  ROS_INFO_STREAM_ONCE("Response of detection service set.");

  return true;

}


/**
 * function for mask improvement
 */
void Detector::cvOpen(const CvArr *source, CvArr *dest, IplConvKernel *element)
{
  cvErode (source, dest, element, 1);
  cvDilate(source, dest, element, 1);
}


/**
 * function for mask improvement
 */
void Detector::cvClose(const CvArr *source, CvArr *dest, IplConvKernel *element)
{
  cvDilate(source, dest, element, 1);
  cvErode (source, dest, element, 1);
}



int main (int argc, char** argv)
{
  ROS_INFO_STREAM("Detector started.");

  //ROS initialization
  ros::init(argc, argv, "detector");

  //create node
  ros::NodeHandle n;

  //create detector object
  Detector d(n);

  //keep the node alive
  ros::spin();

  return 0;
}
