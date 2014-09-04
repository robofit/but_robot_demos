/**
 * Author: Dagmar Prokopova
 * File: detector.cpp
 * Description: Detection of tennis balls and human hands
 * Bachelor's thesis, 2013/2014
 *
 */

#include "ball_picker/detector.h"

using namespace ball_picker;
using namespace std;
using namespace cv;


/**
 * constructor of detector
 * establishes subscription and services for detections
 */
Detector::Detector(ros::NodeHandle& node): nh(node), it(node)
{

  image_recieved_flag = false;
  ball_detection_ready_flag = false;
  hand_detection_ready_flag = false;

  //define subscription on rgb frames from kinect
  rgb_sub = it.subscribe("kinect_data", 10, &Detector::rgbImageCallback, this);

  //create a service for getting detections and register its callback function
  detect_ball_srv = nh.advertiseService("detect_balls", &Detector::detectBalls, this);
  detect_hand_srv = nh.advertiseService("detect_hands", &Detector::detectHands, this);

  //load the hand cascade classifier from parameter server
  ros::NodeHandle n_private("~");
  string handclassifier;

  if (n_private.getParam((string)"handclassifier", handclassifier))
  {
    ROS_INFO("Loading hand classifier.");
    hand_cascade = (CvHaarClassifierCascade*)cvLoad(handclassifier.c_str(), 0, 0, 0);

    if (hand_cascade == NULL)
      ROS_ERROR("Detector: Cannot load haar clasifier for hand detection.");
  }
  else
    ROS_ERROR("Detector: Cannot get the param value of classifier xml file location.");

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
  ROS_INFO_ONCE("Detector: First RGB image from kinect received.");

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

  if (ball_detection_ready_flag)
  {
    cvCircle(imgptr, center, 3, CV_RGB(0,255,0), -1, 8, 0 );
    cvCircle(imgptr, center, radius, CV_RGB(255,0,0), 3, 8, 0 );
  }

  if (hand_detection_ready_flag)
  {
    cvCircle(imgptr, cvPoint(rect->x + rect->width/2, rect->y + 2*rect->height/3), 3, CV_RGB(0,255,0), -1, 8, 0);
    cvRectangle(imgptr, cvPoint(rect->x, rect->y), cvPoint(rect->x + rect->width, rect->y + rect->height), CV_RGB(0,0,255), 3, 8, 0);
  }

  cvShowImage("Original image with captured detections", imgptr);
  waitKey(10);

}


/**
 * service which detects hands and fills data into the response
 */
bool Detector::detectHands(ball_picker::DetectObjects::Request &req, ball_picker::DetectObjects::Response &res)
{
  //check if any image frame is prepared for detection
  if (!image_recieved_flag)
    return false;

  IplImage img = rgb_img->image;

  //detect hands
  CvMemStorage * storage = cvCreateMemStorage(0);
  CvSeq * hands = cvHaarDetectObjects(&img, hand_cascade, storage, 1.2, 2, CV_HAAR_DO_CANNY_PRUNING, cvSize(100, 100));

  //fill the response with type
  res.detections.type = ball_picker::Detections::HAND;

  hand_detection_ready_flag = false;

  //fill the response with centers of hands
  for(int i = 0; i < (int)hands->total; i++)
  {
    rect = (CvRect*)cvGetSeqElem(hands, i);

    ball_picker::PointOfInterest det;
    det.x = rect->x + rect->width/2;
    det.y = rect->y + 2*rect->height/3;

    res.detections.objectcenters.push_back(det);

    hand_detection_ready_flag = true;
  }

  ROS_INFO_STREAM_ONCE("Response of detection service set.");

  return true;

}

/**
 * service which detects tennis balls and fills data into the response
 */
bool Detector::detectBalls(ball_picker::DetectObjects::Request &req, ball_picker::DetectObjects::Response &res)
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
  cvInRangeS(hsv, cvScalar(0.11*256, 0.20*256, 0.20*256, 0), cvScalar(0.16*256, 1.00*256, 1.00*256, 0), mask);

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
  res.detections.type = ball_picker::Detections::BALL;


  ball_detection_ready_flag = false;

  //fill the response with centers of balls
  for(int i = 0; i < (int)circles->total; i++)
  {
    float *p = (float*)cvGetSeqElem(circles, i);

    ball_picker::PointOfInterest det;
    det.x = cvRound(p[0]);
    det.y = cvRound(p[1]);

    center.x = cvRound(p[0]);
    center.y = cvRound(p[1]);
    radius = cvRound(p[2]);

    res.detections.objectcenters.push_back(det);

    ball_detection_ready_flag = true;
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
