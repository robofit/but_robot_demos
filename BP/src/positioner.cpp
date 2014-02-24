/**
 * Author: Dagmar Prokopova
 * File: positioner.cpp
 * Description: Trasformation of detected balls into real coordinates
 * Bachelor's thesis, 2013/2014
 *
 */

#include "BP/positioner.h"

using namespace BP;
using namespace std;
using namespace cv;


/**
 * constructor of positioner
 * establishes subscriptions and service client for detections
 */
Positioner::Positioner(ros::NodeHandle& node): nh(node), it(node)
{
  service_client = nh.serviceClient<DetectBalls>("/ball_detections");
  timer = nh.createTimer(ros::Duration(0.1), &Positioner::timerCallback, this);
  depth_sub = it.subscribe("depth_rect", 1, &Positioner::depthImageCallback, this);
  cam_info_sub = nh.subscribe("cam_info", 1, &Positioner::camInfoCallback, this);

  goal_pub = nh.advertise<BP::GoalCoords>("goal_coords", 1);


  depth_img.reset();

  ROS_INFO_STREAM("Positioner initialized.");
}


/**
 * destructor of positioner
 */
Positioner::~Positioner() {}


/**
 * periodical call of detection service
 */
void Positioner::timerCallback(const ros::TimerEvent& event)
{
  ROS_INFO_ONCE("Timer triggered.");

  if (!service_client.waitForExistence(ros::Duration(0.05)))
  {
    ROS_WARN_THROTTLE(1.0, "Service not available.");
    return;
  }

  if (!cam_model.initialized())
  {
    ROS_WARN_THROTTLE(1.0, "Camera model not initialized.");
    //return; //XXX ODKOMENTOVAT
  }

  DetectBalls srv;

  //fill the parameters for the service
  srv.request.balls = true;

  //call the service for detection
  if (!service_client.call(srv))
  {
    ROS_ERROR("Error on calling service!");
    return;
  }

  if (depth_img == NULL)
    return;



  float min = FLT_MAX;
  unsigned int nearest_index = 0;
  bool success = false;

  //process all the detections and find the nearest one
  for(unsigned int i=0; i < srv.response.detections.ballcenters.size(); i++)
  {

    Point3f p3f;

    if (cameraTo3D(srv.response.detections.ballcenters[i].x, srv.response.detections.ballcenters[i].y, &p3f))
      success = true;
    else
      continue;

    if (p3f.z < min)
    {
      min = p3f.z;
      nearest_index = i;
    }

  }

  //if no detections found or error occured, return
  if (!success)
    return; 


  Point3f point3d;

  if (!cameraTo3D(srv.response.detections.ballcenters[nearest_index].x, srv.response.detections.ballcenters[nearest_index].y, &point3d))
    return;


  //transformation from camera coordinates to robot coordinates system
  geometry_msgs::PoseStamped ps;
  ps.header.frame_id = cam_model.tfFrame();
  ps.header.stamp = ros::Time::now();

  ps.pose.position.x = point3d.x;
  ps.pose.position.y = point3d.y;
  ps.pose.position.z = point3d.z;

  ps.pose.orientation.x = 0;
  ps.pose.orientation.y = 0;
  ps.pose.orientation.z = 0;
  ps.pose.orientation.w = 1;

  if (!tfl.waitForTransform("/map", ps.header.frame_id, ps.header.stamp, ros::Duration(0.5)))
  {
    ROS_WARN_THROTTLE(1.0,"Transform not available!");
    return;
  }

  try
  {
    tfl.transformPose("/map", ps, ps);
  }
  catch (tf::TransformException& ex)
  {
    ROS_WARN("TF exception:\n%s", ex.what());
    return;
  }

  //LADICI vypis - pozdeji smaz
  cout << "x: " << ps.pose.position.x << ", y: " << ps.pose.position.y << ", z: " << ps.pose.position.z << endl; 


  //advertise the result on goal_coords topic
  BP::GoalCoords msg_goal;
  msg_goal.x = ps.pose.position.x;
  msg_goal.y = ps.pose.position.y;
  msg_goal.z = ps.pose.position.z;

  goal_pub.publish(msg_goal);
  ros::spinOnce();


}



/**
 * gets the depth and 
 */
bool Positioner::cameraTo3D(int center_x, int center_y, Point3f* point3d)
{
  //extract the depth in the centre of the ball
  float z = depth_img->image.at<float>(Point(center_x,center_y));

  //check for NAN
  if (z != z)
  {
    ROS_WARN("Unable to get depth.");
    return false;
  }

  Point2f point2d(center_x,center_y);
  *point3d = cam_model.projectPixelTo3dRay(point2d);

  point3d->z *= z;

  //LADICI VYPIS - zrusit pozdeji
  cout << "x:" << point3d->x << ", y: " << point3d->y << ", z: " << point3d->z << endl;

  return true;

}


/**
 * callback on incoming camera info msg
 */
void Positioner::camInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg)
{
  if (!cam_model.fromCameraInfo(msg))
  {
    ROS_WARN_THROTTLE(1.0, "Can't initialize camera model.");
  }
}


/**
 * callback on incoming depth msg
 */
void Positioner::depthImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  ROS_INFO_ONCE("Depth image received.");

  try
  {
    depth_img = cv_bridge::toCvCopy(msg, msg->encoding);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

}


int main(int argc, char **argv)
{
   ROS_INFO_STREAM("Positioner started.");

    //ROS initialization
    ros::init(argc, argv, "positioner");

    //create node
    ros::NodeHandle n;

    //create positioner object
    Positioner p(n);

    //keep the node alive
    ros::spin();

    return 0;
}
