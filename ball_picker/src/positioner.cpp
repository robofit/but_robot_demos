/**
 * Author: Dagmar Prokopova
 * File: positioner.cpp
 * Description: Trasformation of detected balls into real coordinates
 * Bachelor's thesis, 2013/2014
 *
 */

#include "ball_picker/positioner.h"

using namespace ball_picker;
using namespace std;
using namespace cv;


/**
 * constructor of positioner
 * establishes subscriptions and service client for detections
 */
Positioner::Positioner(ros::NodeHandle& node): nh(node), it(node)
{
  constflowid = ball_picker::FlowCommands::SEARCH;

  detection_client = nh.serviceClient<DetectBalls>("/ball_detections");
  control_client = nh.serviceClient<FlowControl>("/flow_control");

  timer = nh.createWallTimer(ros::WallDuration(0.1), &Positioner::timerCallback, this);
  limit = nh.createWallTimer(ros::WallDuration(3.0), &Positioner::limitCallback, this);

  depth_sub = it.subscribe("depth_rect", 1, &Positioner::depthImageCallback, this);
  cam_info_sub = nh.subscribe("cam_info", 1, &Positioner::camInfoCallback, this);
  control_sub = nh.subscribe("flow_commands", 1, &Positioner::controlCallback, this);
  
  goal_pub = nh.advertise<ball_picker::GoalCoords>("goal_coords", 1);

  depth_img.reset();

  detected_balls.clear();

  ROS_INFO_STREAM("Positioner initialized.");

}


/**
 * destructor of positioner
 */
Positioner::~Positioner() {}


/**
 * timeout for searching balls
 */
void Positioner::limitCallback(const ros::WallTimerEvent& event)
{


  ROS_INFO("Time limit timed out.");

  if (!control_client.waitForExistence(ros::Duration(0.5)))
  {
    ROS_WARN_THROTTLE(1.0, "Control service not available.");
    return;
  }

  //stop the periodical calling of detection service
  timer.stop();
  limit.stop();
  
  FlowControl srv;
  srv.request.flowcmd.flowid = constflowid;

  bool ballfound = false;
  Point3f point3d;

  if (!detected_balls.empty())
  {
    point3d.z = FLT_MAX;

    for (std::vector<DetCoords>::iterator iter = detected_balls.begin() ; iter != detected_balls.end(); iter++)
    {
      //LADICI vypisy - pozdeji smazat
      cout << "-------------" << endl;
      cout << iter->point.z << endl;
      cout << iter->count << endl;

      if ((iter->point.z < point3d.z) && (iter->count >= 5))
      {
        point3d.x = iter->point.x;
        point3d.y = iter->point.y;
        point3d.z = iter->point.z;

        ballfound = true;
      }
    }

  }

  //if any valid detections found, start with transform
  if (ballfound)
  {
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
      detected_balls.clear();
      timer.start();
      limit.start();
      return;
    }

    try
    {
      tfl.transformPose("/map", ps, ps);
    }
    catch (tf::TransformException& ex)
    {
      ROS_ERROR("TF exception:\n%s", ex.what());
      return;
    }

    //LADICI vypis - pozdeji smazat
    cout << "the nearest ball is on the position --> x: " << ps.pose.position.x << ", y: " << ps.pose.position.y << ", z: " << ps.pose.position.z << endl; 


    //advertise the result on goal_coords topic
    ball_picker::GoalCoords msg_goal;
    msg_goal.x = ps.pose.position.x;
    msg_goal.y = ps.pose.position.y;
    msg_goal.z = ps.pose.position.z;

    goal_pub.publish(msg_goal);
    ros::spinOnce();
 
    //change positive state into control service
    srv.request.state = true;
  }
  else
  {

    ROS_INFO("No tennis ball found.");
    //send negative state into control service
    srv.request.state = false;
  }

  //call the control service
  if (!control_client.call(srv))
  {
    ROS_ERROR("Error on calling control service!");
    return;
  }

}



/**
 * periodical call of detection service
 */
void Positioner::timerCallback(const ros::WallTimerEvent& event)
{


  ROS_INFO_ONCE("Timer triggered.");

  if (!detection_client.waitForExistence(ros::Duration(0.05)))
  {
    ROS_WARN_THROTTLE(1.0, "Detection service not available.");
    return;
  }

  if (!cam_model.initialized())
  {
    ROS_WARN_THROTTLE(1.0, "Camera model not initialized.");
    return;
  }

  DetectBalls srv;

  //fill the parameters for the service
  srv.request.balls = true;

  //call the service for detection
  if (!detection_client.call(srv))
  {
    ROS_ERROR("Error on calling service!");
    return;
  }

  if (depth_img == NULL)
  {
    ROS_WARN("Depth not available");
    return;
  }


  //process all the detections and find the nearest one
  for(unsigned int i=0; i < srv.response.detections.ballcenters.size(); i++)
  {
    DetCoords dcoords;

    if (cameraTo3D(srv.response.detections.ballcenters[i].x, srv.response.detections.ballcenters[i].y, &(dcoords.point)))
    {
        bool modified = false;
 
        for (std::vector<DetCoords>::iterator iter = detected_balls.begin() ; iter != detected_balls.end(); iter++)
        {
          if ( (abs(iter->point.x - dcoords.point.x) < 0.01) &&
               (abs(iter->point.y - dcoords.point.y) < 0.01) &&
               (abs(iter->point.z - dcoords.point.z) < 0.2e-37) )
          {
            iter->point.x = ((iter->point.x + dcoords.point.x)/2);
            iter->point.y = ((iter->point.y + dcoords.point.y)/2);
            iter->point.z = ((iter->point.z + dcoords.point.z)/2);
            iter->count++;

            modified = true;
          }

        }
       
        if (!modified)
        {
          dcoords.count = 1;
          detected_balls.push_back(dcoords);
        }
    }
  }
}



/**
 * gets the depth 
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

/**
 * callback on incoming control command
 */
void Positioner::controlCallback(const ball_picker::FlowCommands& msg)
{

  ROS_INFO_ONCE("Control command recieved.");

  if (msg.flowid == constflowid)
  {
    //reset vector with captured detections
    detected_balls.clear();
  
    ROS_INFO_STREAM("Timer started.");
 
    //start timers 
    timer.start();
    limit.start();
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
