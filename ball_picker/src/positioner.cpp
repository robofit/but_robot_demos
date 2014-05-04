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
  constflowid = ball_picker::FlowCommands::SEARCHBALL;

  ball_detection_client = nh.serviceClient<DetectObjects>("/ball_detections");
  hand_detection_client = nh.serviceClient<DetectObjects>("/hand_detections");
  control_client = nh.serviceClient<FlowControl>("/flow_control");

  timer = nh.createWallTimer(ros::WallDuration(0.1), &Positioner::timerCallback, this, false, false);
  limit = nh.createWallTimer(ros::WallDuration(3.0), &Positioner::limitCallback, this, false, false);

  depth_sub = it.subscribe("depth_rect", 1, &Positioner::depthImageCallback, this);
  cam_info_sub = nh.subscribe("cam_info", 1, &Positioner::camInfoCallback, this);
  control_sub = nh.subscribe("flow_commands", 1, &Positioner::controlCallback, this);
  odom_sub = nh.subscribe("odom", 1, &Positioner::odometryCallback, this);
  
  goal_pub = nh.advertise<geometry_msgs::Pose>("goal_coords", 1);
  costmap_pub = nh.advertise<ball_picker::Detections>("costmap_custom_obstacles", 1);
  kinect_pub = nh.advertise<std_msgs::Float64>("kinect_servo",1);

  depth_img.reset();

  odometry_recieved = false;

  space = SPACE;
  height = 0.0;
  falseangle = 0.0;
  pitch = 0.0;
  transformframe = "/map";

  kinectposition = 0.0;

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
  geometry_msgs::Pose msg_goal;
  ball_picker::Detections msg_costmap;
  msg_costmap.type = ball_picker::Detections::BALL;
  ball_picker::PointOfInterest obstacle;

  if (!detected_obstacles.empty())
  {
    for (std::vector<DetCoords>::iterator iter = detected_obstacles.begin() ; iter != detected_obstacles.end(); iter++)
    {
      //transformation from camera coordinates to robot coordinates system
      geometry_msgs::PoseStamped ps;
      ps.header.frame_id = cam_model.tfFrame();
      ps.header.stamp = ros::Time::now();

      ps.pose.position.x = iter->point.x;
      ps.pose.position.y = iter->point.y;
      ps.pose.position.z = iter->point.z;

      ps.pose.orientation.x = 0.0;
      ps.pose.orientation.y = 0.0;
      ps.pose.orientation.z = 0.0;
      ps.pose.orientation.w = 1.0;

      if (!tfl.waitForTransform("/map", ps.header.frame_id, ps.header.stamp, ros::Duration(1.0)))
      {
        ROS_WARN_THROTTLE(1.0,"Transform not available!");
        detected_targets.clear();
	detected_obstacles.clear();
        timer.start();
        limit.start();
        return;
      }

      try
      {
        tfl.transformPose(transformframe, ps, ps);
      }
      catch (tf::TransformException& ex)
      {
        ROS_ERROR("TF exception:\n%s", ex.what());
        return;
      }
 
      if (iter->count > 1)
      {
        obstacle.x = ps.pose.position.x;
        obstacle.y = ps.pose.position.y;
        msg_costmap.objectcenters.push_back(obstacle);
      }
    }
  }




  if (!detected_targets.empty())
  {
    point3d.z = FLT_MAX;

    for (std::vector<DetCoords>::iterator iter = detected_targets.begin() ; iter != detected_targets.end(); iter++)
    {
      //LADICI vypisy - pozdeji smazat
      cout << "-------------" << endl;
      cout << iter->point.x << endl;
      cout << iter->point.y << endl;
      cout << iter->point.z << endl;
      cout << iter->count << endl;



      //transformation from camera coordinates to robot coordinates system
      geometry_msgs::PoseStamped ps;
      ps.header.frame_id = cam_model.tfFrame();
      ps.header.stamp = ros::Time::now();

      ps.pose.position.x = iter->point.x;
      ps.pose.position.y = iter->point.y;
      ps.pose.position.z = iter->point.z;

      ps.pose.orientation.x = 0.0;
      ps.pose.orientation.y = 0.0;
      ps.pose.orientation.z = 0.0;
      ps.pose.orientation.w = 1.0;

      if (!tfl.waitForTransform(transformframe, ps.header.frame_id, ps.header.stamp, ros::Duration(0.5)))
      {
        ROS_WARN_THROTTLE(1.0,"Transform not available!");
        detected_targets.clear();
	detected_obstacles.clear();
        timer.start();
        limit.start();
        return;
      }

      try
      {
        tfl.transformPose(transformframe, ps, ps);
      }
      catch (tf::TransformException& ex)
      {
        ROS_ERROR("TF exception:\n%s", ex.what());
        return;
      }
 
      if ((iter->count > 1) && (constflowid == ball_picker::FlowCommands::SEARCHBALL))
      {
        obstacle.x = ps.pose.position.x;
        obstacle.y = ps.pose.position.y;
        msg_costmap.objectcenters.push_back(obstacle);
      }

      if ((iter->point.z < point3d.z) && (iter->count >= 5))
      {
        point3d.z = iter->point.z;

        msg_goal.position.x = ps.pose.position.x;
        msg_goal.position.y = ps.pose.position.y;
        msg_goal.position.z = ps.pose.position.z;

        msg_goal.orientation.x = 0.0;
        msg_goal.orientation.y = 0.0;
        msg_goal.orientation.z = 0.0;
        msg_goal.orientation.w = 1.0;

        ballfound = true;
      }
    }

  }

  //if any valid detections found, start with transform
  if (ballfound)
  {
    //LADICI vypis - pozdeji smazat
    cout << "the nearest ball is on the position --> x: " << msg_goal.position.x << ", y: " << msg_goal.position.y << ", z: " << msg_goal.position.z << endl; 



    int modif_x = 1;
    int modif_y = 1;
    double base_angle = 0;
    double angle = 0;

    while (!odometry_recieved)
      ;

    if (msg_goal.position.x >= origin.x)
    {
      //II. quadrant
      if (msg_goal.position.y >= origin.y)
      {
        modif_x = 1;
        modif_y = 1;
      }
      //I. quadrant
      else
      {
        modif_x = 1;
        modif_y = -1;
      }
      
      base_angle = 0;
    }
    else
    {
      //IV. quadrant
      if (msg_goal.position.y <= origin.y)
      {
        modif_x = -1;
        modif_y = -1;
      }
      //III. quadrant
      else
      {
        modif_x = -1;
        modif_y = 1;
      }

      base_angle = PI;
    }

    geometry_msgs::Point diff = getDistance(fabs((modif_x)*(msg_goal.position.x-origin.x)), fabs((modif_y)*(msg_goal.position.y-origin.y)));
    msg_goal.position.x = origin.x + ((modif_x)*diff.x);
    msg_goal.position.y = origin.y + ((modif_y)*diff.y);
    
    if (fabs(msg_goal.position.x-origin.x) < 0.00001)
      angle = PI;
    else
      angle = atan(fabs((modif_y)*(msg_goal.position.y-origin.y))/fabs((modif_x)*(msg_goal.position.x-origin.x)));    


    angle = (modif_x)*(modif_y)*(angle - base_angle);


    angle = angle - falseangle;

     tf::Quaternion q;
     q.setRPY(0.0, pitch, angle);
     tf::quaternionTFToMsg(q, msg_goal.orientation);

    //msg_goal.position.z = msg_goal.position.z + height;

    //advertise the result on goal_coords topic
    goal_pub.publish(msg_goal);
    ros::spinOnce();
 
    //send positive state into control service
    srv.request.state = true;
  }
  else
  {

    ROS_INFO("No object found.");
    //send negative state into control service
    srv.request.state = false;
  }

  if (!msg_costmap.objectcenters.empty())
  {
    costmap_pub.publish(msg_costmap);
    ros::spinOnce();
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


  ROS_INFO_ONCE("Timer triggered for the first time.");

  if (!ball_detection_client.waitForExistence(ros::Duration(0.05)))
  {
    ROS_WARN_THROTTLE(1.0,"Ball detection service not available.");
    return;
  }

  if (!hand_detection_client.waitForExistence(ros::Duration(0.05)))
  {
    ROS_WARN_THROTTLE(1.0,"Hand detection service not available.");
    return;
  }

  if (!cam_model.initialized())
  {
    ROS_WARN_THROTTLE(1.0, "Camera model not initialized.");
    return;
  }

  vector<ball_picker::Detections> dets;
  dets.clear();



  DetectObjects srv;

  //fill the parameters for the service
  srv.request.all = true;


  if (constflowid != ball_picker::FlowCommands::CHECKHAND)
  {
    //call the service for detection
    if (!ball_detection_client.call(srv))
    {
      ROS_WARN_THROTTLE(1.0,"Error on calling ball detection service!");
      return;
    }
    dets.push_back(srv.response.detections);
  }


  if ((constflowid == ball_picker::FlowCommands::SEARCHHAND) ||
      (constflowid == ball_picker::FlowCommands::CHECKHAND))
  {
     //call the service for detection
    if (!hand_detection_client.call(srv))
    {
      ROS_WARN_THROTTLE(1.0,"Error on calling hand detection service!");
      return;
    }
    dets.push_back(srv.response.detections);
  }



  if (depth_img == NULL)
  {
    ROS_WARN_THROTTLE(1.0,"Depth not available");
    return;
  }
 
  vector<DetCoords> *storage_ptr; 
  
  for (std::vector<ball_picker::Detections>::iterator detsit = dets.begin(); detsit != dets.end(); detsit++)
  {

    if ((constflowid == ball_picker::FlowCommands::SEARCHHAND) && (detsit->type == ball_picker::Detections::BALL))
      storage_ptr = &detected_obstacles;
    else
      storage_ptr = &detected_targets;


    //process all the detections and find the nearest one
    for(unsigned int i=0; i < detsit->objectcenters.size(); i++)
    {
      DetCoords dcoords;

      if (cameraTo3D(detsit->objectcenters[i].x, detsit->objectcenters[i].y, &(dcoords.point)))
      {
        bool modified = false;


        for (std::vector<DetCoords>::iterator iter = storage_ptr->begin() ; iter != storage_ptr->end(); iter++)
        {
          if ( (fabs(iter->point.x - dcoords.point.x) < 0.01) &&
               (fabs(iter->point.y - dcoords.point.y) < 0.01) /*&&
               (fabs(iter->point.z - dcoords.point.z) < 0.2e-37) */)
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
          storage_ptr->push_back(dcoords);
        }
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
    ROS_WARN_THROTTLE(1.0,"Unable to get depth.");
    return false;
  }

  Point2f point2d(center_x,center_y);
  *point3d = cam_model.projectPixelTo3dRay(point2d);

  point3d->z *= z;

  return true;

}


/**
 * counts the x and y distance of the goal position
 */
geometry_msgs::Point Positioner::getDistance(double a, double b)
{
  geometry_msgs::Point result;

  double distance = sqrt(a*a+b*b) - space;
  
  if (a == 0)
  {
    result.x = 0;
    result.y = distance;
    return result;
  }
  
  if (b == 0)
  {
    result.x = distance;
    result.y = 0;
    return result;
  }
 
  result.x = sqrt((a*a*distance*distance)/(a*a+b*b));
  result.y = (b/a)*result.x;
  return result;
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

  if ((msg.flowid == ball_picker::FlowCommands::SEARCHBALL) ||
      (msg.flowid == ball_picker::FlowCommands::CHECKBALL) ||
      (msg.flowid == ball_picker::FlowCommands::SEARCHHAND) ||
      (msg.flowid == ball_picker::FlowCommands::CHECKHAND))
  {
    constflowid = msg.flowid;

    std_msgs::Float64 kinect_msg;

    switch (constflowid)
    {
      case ball_picker::FlowCommands::SEARCHBALL:
	space = SPACE;
	height = 0.0;
	falseangle = FALSEANGLE;
	pitch = 0.0;
	transformframe = "/map";
        kinect_msg.data = -0.6;
        break;
      case ball_picker::FlowCommands::CHECKBALL:
	space = 0.0;
	height = HEIGHT;
	falseangle = 0.0;
	pitch = PITCH;
	transformframe = "/odom";
	kinect_msg.data = -1.0;
        break;
      case ball_picker::FlowCommands::SEARCHHAND:
        space = SPACE;
	height = 0.0;
	falseangle = 0.0;
	pitch = 0.0;
	transformframe = "/map";
	kinect_msg.data = -0.4;
        break;
      case ball_picker::FlowCommands::CHECKHAND:
       	space = 0.0;
	height = HEIGHT;
	falseangle = 0.0;
	pitch = PITCH;
	transformframe = "/odom";
	kinect_msg.data = -0.6;
        break;
   }


    kinect_pub.publish(kinect_msg);
    ros::spinOnce();

    if (fabs(kinectposition - kinect_msg.data) > 0.00001)
      ros::Duration(1.0).sleep();
    
    kinectposition = kinect_msg.data;


    //reset vector with captured detections
    detected_targets.clear();
    detected_obstacles.clear();
  
    ROS_INFO_STREAM("Timers started.");
 
    //start timers 
    timer.start();
    limit.start();
  }

}

void Positioner::odometryCallback(const nav_msgs::Odometry& msg)
{
  origin.x = msg.pose.pose.position.x;
  origin.y = msg.pose.pose.position.y;
  origin.z = msg.pose.pose.position.z;

  odometry_recieved = true;
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
