/**
 * Author: Dagmar Prokopova
 * File: positioner.cpp
 * Description: Trasformation of detected balls and hands into desired coordinate frame
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
  constflowid = 0;

  //setup clients for detections
  ball_detection_client = nh.serviceClient<DetectObjects>("/ball_detections");
  hand_detection_client = nh.serviceClient<DetectObjects>("/hand_detections");

  //setup control client
  control_client = nh.serviceClient<FlowControl>("/flow_control");

  //set timers (stopped)
  timer = nh.createWallTimer(ros::WallDuration(0.1), &Positioner::timerCallback, this, false, false);
  limit = nh.createWallTimer(ros::WallDuration(3.0), &Positioner::limitCallback, this, false, false);

  //subscribe for depth, camera info and flow commands
  depth_sub = it.subscribe("depth_rect", 1, &Positioner::depthImageCallback, this);
  cam_info_sub = nh.subscribe("cam_info", 1, &Positioner::camInfoCallback, this);
  control_sub = nh.subscribe("flow_commands", 1, &Positioner::controlCallback, this);
 
  //we have 2 costmaps to be cleared (local and global) 
  costmap_updated = 2;
  //this will hold everything until costmaps are updated
  costmap_updated_srv = nh.advertiseService("confirm_updated_costmap", &Positioner::updatedCostmapCallback, this); 

  //prepare publishing of angle, goal, costmap_obstacles and kinect direction
  angle_pub = nh.advertise<std_msgs::Float64>("goal_angle", 1);
  goal_pub = nh.advertise<geometry_msgs::Pose>("goal_coords", 1);
  costmap_pub = nh.advertise<ball_picker::Detections>("costmap_custom_obstacles", 1);
  kinect_pub = nh.advertise<std_msgs::Float64>("kinect_servo",1);

  depth_img.reset();

  space = SPACE;
  falseangle = 0.0;
  transformframe = "/base_footprint";
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

  //check if control client is still with us
  if (!control_client.waitForExistence(ros::Duration(0.5)))
  {
    ROS_WARN_THROTTLE(1.0, "Positioner: Control service not available.");
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
  std_msgs::Float64 angle_msg;
  ball_picker::Detections msg_costmap;
  msg_costmap.type = ball_picker::Detections::BALL;
  ball_picker::PointOfInterest obstacle;
  vector<DetCoords> *detected_targets; 

  //if we want to add something into costmap
  if (((constflowid == ball_picker::FlowCommands::SEARCHBALL) || (constflowid == ball_picker::FlowCommands::SEARCHHAND)) && (!detected_balls.empty()))
  {
    for (std::vector<DetCoords>::iterator iter = detected_balls.begin() ; iter != detected_balls.end(); iter++)
    {
      //transformation from camera coordinates to map coordinates
      geometry_msgs::PoseStamped ps;
      ps.header.frame_id = cam_model.tfFrame();
      ps.header.stamp = ros::Time(0);

      ps.pose.position.x = iter->point.x;
      ps.pose.position.y = iter->point.y;
      ps.pose.position.z = iter->point.z;

      ps.pose.orientation.x = 0.0;
      ps.pose.orientation.y = 0.0;
      ps.pose.orientation.z = 0.0;
      ps.pose.orientation.w = 1.0;

      //transform into map
      if (!tfl.waitForTransform("/map", ps.header.frame_id, ps.header.stamp, ros::Duration(1.0)))
      {
        ROS_WARN_THROTTLE(1.0,"Positioner: Transform from camera to map not available!");
        detected_balls.clear();
	detected_hands.clear();
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

      //we need at least 2 detections to make it a ball and an obstacle 
      if (iter->count > 1)
      {
        obstacle.x = ps.pose.position.x;
        obstacle.y = ps.pose.position.y;
        msg_costmap.objectcenters.push_back(obstacle);
      }
    }
  }

  //if we have something to add into the costmap
  if (!msg_costmap.objectcenters.empty())
  {
    costmap_pub.publish(msg_costmap);
    ros::spinOnce();
   
    //wait for update of costmaps 
    while(costmap_updated != 0)
    {
      ROS_INFO_THROTTLE(1.0, "Positioner: Waiting for costmaps to be updated.");
      ros::spinOnce();
    }

    costmap_updated = 2;
  }


  if ((constflowid == ball_picker::FlowCommands::SEARCHBALL) || (constflowid == ball_picker::FlowCommands::CHECKBALL))
    detected_targets = &detected_balls;
  else
    detected_targets = &detected_hands;


  if (!detected_targets->empty())
  {
    point3d.z = FLT_MAX;

    for (std::vector<DetCoords>::iterator iter = detected_targets->begin() ; iter != detected_targets->end(); iter++)
    {

      //transformation from camera coordinates to base_footprint or arm_shoulder_pan_link
      geometry_msgs::PoseStamped ps;
      ps.header.frame_id = cam_model.tfFrame();
      ps.header.stamp = ros::Time(0);

      ps.pose.position.x = iter->point.x;
      ps.pose.position.y = iter->point.y;
      ps.pose.position.z = iter->point.z;

      ps.pose.orientation.x = 0.0;
      ps.pose.orientation.y = 0.0;
      ps.pose.orientation.z = 0.0;
      ps.pose.orientation.w = 1.0;

      if (!tfl.waitForTransform(transformframe, ps.header.frame_id, ps.header.stamp, ros::Duration(1.0)))
      {
        ROS_WARN_THROTTLE(1.0,"Transform from camera to transformframe not available!");
        detected_balls.clear();
	detected_hands.clear();
        timer.start();
        limit.start();
        return;
      }

      bool tfextrapolation = false;
      do
      {
        try
        {
          tfl.transformPose(transformframe, ps, ps);
          tfextrapolation = false;
        }
        catch (tf::ExtrapolationException& ex)
        {
          tfextrapolation = true;
        }
        catch (tf::TransformException& ex)
        {
          ROS_ERROR("TF exception:\n%s", ex.what());
          return;
        }
        
        if (tfextrapolation)
        {
          ROS_WARN_THROTTLE(1.0, "Waiting for transformation.");
          ros::Duration(0.1).sleep();
        }

      } while (tfextrapolation);

      //we need at least 5 detections of the object to bother to move to it
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
    ROS_INFO("Positioner: The nearest object found on the position x: %f, y: %f, z: %f. ", msg_goal.position.x, msg_goal.position.y, msg_goal.position.z);


    int modif_x = 1;
    int modif_y = 1;
    double base_angle = 0;
    double angle = 0;


    //set the modifiers

    if (msg_goal.position.x >= 0)
    {
      //I. quadrant
      if (msg_goal.position.y >= 0)
      {
        modif_x = 1;
        modif_y = 1;
      }
      //IV. quadrant
      else
      {
        modif_x = 1;
        modif_y = -1;
      }
      
      base_angle = 0;
    }
    else
    {
      //III. quadrant
      if (msg_goal.position.y <= 0)
      {
        modif_x = -1;
        modif_y = -1;
      }
      //II. quadrant
      else
      {
        modif_x = -1;
        modif_y = 1;
      }

      base_angle = PI;
    }

    //count the new coordinates with SPACE considered
    geometry_msgs::Point diff = getDistance(fabs(msg_goal.position.x), fabs(msg_goal.position.y));
  
    //almost zero 
    if (diff.x < 0.00001)
      angle = PI;
    else
      angle = atan(diff.y/diff.x);    

    //get back into the right quadrant
    msg_goal.position.x = (modif_x)*diff.x;
    msg_goal.position.y = (modif_y)*diff.y;

    //count the angle
    angle_msg.data = (modif_x)*(modif_y)*(angle - base_angle);
    //turn a little bit more not to have arm in front of the ball
    angle_msg.data = angle_msg.data - falseangle;

    //get the value into the interval from -PI/2 to PI/2
    if (angle_msg.data > PI)
      angle_msg.data = angle_msg.data - 2*PI;
    else if (angle_msg.data < -PI)
      angle_msg.data = angle_msg.data + 2*PI;

    //publish the angle
    angle_pub.publish(angle_msg);


    tf::Quaternion q;
    q.setRPY(0.0, 0.0, angle);
    tf::quaternionTFToMsg(q, msg_goal.orientation);

    //advertise the result on goal_coords topic
    goal_pub.publish(msg_goal);
    ros::spinOnce();
 
    //send positive state into control service
    srv.request.state = true;

    

  }
  else
  {
    ROS_INFO("Positioner: No object found.");
    //send negative state into control service
    srv.request.state = false;
  }

  //call the control service
  if (!control_client.call(srv))
  {
    ROS_ERROR("Positioner: Error on calling control service!");
    return;
  }

}



/**
 * periodical call of detection service
 */
void Positioner::timerCallback(const ros::WallTimerEvent& event)
{

  ROS_INFO_ONCE("Positioner: Timer triggered for the first time.");

  if (!ball_detection_client.waitForExistence(ros::Duration(0.05)))
  {
    ROS_WARN_THROTTLE(1.0," Positioner: Ball detection service not available.");
    return;
  }

  if (!hand_detection_client.waitForExistence(ros::Duration(0.05)))
  {
    ROS_WARN_THROTTLE(1.0," Positioner: Hand detection service not available.");
    return;
  }

  if (!cam_model.initialized())
  {
    ROS_WARN_THROTTLE(1.0, "Positioner: Camera model not initialized.");
    return;
  }

  vector<ball_picker::Detections> dets;
  dets.clear();



  DetectObjects srv;

  //fill the parameters for the service
  srv.request.all = true;


  //we need to detect balls
  if (constflowid != ball_picker::FlowCommands::CHECKHAND)
  {
    //call the service for detection
    if (!ball_detection_client.call(srv))
    {
      ROS_WARN_THROTTLE(1.0,"Positioner: Error on calling ball detection service!");
      return;
    }
    dets.push_back(srv.response.detections);
  }

  //we need to detect hands
  if ((constflowid == ball_picker::FlowCommands::SEARCHHAND) ||
      (constflowid == ball_picker::FlowCommands::CHECKHAND))
  {
     //call the service for detection
    if (!hand_detection_client.call(srv))
    {
      ROS_WARN_THROTTLE(1.0,"Positioner: Error on calling hand detection service!");
      return;
    }
    dets.push_back(srv.response.detections);
  }


  //nothing to do if depth image is not awailable
  if (depth_img == NULL)
  {
    ROS_WARN_THROTTLE(1.0,"Depth not available");
    return;
  }
 
  vector<DetCoords> *storage_ptr; 

  //process all the detections  
  for (std::vector<ball_picker::Detections>::iterator detsit = dets.begin(); detsit != dets.end(); detsit++)
  {

    if(detsit->type == ball_picker::Detections::BALL)
      storage_ptr = &detected_balls;
    else
      storage_ptr = &detected_hands;


    //process all the detections: add new one, modify the similar
    for(unsigned int i=0; i < detsit->objectcenters.size(); i++)
    {
      DetCoords dcoords;

      if (cameraTo3D(detsit->objectcenters[i].x, detsit->objectcenters[i].y, &(dcoords.point)))
      {
        bool modified = false;

        for (std::vector<DetCoords>::iterator iter = storage_ptr->begin() ; iter != storage_ptr->end(); iter++)
        {
          //check if it might be the same object
          if ( (fabs(iter->point.x - dcoords.point.x) < 0.01) &&
               (fabs(iter->point.y - dcoords.point.y) < 0.01) )
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
    ROS_WARN_THROTTLE(1.0,"Positioner: Unable to get depth.");
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
    ROS_WARN_THROTTLE(1.0, "Positioner: Can't initialize camera model.");
  }
}


/**
 * callback on incoming depth msg
 */
void Positioner::depthImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  ROS_INFO_ONCE("Positioner: Depth image received.");

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
	falseangle = FALSEANGLE;
	transformframe = "/base_footprint";
        kinect_msg.data = -0.6;
        break;
      case ball_picker::FlowCommands::CHECKBALL:
	space = 0.0;
	falseangle = 0.0;
	transformframe = "/arm_shoulder_pan_link";
	kinect_msg.data = -1.0;
        break;
      case ball_picker::FlowCommands::SEARCHHAND:
        space = SPACE;
	falseangle = 0.0;
	transformframe = "/base_footprint";
	kinect_msg.data = -0.4;
        break;
      case ball_picker::FlowCommands::CHECKHAND:
       	space = 0.0;
	falseangle = 0.0;
	transformframe = "/arm_shoulder_pan_link";
	kinect_msg.data = -0.6;
        break;
     }
   


    kinect_pub.publish(kinect_msg);
    ros::spinOnce();

    //if we have kinect somewhere else, wait a while
    if (fabs(kinectposition - kinect_msg.data) > 0.00001)
      ros::Duration(1.0).sleep();
    
    kinectposition = kinect_msg.data;


    //reset vector with captured detections
    detected_balls.clear();
    detected_hands.clear();
  
    ROS_INFO_STREAM("Positioner: Timers started.");
 
    //start timers 
    timer.start();
    limit.start();
  }

}

/*
 * Informes that one of the costmaps is updated.
 */
bool Positioner::updatedCostmapCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  ROS_INFO("Costmap updated with balls.");
  costmap_updated--;
  return true;
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
