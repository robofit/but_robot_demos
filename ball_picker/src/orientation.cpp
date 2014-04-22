/**
 * Author: Dagmar Prokopova
 * File: orientation.cpp
 * Description: Gives command for moving
 * Bachelor's thesis, 2013/2014
 *
 */

#include "ball_picker/orientation.h"

using namespace ball_picker;
using namespace std;



/**
 * constructor of mover
 * establishes subscription and initializes action client
 */
Orientation::Orientation(ros::NodeHandle& node): nh(node) {

  odom_sub = nh.subscribe("/odom", 1, &Orientation::odomCallback, this);

  ROS_INFO_STREAM("Orientation initialized.");
}


/**
 * destructor of mover
 */
Orientation::~Orientation() {}


/**
 * callback function called after new message appears in the topic with goal coordinates
 */
void Orientation::odomCallback(const nav_msgs::Odometry& msg)
{
  ROS_INFO_ONCE("Odometry received.");
 
  geometry_msgs::Point origin;
  geometry_msgs::Pose msg_goal;
 
  msg_goal.position.x = -1;
  msg_goal.position.y = 0.3;
  msg_goal.position.z = 0.03;

  origin.x = msg.pose.pose.position.x;
  origin.y = msg.pose.pose.position.y;

    int modif_x = 1;
    int modif_y = -1;
    double base_angle = 0;
    double angle = 0;

    if (msg_goal.position.x >= origin.x)
    {
      //IV. quadrant
      if (msg_goal.position.y >= origin.y)
      {
        modif_x = 1;
        modif_y = 1;
        base_angle = 0;
      }
      //I. quadrant
      else
      {
        modif_x = 1;
        modif_y = -1;
        base_angle = 0;
      }
    }
    else
    {
      //IV quadrant
      if (msg_goal.position.y <= origin.y)
      {
        modif_x = -1;
        modif_y = -1;
        base_angle = PI;
      }
      //III quadrant
      else
      {
        modif_x = -1;
        modif_y = 1;
        base_angle = PI;
      }
    }


    geometry_msgs::Point diff = getDistance(abs((modif_x)*msg_goal.position.x-(modif_x)*origin.x), abs((modif_y)*msg_goal.position.y-(modif_y)*origin.y));
    msg_goal.position.x = origin.x + ((modif_x)*diff.x);
    msg_goal.position.y = origin.y + ((modif_y)*diff.y);

    angle = atan(abs((modif_y)*msg_goal.position.y-(modif_y)*origin.y)/abs((modif_x)*msg_goal.position.x-(modif_x)*origin.x));

    angle = (-1)*(modif_x)*(modif_y)*(base_angle - angle);

     tf::Quaternion q;
     q.setRPY(0.0, 0.0, angle);
     tf::quaternionTFToMsg(q, msg_goal.orientation);



  cout << "x: " << msg_goal.position.x << endl;
  cout << "y: " << msg_goal.position.y << endl;
  cout << "z: " << msg_goal.position.z << endl;


  cout << "angle: " << angle << endl;;
  cout << "x: " << msg_goal.orientation.x << endl;
  cout << "y: " << msg_goal.orientation.y << endl;
  cout << "z: " << msg_goal.orientation.z << endl;
  cout << "w: " << msg_goal.orientation.w << endl;

  cout << "-----------------------" << endl;
}


geometry_msgs::Point Orientation::getDistance(double a, double b)
{
  geometry_msgs::Point result;

  double distance = sqrt(a*a+b*b) - 0.5;

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



int main (int argc, char** argv)
{
  ROS_INFO_STREAM("Orientation started.");

  //ROS initialization
  ros::init(argc, argv, "orientation");

  //create node
  ros::NodeHandle n;

  //create mover object
  Orientation o(n);

  //keep the node alive
  ros::spin();

  return 0;
}
