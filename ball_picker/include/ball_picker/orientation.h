/**
 * Author: Dagmar Prokopova
 * File: orientation.h
 * Description: Header file for mover
 * Bachelor's thesis, 2013/2014
 *
 */


#ifndef ORIENTATION_H_
#define ORIENTATION_H_

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

#define PI 3.14159268

namespace ball_picker {

  class Orientation 
  {
    public:
      Orientation(ros::NodeHandle& nh);
      ~Orientation();

    protected:

      ros::NodeHandle nh;
      ros::Subscriber odom_sub;

      void odomCallback(const nav_msgs::Odometry& msg);
      geometry_msgs::Point getDistance(double a, double b);


   };

}


#endif
