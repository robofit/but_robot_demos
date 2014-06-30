#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "SpeedControl.h"
using namespace roadcheck;
int main(int argc, char **argv) {
    printf(" ### Robot road check run. ###\n");
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;
    SpeedControl sc(n);
    ros::Subscriber sub = n.subscribe("/imu_3dm_node/imu/data", 1000, &SpeedControl::callback, &sc);
    ros::Subscriber sub2 = n.subscribe("/cmd_vel_orig", 1000, &SpeedControl::callbackManual, &sc);
    ros::spin();
    return 0;
}
