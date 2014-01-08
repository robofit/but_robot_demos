cmake_minimum_required(VERSION 2.8.3)
project(velodyne_gps)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs  velodyne_gps_driver sensor_msgs)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

find_package(Boost REQUIRED COMPONENTS thread)


#######################################
## Declare ROS messages and services ##
#######################################



###################################
## catkin specific configuration ##
###################################

catkin_package(
INCLUDE_DIRS include
CATKIN_DEPENDS roscpp rospy std_msgs velodyne_gps_driver sensor_msgs
)

###########
## Build ##
###########


include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_library(velodyne_gps src/nmea_convertor.cc  )
add_executable(velodyne_gps_node src/velodyne_gps_node.cc)
target_link_libraries(velodyne_gps_node velodyne_gps   ${catkin_LIBRARIES})


