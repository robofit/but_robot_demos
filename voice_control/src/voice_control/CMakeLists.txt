cmake_minimum_required(VERSION 2.8.3)
project(voice_control)

find_package(catkin REQUIRED)

catkin_package(
)

install(FILES
  ${CMAKE_HOME_DIRECTORY}/../data/launch/voice_control.launch
  ${CMAKE_HOME_DIRECTORY}/../data/launch/voice_control_vel_cmd.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
