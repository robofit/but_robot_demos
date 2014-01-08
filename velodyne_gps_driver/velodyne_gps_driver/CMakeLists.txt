cmake_minimum_required(VERSION 2.8.3)
project(velodyne_gps_driver)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS  roscpp rospy std_msgs nodelet roscpp tf message_generation)
  
  # This driver uses Boost threads
find_package(Boost REQUIRED COMPONENTS thread)

# libpcap provides no pkg-config or find_package module:
set(libpcap_LIBRARIES -lpcap)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})


add_message_files(
  DIRECTORY msg
  FILES
  VelodyneGPSPacket.msg
  VelodyneGPSScan.msg
)
generate_messages(DEPENDENCIES std_msgs)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS message_runtime roscpp rospy std_msgs nodelet
INCLUDE_DIRS include
LIBRARIES velodyne_gps_input
)

###########
## Build ##
###########

add_subdirectory(src/lib)
add_subdirectory(src/driver)


install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(FILES nodelet_velodyne.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(PROGRAMS src/vdump
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Declare a cpp library
# add_library(velodyne_gps_driver
#   src/${PROJECT_NAME}/velodyne_gps_driver.cpp
# )

## Declare a cpp executable
# add_executable(velodyne_gps_driver_node src/velodyne_gps_driver_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(velodyne_gps_driver_node velodyne_gps_driver_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(velodyne_gps_driver_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS velodyne_gps_driver velodyne_gps_driver_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_velodyne_gps_driver.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
