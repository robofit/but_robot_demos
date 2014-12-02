##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(recognizer)

##############################################################################
# Catkin
##############################################################################

find_package(catkin REQUIRED rosjava_build_tools)

# Set the gradle targets you want catkin's make to run by default, e.g.
#   catkin_rosjava_setup(installApp)
# Note that the catkin_create_rosjava_xxx scripts will usually automatically
# add tasks to this for you when you create subprojects.
catkin_rosjava_setup(installApp) #publishMavenJavaPublicationToMavenRepository)

catkin_package()

message ("current  " ${CMAKE_CURRENT_LIST_DIR})

add_custom_command (TARGET "gradle-recognizer"
		POST_BUILD
		COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_LIST_DIR}/${PROJECT_NAME}/build/install/${PROJECT_NAME}/lib
				${CATKIN_DEVEL_PREFIX}/share/${PROJECT_NAME}/lib
)

file (COPY ${CMAKE_HOME_DIRECTORY}/../data/${PROJECT_NAME}  
	DESTINATION ${CATKIN_DEVEL_PREFIX}/share/${PROJECT_NAME})
#	PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)

file (COPY ${CMAKE_HOME_DIRECTORY}/../data/data
	DESTINATION ${CATKIN_DEVEL_PREFIX}/share/${PROJECT_NAME})

##############################################################################
# Installation
##############################################################################

# Change this to match the maven group name you have specified in the 
# allprojects closure the root build.gradle

install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/share/${PROJECT_NAME}/lib
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(FILES ${CMAKE_HOME_DIRECTORY}/../data/${PROJECT_NAME} 
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/
	PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)

install(DIRECTORY
  ${CMAKE_HOME_DIRECTORY}/../data/data
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES
  ${CMAKE_HOME_DIRECTORY}/../data/launch/recognizer.launch
  ${CMAKE_HOME_DIRECTORY}/../data/launch/recognizer_vel_cmd.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)




