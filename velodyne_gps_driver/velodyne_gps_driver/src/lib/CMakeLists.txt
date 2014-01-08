add_library(velodyne_gps_input input.cc)
target_link_libraries(velodyne_gps_input
  ${catkin_LIBRARIES}
  ${libpcap_LIBRARIES}
)
if(catkin_EXPORTED_TARGETS)
  add_dependencies(velodyne_gps_input ${catkin_EXPORTED_TARGETS})
endif()

install(TARGETS velodyne_gps_input
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
