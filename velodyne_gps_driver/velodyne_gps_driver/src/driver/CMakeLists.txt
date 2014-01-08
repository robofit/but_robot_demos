# build the driver node
add_executable(velodyne_gps_driver_node velodyne_gps_driver_node.cc driver.cc)
target_link_libraries(velodyne_gps_driver_node
  velodyne_gps_input
  ${catkin_LIBRARIES}
  ${libpcap_LIBRARIES}
)


# install runtime files
install(TARGETS velodyne_gps_driver_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        COMPONENT main
)
