cmake_minimum_required(VERSION 2.8.3)
project(imu_filter_madgwick)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs geometry_msgs tf nodelet message_filters dynamic_reconfigure message_generation std_msgs)

find_package(Boost REQUIRED COMPONENTS system thread signals)




#create srv files
add_service_files(
	FILES
	imu_yaw.srv
)

# Generate dynamic parameters
generate_dynamic_reconfigure_options(cfg/ImuFilterMadgwick.cfg)

#trying to generate my srv file
generate_messages(
	DEPENDENCIES
	std_msgs
)


catkin_package(
  DEPENDS 
  CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs tf nodelet message_filters dynamic_reconfigure message_generation std_msgs message_runtime
  INCLUDE_DIRS
  LIBRARIES imu_filter imu_filter_nodelet
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

# create imu_filter library
add_library (imu_filter src/imu_filter.cpp)
add_dependencies(imu_filter imu_filter_madgwick_generate_messages_cpp ${PROJECT_NAME}_gencfg  ${catkin_EXPORTED_TARGETS} )
target_link_libraries(imu_filter ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# create imu_filter_nodelet library
add_library (imu_filter_nodelet src/imu_filter_nodelet.cpp)
add_dependencies(imu_filter_nodelet imu_filter_gencpp ${PROJECT_NAME}_gencfg imu_filter_generate_messages_cpp)
target_link_libraries(imu_filter_nodelet imu_filter ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# create imu_filter_node executable
add_executable(imu_filter_node src/imu_filter_node.cpp)
add_dependencies(imu_filter_node imu_filter_gencpp ${PROJECT_NAME}_gencfg imu_filter_generate_messages_cpp)
target_link_libraries(imu_filter_node imu_filter ${catkin_LIBRARIES} ${Boost_LIBRARIES})


install(TARGETS imu_filter imu_filter_nodelet imu_filter_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES imu_filter_nodelet.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
