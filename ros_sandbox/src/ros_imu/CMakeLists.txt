cmake_minimum_required(VERSION 2.8.3)
project(ros_imu)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  std_msgs
  geometry_msgs
  message_generation
)

add_message_files(
	FILES
	orientation.msg
	rotMatrix.msg
	spatialRaw.msg
)

add_service_files(
	FILES
	imu_filter.srv
	GetOrientation.srv
)

generate_messages(
	DEPENDENCIES
	std_msgs
	geometry_msgs
)

catkin_package(
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(orientation_server src/calculate_orientationServer.cpp src/orientation_headers/imuFilter.cpp)
target_link_libraries(orientation_server ${catkin_LIBRARIES})
add_dependencies(orientation_server ros_imu_generate_messages_cpp)

add_executable(rawIMUPublisher src/publishIMU.cpp src/phidget_headers/spatial_helper.cpp)
target_link_libraries(rawIMUPublisher /usr/local/lib/libphidget21.so ${catkin_LIBRARIES})
add_dependencies(rawIMUPublisher ros_imu_generate_messages_cpp)
