execute_process(COMMAND "/home/umkc/umkc_robot_2014_arduino/wesley/ros/build/rosserial/rosserial_client/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/umkc/umkc_robot_2014_arduino/wesley/ros/build/rosserial/rosserial_client/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
