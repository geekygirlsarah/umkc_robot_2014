# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "imu_filter_madgwick: 0 messages, 1 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/hydro/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(imu_filter_madgwick_generate_messages ALL)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(imu_filter_madgwick
  "/home/umkc/umkc_robot_2014_arduino/wesley/ros/src/imu_tools/imu_filter_madgwick/srv/imu_yaw.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/imu_filter_madgwick
)

### Generating Module File
_generate_module_cpp(imu_filter_madgwick
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/imu_filter_madgwick
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(imu_filter_madgwick_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(imu_filter_madgwick_generate_messages imu_filter_madgwick_generate_messages_cpp)

# target for backward compatibility
add_custom_target(imu_filter_madgwick_gencpp)
add_dependencies(imu_filter_madgwick_gencpp imu_filter_madgwick_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS imu_filter_madgwick_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(imu_filter_madgwick
  "/home/umkc/umkc_robot_2014_arduino/wesley/ros/src/imu_tools/imu_filter_madgwick/srv/imu_yaw.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/imu_filter_madgwick
)

### Generating Module File
_generate_module_lisp(imu_filter_madgwick
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/imu_filter_madgwick
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(imu_filter_madgwick_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(imu_filter_madgwick_generate_messages imu_filter_madgwick_generate_messages_lisp)

# target for backward compatibility
add_custom_target(imu_filter_madgwick_genlisp)
add_dependencies(imu_filter_madgwick_genlisp imu_filter_madgwick_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS imu_filter_madgwick_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(imu_filter_madgwick
  "/home/umkc/umkc_robot_2014_arduino/wesley/ros/src/imu_tools/imu_filter_madgwick/srv/imu_yaw.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/imu_filter_madgwick
)

### Generating Module File
_generate_module_py(imu_filter_madgwick
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/imu_filter_madgwick
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(imu_filter_madgwick_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(imu_filter_madgwick_generate_messages imu_filter_madgwick_generate_messages_py)

# target for backward compatibility
add_custom_target(imu_filter_madgwick_genpy)
add_dependencies(imu_filter_madgwick_genpy imu_filter_madgwick_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS imu_filter_madgwick_generate_messages_py)


debug_message(2 "imu_filter_madgwick: Iflags=${MSG_I_FLAGS}")


if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/imu_filter_madgwick)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/imu_filter_madgwick
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(imu_filter_madgwick_generate_messages_cpp std_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/imu_filter_madgwick)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/imu_filter_madgwick
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(imu_filter_madgwick_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/imu_filter_madgwick)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/imu_filter_madgwick\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/imu_filter_madgwick
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(imu_filter_madgwick_generate_messages_py std_msgs_generate_messages_py)
