# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "wesley: 2 messages, 0 services")

set(MSG_I_FLAGS "-Iwesley:/home/umkc/umkc_robot_2014_arduino/wesley/ros/src/wesley/msg;-Istd_msgs:/opt/ros/hydro/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(wesley_generate_messages ALL)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(wesley
  "/home/umkc/umkc_robot_2014_arduino/wesley/ros/src/wesley/msg/arm_point.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/wesley
)
_generate_msg_cpp(wesley
  "/home/umkc/umkc_robot_2014_arduino/wesley/ros/src/wesley/msg/arm_angle.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/wesley
)

### Generating Services

### Generating Module File
_generate_module_cpp(wesley
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/wesley
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(wesley_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(wesley_generate_messages wesley_generate_messages_cpp)

# target for backward compatibility
add_custom_target(wesley_gencpp)
add_dependencies(wesley_gencpp wesley_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS wesley_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(wesley
  "/home/umkc/umkc_robot_2014_arduino/wesley/ros/src/wesley/msg/arm_point.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/wesley
)
_generate_msg_lisp(wesley
  "/home/umkc/umkc_robot_2014_arduino/wesley/ros/src/wesley/msg/arm_angle.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/wesley
)

### Generating Services

### Generating Module File
_generate_module_lisp(wesley
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/wesley
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(wesley_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(wesley_generate_messages wesley_generate_messages_lisp)

# target for backward compatibility
add_custom_target(wesley_genlisp)
add_dependencies(wesley_genlisp wesley_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS wesley_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(wesley
  "/home/umkc/umkc_robot_2014_arduino/wesley/ros/src/wesley/msg/arm_point.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/wesley
)
_generate_msg_py(wesley
  "/home/umkc/umkc_robot_2014_arduino/wesley/ros/src/wesley/msg/arm_angle.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/wesley
)

### Generating Services

### Generating Module File
_generate_module_py(wesley
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/wesley
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(wesley_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(wesley_generate_messages wesley_generate_messages_py)

# target for backward compatibility
add_custom_target(wesley_genpy)
add_dependencies(wesley_genpy wesley_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS wesley_generate_messages_py)


debug_message(2 "wesley: Iflags=${MSG_I_FLAGS}")


if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/wesley)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/wesley
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(wesley_generate_messages_cpp std_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/wesley)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/wesley
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(wesley_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/wesley)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/wesley\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/wesley
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(wesley_generate_messages_py std_msgs_generate_messages_py)
