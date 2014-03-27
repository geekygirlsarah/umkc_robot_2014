# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "mega_caretaker: 2 messages, 0 services")

set(MSG_I_FLAGS "-Imega_caretaker:/home/umkc/wesley/umkc_robot_2014_arduino/wesley/ros/src/mega_caretaker/msg;-Istd_msgs:/opt/ros/hydro/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(mega_caretaker_generate_messages ALL)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(mega_caretaker
  "/home/umkc/wesley/umkc_robot_2014_arduino/wesley/ros/src/mega_caretaker/msg/MegaPacket.msg"
  "${MSG_I_FLAGS}"
  "/home/umkc/wesley/umkc_robot_2014_arduino/wesley/ros/src/mega_caretaker/msg/MotorCommand.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mega_caretaker
)
_generate_msg_cpp(mega_caretaker
  "/home/umkc/wesley/umkc_robot_2014_arduino/wesley/ros/src/mega_caretaker/msg/MotorCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mega_caretaker
)

### Generating Services

### Generating Module File
_generate_module_cpp(mega_caretaker
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mega_caretaker
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(mega_caretaker_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(mega_caretaker_generate_messages mega_caretaker_generate_messages_cpp)

# target for backward compatibility
add_custom_target(mega_caretaker_gencpp)
add_dependencies(mega_caretaker_gencpp mega_caretaker_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mega_caretaker_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(mega_caretaker
  "/home/umkc/wesley/umkc_robot_2014_arduino/wesley/ros/src/mega_caretaker/msg/MegaPacket.msg"
  "${MSG_I_FLAGS}"
  "/home/umkc/wesley/umkc_robot_2014_arduino/wesley/ros/src/mega_caretaker/msg/MotorCommand.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mega_caretaker
)
_generate_msg_lisp(mega_caretaker
  "/home/umkc/wesley/umkc_robot_2014_arduino/wesley/ros/src/mega_caretaker/msg/MotorCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mega_caretaker
)

### Generating Services

### Generating Module File
_generate_module_lisp(mega_caretaker
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mega_caretaker
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(mega_caretaker_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(mega_caretaker_generate_messages mega_caretaker_generate_messages_lisp)

# target for backward compatibility
add_custom_target(mega_caretaker_genlisp)
add_dependencies(mega_caretaker_genlisp mega_caretaker_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mega_caretaker_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(mega_caretaker
  "/home/umkc/wesley/umkc_robot_2014_arduino/wesley/ros/src/mega_caretaker/msg/MegaPacket.msg"
  "${MSG_I_FLAGS}"
  "/home/umkc/wesley/umkc_robot_2014_arduino/wesley/ros/src/mega_caretaker/msg/MotorCommand.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mega_caretaker
)
_generate_msg_py(mega_caretaker
  "/home/umkc/wesley/umkc_robot_2014_arduino/wesley/ros/src/mega_caretaker/msg/MotorCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mega_caretaker
)

### Generating Services

### Generating Module File
_generate_module_py(mega_caretaker
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mega_caretaker
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(mega_caretaker_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(mega_caretaker_generate_messages mega_caretaker_generate_messages_py)

# target for backward compatibility
add_custom_target(mega_caretaker_genpy)
add_dependencies(mega_caretaker_genpy mega_caretaker_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mega_caretaker_generate_messages_py)


debug_message(2 "mega_caretaker: Iflags=${MSG_I_FLAGS}")


if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mega_caretaker)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mega_caretaker
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(mega_caretaker_generate_messages_cpp std_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mega_caretaker)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mega_caretaker
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(mega_caretaker_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mega_caretaker)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mega_caretaker\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mega_caretaker
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(mega_caretaker_generate_messages_py std_msgs_generate_messages_py)
