# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "dylan_msc: 1 messages, 0 services")

set(MSG_I_FLAGS "-Idylan_msc:/home/dylan/catkin_ws/src/dylan_msc/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(dylan_msc_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/dylan/catkin_ws/src/dylan_msc/msg/obj.msg" NAME_WE)
add_custom_target(_dylan_msc_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dylan_msc" "/home/dylan/catkin_ws/src/dylan_msc/msg/obj.msg" "geometry_msgs/Point"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(dylan_msc
  "/home/dylan/catkin_ws/src/dylan_msc/msg/obj.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dylan_msc
)

### Generating Services

### Generating Module File
_generate_module_cpp(dylan_msc
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dylan_msc
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(dylan_msc_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(dylan_msc_generate_messages dylan_msc_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/dylan/catkin_ws/src/dylan_msc/msg/obj.msg" NAME_WE)
add_dependencies(dylan_msc_generate_messages_cpp _dylan_msc_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dylan_msc_gencpp)
add_dependencies(dylan_msc_gencpp dylan_msc_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dylan_msc_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(dylan_msc
  "/home/dylan/catkin_ws/src/dylan_msc/msg/obj.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dylan_msc
)

### Generating Services

### Generating Module File
_generate_module_eus(dylan_msc
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dylan_msc
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(dylan_msc_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(dylan_msc_generate_messages dylan_msc_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/dylan/catkin_ws/src/dylan_msc/msg/obj.msg" NAME_WE)
add_dependencies(dylan_msc_generate_messages_eus _dylan_msc_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dylan_msc_geneus)
add_dependencies(dylan_msc_geneus dylan_msc_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dylan_msc_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(dylan_msc
  "/home/dylan/catkin_ws/src/dylan_msc/msg/obj.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dylan_msc
)

### Generating Services

### Generating Module File
_generate_module_lisp(dylan_msc
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dylan_msc
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(dylan_msc_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(dylan_msc_generate_messages dylan_msc_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/dylan/catkin_ws/src/dylan_msc/msg/obj.msg" NAME_WE)
add_dependencies(dylan_msc_generate_messages_lisp _dylan_msc_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dylan_msc_genlisp)
add_dependencies(dylan_msc_genlisp dylan_msc_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dylan_msc_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(dylan_msc
  "/home/dylan/catkin_ws/src/dylan_msc/msg/obj.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dylan_msc
)

### Generating Services

### Generating Module File
_generate_module_nodejs(dylan_msc
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dylan_msc
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(dylan_msc_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(dylan_msc_generate_messages dylan_msc_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/dylan/catkin_ws/src/dylan_msc/msg/obj.msg" NAME_WE)
add_dependencies(dylan_msc_generate_messages_nodejs _dylan_msc_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dylan_msc_gennodejs)
add_dependencies(dylan_msc_gennodejs dylan_msc_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dylan_msc_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(dylan_msc
  "/home/dylan/catkin_ws/src/dylan_msc/msg/obj.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dylan_msc
)

### Generating Services

### Generating Module File
_generate_module_py(dylan_msc
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dylan_msc
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(dylan_msc_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(dylan_msc_generate_messages dylan_msc_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/dylan/catkin_ws/src/dylan_msc/msg/obj.msg" NAME_WE)
add_dependencies(dylan_msc_generate_messages_py _dylan_msc_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dylan_msc_genpy)
add_dependencies(dylan_msc_genpy dylan_msc_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dylan_msc_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dylan_msc)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dylan_msc
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(dylan_msc_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(dylan_msc_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dylan_msc)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dylan_msc
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(dylan_msc_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(dylan_msc_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dylan_msc)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dylan_msc
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(dylan_msc_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(dylan_msc_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dylan_msc)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dylan_msc
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(dylan_msc_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(dylan_msc_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dylan_msc)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dylan_msc\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dylan_msc
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(dylan_msc_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(dylan_msc_generate_messages_py geometry_msgs_generate_messages_py)
endif()
