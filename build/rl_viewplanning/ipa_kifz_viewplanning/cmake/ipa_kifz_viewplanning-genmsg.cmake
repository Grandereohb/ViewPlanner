# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "ipa_kifz_viewplanning: 0 messages, 1 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/noetic/share/sensor_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(ipa_kifz_viewplanning_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/ros/abb_ws/src/rl_viewplanning/ipa_kifz_viewplanning/srv/GetAreaGain.srv" NAME_WE)
add_custom_target(_ipa_kifz_viewplanning_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ipa_kifz_viewplanning" "/home/ros/abb_ws/src/rl_viewplanning/ipa_kifz_viewplanning/srv/GetAreaGain.srv" "sensor_msgs/PointCloud2:std_msgs/Header:sensor_msgs/PointField"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(ipa_kifz_viewplanning
  "/home/ros/abb_ws/src/rl_viewplanning/ipa_kifz_viewplanning/srv/GetAreaGain.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointField.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ipa_kifz_viewplanning
)

### Generating Module File
_generate_module_cpp(ipa_kifz_viewplanning
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ipa_kifz_viewplanning
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(ipa_kifz_viewplanning_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(ipa_kifz_viewplanning_generate_messages ipa_kifz_viewplanning_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ros/abb_ws/src/rl_viewplanning/ipa_kifz_viewplanning/srv/GetAreaGain.srv" NAME_WE)
add_dependencies(ipa_kifz_viewplanning_generate_messages_cpp _ipa_kifz_viewplanning_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ipa_kifz_viewplanning_gencpp)
add_dependencies(ipa_kifz_viewplanning_gencpp ipa_kifz_viewplanning_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ipa_kifz_viewplanning_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(ipa_kifz_viewplanning
  "/home/ros/abb_ws/src/rl_viewplanning/ipa_kifz_viewplanning/srv/GetAreaGain.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointField.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ipa_kifz_viewplanning
)

### Generating Module File
_generate_module_eus(ipa_kifz_viewplanning
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ipa_kifz_viewplanning
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(ipa_kifz_viewplanning_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(ipa_kifz_viewplanning_generate_messages ipa_kifz_viewplanning_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ros/abb_ws/src/rl_viewplanning/ipa_kifz_viewplanning/srv/GetAreaGain.srv" NAME_WE)
add_dependencies(ipa_kifz_viewplanning_generate_messages_eus _ipa_kifz_viewplanning_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ipa_kifz_viewplanning_geneus)
add_dependencies(ipa_kifz_viewplanning_geneus ipa_kifz_viewplanning_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ipa_kifz_viewplanning_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(ipa_kifz_viewplanning
  "/home/ros/abb_ws/src/rl_viewplanning/ipa_kifz_viewplanning/srv/GetAreaGain.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointField.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ipa_kifz_viewplanning
)

### Generating Module File
_generate_module_lisp(ipa_kifz_viewplanning
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ipa_kifz_viewplanning
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(ipa_kifz_viewplanning_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(ipa_kifz_viewplanning_generate_messages ipa_kifz_viewplanning_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ros/abb_ws/src/rl_viewplanning/ipa_kifz_viewplanning/srv/GetAreaGain.srv" NAME_WE)
add_dependencies(ipa_kifz_viewplanning_generate_messages_lisp _ipa_kifz_viewplanning_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ipa_kifz_viewplanning_genlisp)
add_dependencies(ipa_kifz_viewplanning_genlisp ipa_kifz_viewplanning_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ipa_kifz_viewplanning_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(ipa_kifz_viewplanning
  "/home/ros/abb_ws/src/rl_viewplanning/ipa_kifz_viewplanning/srv/GetAreaGain.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointField.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ipa_kifz_viewplanning
)

### Generating Module File
_generate_module_nodejs(ipa_kifz_viewplanning
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ipa_kifz_viewplanning
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(ipa_kifz_viewplanning_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(ipa_kifz_viewplanning_generate_messages ipa_kifz_viewplanning_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ros/abb_ws/src/rl_viewplanning/ipa_kifz_viewplanning/srv/GetAreaGain.srv" NAME_WE)
add_dependencies(ipa_kifz_viewplanning_generate_messages_nodejs _ipa_kifz_viewplanning_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ipa_kifz_viewplanning_gennodejs)
add_dependencies(ipa_kifz_viewplanning_gennodejs ipa_kifz_viewplanning_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ipa_kifz_viewplanning_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(ipa_kifz_viewplanning
  "/home/ros/abb_ws/src/rl_viewplanning/ipa_kifz_viewplanning/srv/GetAreaGain.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointField.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ipa_kifz_viewplanning
)

### Generating Module File
_generate_module_py(ipa_kifz_viewplanning
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ipa_kifz_viewplanning
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(ipa_kifz_viewplanning_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(ipa_kifz_viewplanning_generate_messages ipa_kifz_viewplanning_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ros/abb_ws/src/rl_viewplanning/ipa_kifz_viewplanning/srv/GetAreaGain.srv" NAME_WE)
add_dependencies(ipa_kifz_viewplanning_generate_messages_py _ipa_kifz_viewplanning_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ipa_kifz_viewplanning_genpy)
add_dependencies(ipa_kifz_viewplanning_genpy ipa_kifz_viewplanning_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ipa_kifz_viewplanning_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ipa_kifz_viewplanning)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ipa_kifz_viewplanning
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(ipa_kifz_viewplanning_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(ipa_kifz_viewplanning_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ipa_kifz_viewplanning)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ipa_kifz_viewplanning
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(ipa_kifz_viewplanning_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(ipa_kifz_viewplanning_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ipa_kifz_viewplanning)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ipa_kifz_viewplanning
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(ipa_kifz_viewplanning_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(ipa_kifz_viewplanning_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ipa_kifz_viewplanning)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ipa_kifz_viewplanning
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(ipa_kifz_viewplanning_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(ipa_kifz_viewplanning_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ipa_kifz_viewplanning)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ipa_kifz_viewplanning\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ipa_kifz_viewplanning
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(ipa_kifz_viewplanning_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(ipa_kifz_viewplanning_generate_messages_py sensor_msgs_generate_messages_py)
endif()
