# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "roscpp;std_msgs;sensor_msgs;control_msgs;trajectory_msgs;simple_message;actionlib_msgs;actionlib;urdf;industrial_msgs;industrial_utils".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lindustrial_robot_client_dummy".split(';') if "-lindustrial_robot_client_dummy" != "" else []
PROJECT_NAME = "industrial_robot_client"
PROJECT_SPACE_DIR = "/home/ros/abb_ws/install"
PROJECT_VERSION = "0.7.1"
