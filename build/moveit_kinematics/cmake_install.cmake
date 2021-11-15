# Install script for directory: /home/ros/abb_ws/src/moveit_kinematics

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/ros/abb_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/ros/abb_ws/build/moveit_kinematics/catkin_generated/installspace/moveit_kinematics.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/moveit_kinematics/cmake" TYPE FILE FILES
    "/home/ros/abb_ws/build/moveit_kinematics/catkin_generated/installspace/moveit_kinematicsConfig.cmake"
    "/home/ros/abb_ws/build/moveit_kinematics/catkin_generated/installspace/moveit_kinematicsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/moveit_kinematics" TYPE FILE FILES "/home/ros/abb_ws/src/moveit_kinematics/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/moveit_kinematics" TYPE FILE FILES
    "/home/ros/abb_ws/src/moveit_kinematics/kdl_kinematics_plugin_description.xml"
    "/home/ros/abb_ws/src/moveit_kinematics/lma_kinematics_plugin_description.xml"
    "/home/ros/abb_ws/src/moveit_kinematics/srv_kinematics_plugin_description.xml"
    "/home/ros/abb_ws/src/moveit_kinematics/cached_ik_kinematics_plugin_description.xml"
    )
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/ros/abb_ws/build/moveit_kinematics/kdl_kinematics_plugin/cmake_install.cmake")
  include("/home/ros/abb_ws/build/moveit_kinematics/lma_kinematics_plugin/cmake_install.cmake")
  include("/home/ros/abb_ws/build/moveit_kinematics/srv_kinematics_plugin/cmake_install.cmake")
  include("/home/ros/abb_ws/build/moveit_kinematics/ikfast_kinematics_plugin/cmake_install.cmake")
  include("/home/ros/abb_ws/build/moveit_kinematics/cached_ik_kinematics_plugin/cmake_install.cmake")
  include("/home/ros/abb_ws/build/moveit_kinematics/test/cmake_install.cmake")

endif()

