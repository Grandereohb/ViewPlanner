execute_process(COMMAND "/home/ros/abb_ws/build/rl_viewplanning/ipa_kifz_viewplanning/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/ros/abb_ws/build/rl_viewplanning/ipa_kifz_viewplanning/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
