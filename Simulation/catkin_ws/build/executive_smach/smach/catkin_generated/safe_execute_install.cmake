execute_process(COMMAND "/home/alex/pit-navigator-utah/Simulation/catkin_ws/build/executive_smach/smach/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/alex/pit-navigator-utah/Simulation/catkin_ws/build/executive_smach/smach/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
