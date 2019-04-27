execute_process(COMMAND "/home/muptii/Documents/robocup/team-nust-robocup-v2/resources/team_nust_nao_interface_ws/build/team_nust_visualizer/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/muptii/Documents/robocup/team-nust-robocup-v2/resources/team_nust_nao_interface_ws/build/team_nust_visualizer/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
