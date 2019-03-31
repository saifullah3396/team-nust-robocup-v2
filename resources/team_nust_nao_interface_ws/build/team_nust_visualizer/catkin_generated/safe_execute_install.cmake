execute_process(COMMAND "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/build/team_nust_visualizer/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/build/team_nust_visualizer/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
