execute_process(COMMAND "/home/bezzo/bezzoUAV_ws/build/quadrotor_sim/joystick_pk/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/bezzo/bezzoUAV_ws/build/quadrotor_sim/joystick_pk/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
