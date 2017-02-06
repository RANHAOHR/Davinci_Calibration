execute_process(COMMAND "/home/rxh349/ros_ws/src/Davinci_Calibration/davinci_calibration/cmake-build-debug/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/rxh349/ros_ws/src/Davinci_Calibration/davinci_calibration/cmake-build-debug/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
