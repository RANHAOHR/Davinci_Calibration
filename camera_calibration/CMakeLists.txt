cmake_minimum_required(VERSION 2.8)
project(camera_calibration)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation cwru_opencv_common
)

find_package(OpenCV REQUIRED)
include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS})

add_message_files(
  FILES
  points.msg
  corners.msg
  intrinsic_param.msg
)

catkin_python_setup()

generate_messages(
  DEPENDENCIES
  std_msgs
)


if(CATKIN_ENABLE_TESTING)
  # Unit test of calibrator.py
  catkin_add_nosetests(test/directed.py)

  # Tests simple calibration dataset
  catkin_download_test_data(camera_calibration.tar.gz http://download.ros.org/data/camera_calibration/camera_calibration.tar.gz
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/tests
    MD5 6da43ea314640a4c15dd7a90cbc3aee0
  )

  # Tests multiple checkerboards
  catkin_download_test_data(multi_board_calibration.tar.gz http://download.ros.org/data/camera_calibration/multi_board_calibration.tar.gz
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/tests
    MD5 ddc0f69582d140e33f9d3bfb681956bb
  )
  catkin_add_nosetests(test/multiple_boards.py)
endif()

catkin_install_python(PROGRAMS nodes/cameracalibrator.py
  nodes/cameracheck.py
  scripts/tarfile_calibration.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

  include_directories(
      include
      ${catkin_INCLUDE_DIRS}
  )

add_library(davinci_calibrator src/davinci_calibrator/davinci_calibrator.cpp)

target_link_libraries(davinci_calibrator
      ${OpenCV_LIBRARIES}
      ${catkin_LIBRARIES}
)

add_executable(davinci_calibrator_node src/davinci_calibrator/davinci_calibrator_node.cpp)
target_link_libraries(davinci_calibrator ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

target_link_libraries(davinci_calibrator_node
      davinci_calibrator
      ${OpenCV_LIBRARIES}
      ${catkin_LIBRARIES}
)


