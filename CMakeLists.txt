cmake_minimum_required(VERSION 3.6)
project(Davinci_Calibration)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES
        davinci_calibration/include/davinci_calibrator/davinci_calibrator.h
        davinci_calibration/src/davinci_calibrator/davinci_calibrator.cpp
        davinci_calibration/src/davinci_calibrator/davinci_calibrator_node.cpp
        davinci_calibration/CMakeLists.txt
        davinci_calibration/package.xml)

add_executable(Davinci_Calibration ${SOURCE_FILES})
