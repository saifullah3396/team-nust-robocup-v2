# General parameters
cmake_minimum_required(VERSION 2.8.12)
project (tnrs-camera-calibrator)

find_package(qibuild)

include($ENV{PATH_TO_TEAM_NUST_DIR}/make/cmake/common.cmake)
include_directories(include)
include_directories(../)


if(MODULE_IS_REMOTE OR MODULE_IS_LOCAL_SIMULATED)
  file (GLOB SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")
  add_executable (tnrs-camera-calibrator ${SOURCE})
  # Linking libraries from the sdk.
  qi_use_lib(tnrs-camera-calibrator
    OPENCV2_CORE
    OPENCV2_HIGHGUI
    OPENCV2_IMGPROC
    OPENCV2_CALIB3D
    OPENCV2_OBJDETECT
    OPENCV2_VIDEO
    OPENCV2_FEATURES2D
  )
  target_link_libraries(tnrs-camera-calibrator -ljsoncpp)
endif()
