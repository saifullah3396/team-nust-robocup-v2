# General parameters
cmake_minimum_required(VERSION 2.8.12)
project (tnrs-localization-module)

include($ENV{PATH_TO_TEAM_NUST_DIR}/make/cmake/common.cmake)
include_directories(include)

find_path(EIGEN3_INCLUDE_DIRS "Eigen" PATHS "${SDK_DIR}/include" PATH_SUFFIXES "eigen3")
if (EIGEN3_INCLUDE_DIRS)
  include_directories ("${EIGEN3_INCLUDE_DIRS}")
endif (EIGEN3_INCLUDE_DIRS)

file (GLOB TNRS_LOCALIZATION_MODULE_SRC "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")
add_library (tnrs-localization-module SHARED ${TNRS_LOCALIZATION_MODULE_SRC})


if(${ROBOT_VERSION} STREQUAL "V6")
qi_use_lib(tnrs-localization-module
  BOOST
)
target_link_libraries(tnrs-localization-module
  libtnrs-utils.so
  libtnrs-base.so
  libtnrs-user-comm-module.so
  libtnrs-random-lib.so
  -lopencv_core
  -lopencv_highgui
  -lopencv_calib3d
  -lopencv_objdetect
  -lopencv_video
)
else()
qi_use_lib(tnrs-localization-module
  ALCOMMON
  ALPROXIES
  BOOST
  OPENCV2_CORE
  OPENCV2_HIGHGUI
  OPENCV2_CALIB3D
  OPENCV2_OBJDETECT
  OPENCV2_VIDEO
  PTHREAD
)

target_link_libraries(tnrs-localization-module
  libtnrs-utils.so
  libtnrs-base.so
  libtnrs-user-comm-module.so
  libtnrs-random-lib.so
)
endif()
    
if (BUILD_TNRS_TESTS)
    set (test_DIR "${CMAKE_CURRENT_SOURCE_DIR}/tests/")
    file (GLOB TESTS RELATIVE "${test_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}/tests/*.cpp")
    foreach (testname ${TESTS})
        string(REPLACE ".cpp" "" targetname "${testname}")
        add_executable (${targetname} "${test_DIR}/${testname}")
        target_link_libraries (${targetname} tnrs-localization-module
          libtnrs-utils.so
          libtnrs-base.so
          libtnrs-user-comm-module.so
          libtnrs-random-lib.so
          -ljsoncpp
        )
    endforeach (testname ${TESTS})
endif (BUILD_TNRS_TESTS)
