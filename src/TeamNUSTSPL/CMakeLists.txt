# General parameters
cmake_minimum_required(VERSION 2.8.12)
project (tnrs-module)

include($ENV{PATH_TO_TEAM_NUST_DIR}/make/cmake/common.cmake)
include_directories(include)

find_path(EIGEN3_INCLUDE_DIRS "Eigen" PATHS "${SDK_DIR}/include" PATH_SUFFIXES "eigen3")
if (EIGEN3_INCLUDE_DIRS)
  include_directories ("${EIGEN3_INCLUDE_DIRS}")
endif (EIGEN3_INCLUDE_DIRS)

file (GLOB TNRS_SRC "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")

if(MODULE_IS_REMOTE)

add_executable (tnrs-module ${TNRS_SRC})
qi_use_lib(
  tnrs-module
  ALCOMMON
  ALPROXIES
  PTHREAD
  BOOST
  BOOST_FILESYSTEM
  BOOST_IOSTREAMS
  ALVISION
  OPENCV2_CORE
  OPENCV2_HIGHGUI
  OPENCV2_CALIB3D
  OPENCV2_OBJDETECT
  OPENCV2_VIDEO
)
target_link_libraries(tnrs-module
  libtnrs-utils.so
  libtnrs-base.so
  libtnrs-control-module.so
  libtnrs-planning-module.so
  libtnrs-motion-module.so
  libtnrs-gb-module.so
  libtnrs-localization-module.so
  libtnrs-vision-module.so
  libtnrs-game-comm-module.so
  libtnrs-user-comm-module.so
  -ljsoncpp
)
elseif (MODULE_IS_LOCAL_SIMULATED)
add_library (tnrs-module SHARED ${TNRS_SRC})
qi_use_lib(
  tnrs-module
  ALCOMMON
  ALPROXIES
  PTHREAD
  BOOST
  BOOST_FILESYSTEM
  BOOST_IOSTREAMS
  ALVISION
)
target_link_libraries(tnrs-module
  libtnrs-utils.so
  libtnrs-base.so
  libtnrs-control-module.so
  libtnrs-planning-module.so
  libtnrs-motion-module.so
  libtnrs-gb-module.so
  libtnrs-localization-module.so
  libtnrs-vision-module.so
  libtnrs-game-comm-module.so
  libtnrs-user-comm-module.so
  -ljsoncpp
)
else()
if(${ROBOT_VERSION} STREQUAL "V6")
add_executable (tnrs-module ${TNRS_SRC})
qi_use_lib(
  tnrs-module
  QI
  PTHREAD
  BOOST
  BOOST_FILESYSTEM
  BOOST_IOSTREAMS
)
target_link_libraries(tnrs-module
  -lfftw3f
  -lqpOASES
  -lnlopt
  -ljsoncpp
  -lalvalue
  -lalerror
  -ltnrs-sbpl-lib
  -ltnrs-random-lib
  -ltnrs-utils
  -ltnrs-base
  -ltnrs-control-module
  -ltnrs-behavior-manager
  -ltnrs-planning-module
  -ltnrs-motion-module
  -ltnrs-gb-module
  -ltnrs-localization-module
  -ltnrs-vision-module
  -ltnrs-game-comm-module
  -ltnrs-user-comm-module
)
else()
add_library (tnrs-module SHARED ${TNRS_SRC})
qi_use_lib(
  tnrs-module
  ALCOMMON
  ALPROXIES
  PTHREAD
  BOOST
  BOOST_FILESYSTEM
  BOOST_IOSTREAMS
  ALVISION
  OPENCV2_CORE
  OPENCV2_HIGHGUI
  OPENCV2_CALIB3D
  OPENCV2_OBJDETECT
  OPENCV2_VIDEO
)
target_link_libraries(tnrs-module
  libtnrs-utils.so
  libtnrs-base.so
  libtnrs-control-module.so
  libtnrs-planning-module.so
  libtnrs-motion-module.so
  libtnrs-gb-module.so
  libtnrs-localization-module.so
  libtnrs-vision-module.so
  libtnrs-game-comm-module.so
  libtnrs-user-comm-module.so
  -ljsoncpp
)
endif()
endif()

if (BUILD_TNRS_TESTS)
    set (test_DIR "${CMAKE_CURRENT_SOURCE_DIR}/tests/")
    file (GLOB TESTS RELATIVE "${test_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}/tests/*.cpp")
    foreach (testname ${TESTS})
        string(REPLACE ".cpp" "" targetname "${testname}")
        add_executable (${targetname} "${test_DIR}/${testname}")
        target_link_libraries (${targetname} tnrs-module
          libtnrs-memory-module.so
          libtnrs-debug-module.so
          libtnrs-control-module.so
          libtnrs-planning-module.so
          libtnrs-motion-module.so
          libtnrs-gb-module.so
          libtnrs-localization-module.so
          libtnrs-vision-module.so
          libtnrs-game-comm-module.so
          libtnrs-user-comm-module.so
        )
    endforeach (testname ${TESTS})
endif (BUILD_TNRS_TESTS)
