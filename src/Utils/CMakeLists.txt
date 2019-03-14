# General parameters
cmake_minimum_required(VERSION 2.8.12)
project (tnrs-utils)

include($ENV{PATH_TO_TEAM_NUST_DIR}/make/cmake/common.cmake)
include_directories(include)

# Add the build parameters
add_definitions(
  -DNON_MATLAB_PARSING=1 # Vrep configuration parameter
  -DMAX_EXT_API_CONNECTIONS=255 # Vrep max connections parameter
)

####################################
# Settings
####################################
set (TNRS_UTILS_DIR "${CMAKE_CURRENT_SOURCE_DIR}")

####################################
# Other parameters
####################################

find_path(EIGEN3_INCLUDE_DIRS "Eigen" PATHS "${SDK_DIR}/include" PATH_SUFFIXES "eigen3")
if (EIGEN3_INCLUDE_DIRS)
  include_directories ("${EIGEN3_INCLUDE_DIRS}")
endif (EIGEN3_INCLUDE_DIRS)

file (GLOB SRC
  "${TNRS_UTILS_DIR}/src/*.cpp"
  "${TNRS_UTILS_DIR}/../TNRSModules/BehaviorConfigs/src/*.cpp"
  "${TNRS_UTILS_DIR}/../TNRSModules/BehaviorConfigs/src/MBConfigs/*.cpp"
  "${TNRS_UTILS_DIR}/../TNRSModules/BehaviorConfigs/src/GBConfigs/*.cpp"
  "${TNRS_UTILS_DIR}/../TNRSModules/BehaviorConfigs/src/PBConfigs/*.cpp"
  "${TNRS_UTILS_DIR}/src/DataHolders/*.cpp"
  "${TNRS_UTILS_DIR}/src/DataHolders/Camera.cpp"
  "${TNRS_UTILS_DIR}/src/PathPlanner/*.cpp"
  "${TNRS_UTILS_DIR}/src/Solvers/*.cpp"
  "${TNRS_UTILS_DIR}/src/Splines/*.cpp"
  "${TNRS_UTILS_DIR}/src/VREP/*.c"
)
add_library (tnrs-utils SHARED ${SRC})

if(${ROBOT_VERSION} STREQUAL "V6")
    qi_use_lib(tnrs-utils
      BOOST
      BOOST_FILESYSTEM
      BOOST_IOSTREAMS
      PTHREAD
      OPENCV3_CORE
      OPENCV3_HIGHGUI
      OPENCV3_IMGPROC
      OPENCV3_IMGCODECS
      V4L
      JPEG
    )
  target_link_libraries(tnrs-utils
    libtnrs-sbpl-lib.so
    -lnlopt
    -ljsoncpp
    -lalcommon)
else()
    qi_use_lib(tnrs-utils
      ALCOMMON
      BOOST
      BOOST_FILESYSTEM
      BOOST_IOSTREAMS
      OPENCV2_CORE
      OPENCV2_HIGHGUI
      OPENCV2_IMGPROC
      OPENCV2_IMGCODECS
      PTHREAD
      V4L
      JPEG
    )
  target_link_libraries(tnrs-utils
    libtnrs-sbpl-lib.so -lnlopt -ljsoncpp)

endif()
#if (BUILD_TNRS_TESTS)
    set (test_DIR "${TNRS_UTILS_DIR}/tests/")
    file (GLOB TESTS RELATIVE "${test_DIR}" "${TNRS_UTILS_DIR}/tests/*.cpp")
    foreach (testname ${TESTS})
        string(REPLACE ".cpp" "" targetname "${testname}")
        add_executable (${targetname} "${test_DIR}/${testname}")
        target_link_libraries (${targetname} tnrs-utils)
    endforeach (testname ${TESTS})
#endif (BUILD_TNRS_TESTS)
