# General parameters
cmake_minimum_required(VERSION 2.8.12)
project (tnrs-user-comm-module)

include($ENV{PATH_TO_TEAM_NUST_DIR}/make/cmake/common.cmake)
include_directories(include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

find_path(EIGEN3_INCLUDE_DIRS "Eigen" PATHS "${SDK_DIR}/include" PATH_SUFFIXES "eigen3")
if (EIGEN3_INCLUDE_DIRS)
  include_directories ("${EIGEN3_INCLUDE_DIRS}")
endif (EIGEN3_INCLUDE_DIRS)

file (GLOB TNRS_USER_COMM_MODULE_SRC "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")
add_library (tnrs-user-comm-module SHARED ${TNRS_USER_COMM_MODULE_SRC})

if(${ROBOT_VERSION} STREQUAL "V6")
qi_use_lib(tnrs-user-comm-module
  PTHREAD
  BOOST_IOSTREAMS
  BOOST_THREAD
  BOOST_SERIALIZATION
  OPENCV2_CORE
)
target_link_libraries(tnrs-user-comm-module 
  libtnrs-utils.so 
  libtnrs-base.so
)
else()
qi_use_lib(tnrs-user-comm-module
  PTHREAD
  BOOST_IOSTREAMS
  BOOST_THREAD
  BOOST_SERIALIZATION
  OPENCV2_CORE
)
target_link_libraries(tnrs-user-comm-module
  libtnrs-utils.so
  libtnrs-base.so
)
endif()
    
if (BUILD_TNRS_TESTS)
    set (test_DIR "${CMAKE_CURRENT_SOURCE_DIR}/tests/")
    file (GLOB TESTS RELATIVE "${test_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}/tests/*.cpp")
    foreach (testname ${TESTS})
        string(REPLACE ".cpp" "" targetname "${testname}")
        add_executable  (${targetname} "${test_DIR}/${testname}")
        target_link_libraries (${targetname} 
          tnrs-user-comm-module
          libtnrs-utils.so 
          libtnrs-base.so
          -ljsoncpp)
    endforeach (testname ${TESTS})
endif (BUILD_TNRS_TESTS)
