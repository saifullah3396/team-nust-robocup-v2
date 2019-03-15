# General parameters
cmake_minimum_required(VERSION 2.8.12)
project (tnrs-motion-module)

include($ENV{PATH_TO_TEAM_NUST_DIR}/make/cmake/common.cmake)
include_directories(include)
include_directories($ENV{PATH_TO_TEAM_NUST_DIR}/resources)

find_path(EIGEN3_INCLUDE_DIRS "Eigen" PATHS "${SDK_DIR}/include" PATH_SUFFIXES "eigen3")
if (EIGEN3_INCLUDE_DIRS)
  include_directories ("${EIGEN3_INCLUDE_DIRS}")
endif (EIGEN3_INCLUDE_DIRS)

file (GLOB TNRS_MOTION_MODULE_SRC 
      "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
      "${CMAKE_CURRENT_SOURCE_DIR}/src/KinematicsModule/*.cpp"
      "${CMAKE_CURRENT_SOURCE_DIR}/src/FallDetector/*.cpp"
      "${CMAKE_CURRENT_SOURCE_DIR}/src/TrajectoryPlanner/*.cpp"
      "${CMAKE_CURRENT_SOURCE_DIR}/src/BalanceModule/*.cpp"
      "${CMAKE_CURRENT_SOURCE_DIR}/src/BalanceModule/Types/*.cpp"
      "${CMAKE_CURRENT_SOURCE_DIR}/src/PostureModule/*.cpp"
      "${CMAKE_CURRENT_SOURCE_DIR}/src/PostureModule/Types/*.cpp"
      "${CMAKE_CURRENT_SOURCE_DIR}/src/KickModule/*.cpp"
      "${CMAKE_CURRENT_SOURCE_DIR}/src/KickModule/Types/*.cpp"
      "${CMAKE_CURRENT_SOURCE_DIR}/src/MotionPlayback/*.cpp"
      "${CMAKE_CURRENT_SOURCE_DIR}/src/MotionPlayback/Types/*.cpp"
      "${CMAKE_CURRENT_SOURCE_DIR}/src/Teleop/*.cpp"
      "${CMAKE_CURRENT_SOURCE_DIR}/src/Teleop/Types/*.cpp"
      "${CMAKE_CURRENT_SOURCE_DIR}/src/MovementModule/*.cpp"
      "${CMAKE_CURRENT_SOURCE_DIR}/src/MovementModule/Types/*.cpp"
      "${CMAKE_CURRENT_SOURCE_DIR}/src/BallThrow/*.cpp"
      "${CMAKE_CURRENT_SOURCE_DIR}/src/BallThrow/Types/*.cpp"
      "${CMAKE_CURRENT_SOURCE_DIR}/src/GetupModule/*.cpp"
      "${CMAKE_CURRENT_SOURCE_DIR}/src/GetupModule/Types/*.cpp"
      "${CMAKE_CURRENT_SOURCE_DIR}/src/HeadControl/*.cpp"
      #"${CMAKE_CURRENT_SOURCE_DIR}/src/HeadControl/Types/*.cpp"
      "${CMAKE_CURRENT_SOURCE_DIR}/src/DiveModule/*.cpp"
      "${CMAKE_CURRENT_SOURCE_DIR}/src/DiveModule/Types/*.cpp"
    )

# Building the code for remote usage.
if(MODULE_IS_REMOTE OR MODULE_IS_LOCAL_SIMULATED)
  add_library (tnrs-motion-module SHARED ${TNRS_MOTION_MODULE_SRC}) 
  # Linking libraries from the sdk.
  qi_use_lib(tnrs-motion-module
    ALCOMMON
    ALPROXIES
  )

  # Linking external libraries installed as pre-requisites.
  # For remote usage the libraries can just be installed on the local pc.
  target_link_libraries(tnrs-motion-module 
    libtnrs-utils.so
    libtnrs-base.so
    libtnrs-user-comm-module.so
    libtnrs-control-module.so
    libtnrs-behavior-manager.so
    -lnlopt 
    -ljsoncpp 
    -lqpOASES 
    -lsymengine
    -lteuchos
    -lbfd
    -lgmp)
else()
  add_library (tnrs-motion-module SHARED ${TNRS_MOTION_MODULE_SRC})
  # Linking libraries from the sdk.
  if(${ROBOT_VERSION} STREQUAL "V6")
      qi_use_lib(tnrs-motion-module
        QI
        BOOST_SIGNALS
      )
      target_link_libraries(tnrs-motion-module
      libtnrs-utils.so
      libtnrs-base.so
      libtnrs-user-comm-module.so
      libtnrs-control-module.so
      libtnrs-behavior-manager.so
      -lnlopt
      -ljsoncpp
      libqpOASES.so)
  else()
      qi_use_lib(
        tnrs-motion-module
        ALCOMMON
        ALPROXIES
      )

      # Linking external libraries installed as pre-requisites in the toolchain
      # These libraries are compiled on the virtual-nao in virtualbox robot env
      # and placed in the folder cross-toolchain/cross/i686-aldebaran-linux-gnu/lib
      target_link_libraries(tnrs-motion-module
      libtnrs-utils.so
      libtnrs-base.so
      libtnrs-user-comm-module.so
      libtnrs-control-module.so
      libtnrs-behavior-manager.so
      -lnlopt
      -ljsoncpp
      libqpOASES.so)
  endif()
endif()
    
if (BUILD_TNRS_TESTS)
    set (test_DIR "${CMAKE_CURRENT_SOURCE_DIR}/tests/")
    file (GLOB test_SRC "${CMAKE_CURRENT_SOURCE_DIR}/tests/src/*.cpp")
    file (GLOB TESTS RELATIVE "${test_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}/tests/*.cpp")
    foreach (testname ${TESTS})
        string(REPLACE ".cpp" "" targetname "${testname}")
        add_executable (${targetname} "${test_DIR}/${testname}" ${test_SRC})
        target_link_libraries (${targetname} tnrs-motion-module
          libtnrs-utils.so
          libtnrs-base.so
          libtnrs-user-comm-module.so
          libtnrs-control-module.so
          libtnrs-behavior-manager.so
        )
    endforeach (testname ${TESTS})
endif (BUILD_TNRS_TESTS)

if (BUILD_TNRS_EXAMPLES)
  if(MODULE_IS_REMOTE)
    set (examples_DIR "${CMAKE_CURRENT_SOURCE_DIR}/examples/")
    file (GLOB EXAMPLES RELATIVE "${examples_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}/examples/*.cpp")
    foreach (examplename ${EXAMPLES})
        string(REPLACE ".cpp" "" targetname "${examplename}")
        add_executable (${targetname} "${examples_DIR}/${examplename}")
        target_link_libraries (${targetname} tnrs-vision-module
          libtnrs-motion-module.so
        )
    endforeach (examplename ${EXAMPLES})
  else()
    set (examples_DIR "${CMAKE_CURRENT_SOURCE_DIR}/examples")
    MESSAGE(STATUS ${examples_DIR})
    file (GLOB EXAMPLES RELATIVE "${examples_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}/examples/*")
    MESSAGE(STATUS ${EXAMPLES})
    include_directories("${examples_DIR}/${EXAMPLES}/include")
    foreach (examplename ${EXAMPLES})
        file (GLOB EXAMPLE_SRC "${examples_DIR}/${EXAMPLES}/src/*.cpp")
        MESSAGE("EXAMPLE_SRC:" ${EXAMPLE_SRC})
        MESSAGE("${examples_DIR}/${EXAMPLES}/src/*.cpp")
        add_library (${examplename} SHARED "${EXAMPLE_SRC}")
        qi_use_lib(${examplename}
          ALCOMMON
          ALPROXIES
          BOOST
          BOOST_FILESYSTEM
          BOOST_IOSTREAMS
          PTHREAD
        )
        # Linking external libraries installed as pre-requisites in the toolchain
        # These libraries are compiled on the virtual-nao in virtualbox robot env
        # and placed in the folder cross-toolchain/cross/i686-aldebaran-linux-gnu/lib
        target_link_libraries(${examplename}
            -lnlopt -ljsoncpp -lqpOASES
            libtnrs-motion-module.so
            libtnrs-utils.so
            libtnrs-base.so
            libtnrs-user-comm-module.so
            libtnrs-control-module.so
            libtnrs-behavior-manager.so)
    endforeach (examplename ${EXAMPLES})
  endif()
endif (BUILD_TNRS_EXAMPLES)
