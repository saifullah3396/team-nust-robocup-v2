# General parameters
cmake_minimum_required(VERSION 2.8.12)
project (tnrs-gb-module)

include($ENV{PATH_TO_TEAM_NUST_DIR}/make/cmake/common.cmake)
include_directories(include $ENV{PATH_TO_TEAM_NUST_DIR}/resources/msgpack/include)

find_path(EIGEN3_INCLUDE_DIRS "Eigen" PATHS "${SDK_DIR}/include" PATH_SUFFIXES "eigen3")
if (EIGEN3_INCLUDE_DIRS)
  include_directories ("${EIGEN3_INCLUDE_DIRS}")
endif (EIGEN3_INCLUDE_DIRS)

file (GLOB SRC
      "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
      "${CMAKE_CURRENT_SOURCE_DIR}/src/StiffnessModule/*.cpp"
      "${CMAKE_CURRENT_SOURCE_DIR}/src/StiffnessModule/Types/*.cpp"
      "${CMAKE_CURRENT_SOURCE_DIR}/src/LedsModule/*.cpp"
      "${CMAKE_CURRENT_SOURCE_DIR}/src/LedsModule/Types/*.cpp"
      "${CMAKE_CURRENT_SOURCE_DIR}/src/WhistleDetector/*.cpp"
      "${CMAKE_CURRENT_SOURCE_DIR}/src/WhistleDetector/Types/*.cpp")

if(MODULE_IS_REMOTE OR MODULE_IS_LOCAL_SIMULATED)
  add_library (tnrs-gb-module SHARED ${SRC})

  qi_use_lib(tnrs-gb-module
    ALCOMMON
    ALPROXIES
  )

  # Linking external libraries installed as pre-requisites.
  # For remote usage the libraries can just be installed on the local pc.
  target_link_libraries(tnrs-gb-module -lfftw3 -lfftw3f -lm -lasound -ljsoncpp)
else()
  # Alsa sound package for whisle detector usage.
  find_package(alsa)
  include_directories(${ALSA_INCLUDE_DIRS})

  add_library (tnrs-gb-module SHARED ${SRC})

  if(${ROBOT_VERSION} STREQUAL "V6")
    qi_use_lib(tnrs-gb-module
      QI
      BOOST_SIGNALS
    )
    target_link_libraries(tnrs-gb-module
      #-lfftw3
      -lfftw3f
      libtnrs-utils.so
      libtnrs-base.so
      ${ALSA_LIBRARIES})
  else()
    # Linking libraries from the sdk.
    qi_use_lib(
      tnrs-gb-module
      ALCOMMON
      ALPROXIES
      ALSA
    )
    # Linking external libraries installed as pre-requisites in the toolchain
    # These libraries are compiled on the virtual-nao in virtualbox robot env
    # and placed in the folder cross-toolchain/cross/i686-aldebaran-linux-gnu/lib
    target_link_libraries(tnrs-gb-module -lfftw3 -lfftw3f -lm -ljsoncpp)
  endif()
endif()

target_link_libraries(tnrs-gb-module
  libtnrs-utils.so
  libtnrs-base.so
  libtnrs-control-module.so
  libtnrs-behavior-manager.so
)

if (BUILD_TNRS_TESTS)
    set (test_DIR "${CMAKE_CURRENT_SOURCE_DIR}/tests/")
    file (GLOB TESTS RELATIVE "${test_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}/tests/*.cpp")
    foreach (testname ${TESTS})
        string(REPLACE ".cpp" "" targetname "${testname}")
        add_executable (${targetname} "${test_DIR}/${testname}")
        target_link_libraries (${targetname} tnrs-gb-module
          libtnrs-utils.so
          libtnrs-base.so
          libtnrs-behavior-manager.so
        )
    endforeach (testname ${TESTS})
endif (BUILD_TNRS_TESTS)
