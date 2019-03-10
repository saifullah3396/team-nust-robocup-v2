# General parameters
cmake_minimum_required(VERSION 2.8.12)
project (tnrs-vision-module)

include($ENV{PATH_TO_TEAM_NUST_DIR}/make/cmake/common.cmake)
include_directories(include)
include_directories($ENV{PATH_TO_TEAM_NUST_DIR}/remote-depends)

find_path(EIGEN3_INCLUDE_DIRS "Eigen" PATHS "${SDK_DIR}/include" PATH_SUFFIXES "eigen3")
if (EIGEN3_INCLUDE_DIRS)
  include_directories ("${EIGEN3_INCLUDE_DIRS}")
endif (EIGEN3_INCLUDE_DIRS)

file (GLOB TNRS_VISION_MODULE_SRC 
      "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
      "${CMAKE_CURRENT_SOURCE_DIR}/src/CameraModule/*.cpp"
      "${CMAKE_CURRENT_SOURCE_DIR}/src/FeatureExtraction/*.cpp")
      
# Building the code for remote usage.
if(MODULE_IS_REMOTE OR MODULE_IS_LOCAL_SIMULATED)
  add_library (tnrs-vision-module SHARED ${TNRS_VISION_MODULE_SRC})
  # Linking libraries from the sdk.
  qi_use_lib(tnrs-vision-module
    ALCOMMON
    ALPROXIES
    PTHREAD
    BOOST
    BOOST_FILESYSTEM
    BOOST_IOSTREAMS
    ALVISION
    OPENCV2_CORE
    OPENCV2_HIGHGUI
    OPENCV2_IMGPROC
    OPENCV2_CALIB3D
    OPENCV2_OBJDETECT
    OPENCV2_VIDEO
    OPENCV2_FEATURES2D
    V4L
  )

  # Linking external libraries installed as pre-requisites.
  # For remote usage the libraries can just be installed on the local pc.
  link_directories($ENV{PATH_TO_TEAM_NUST_DIR}/remote-depends/lib)
  target_link_libraries(tnrs-vision-module -lfftw3 -lfftw3f -lm -llapack /home/sensei/tensorflow/tensorflow/lite/tools/make/gen/linux_x86_64/lib/libtensorflow-lite.a -lpthread -ldl)
else()
  add_library (tnrs-vision-module SHARED ${TNRS_VISION_MODULE_SRC})
  # Linking libraries from the sdk.
  qi_use_lib(
    tnrs-vision-module
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
    OPENCV2_FEATURES2D
    OPENCV2_FLANN
    V4L
  )

  # Linking external libraries installed as pre-requisites in the toolchain
  # These libraries are compiled on the virtual-nao in virtualbox robot env
  # and placed in the folder cross-toolchain/cross/i686-aldebaran-linux-gnu/lib
  target_link_libraries(tnrs-vision-module -lfftw3 -lfftw3f -lm -llapack -lrefblas -lgfortran)
endif()

target_link_libraries(tnrs-vision-module
  libtnrs-utils.so
  libtnrs-base.so
  libtnrs-user-comm-module.so
  libtnrs-behavior-manager.so
)
    
if (BUILD_TNRS_TESTS)
    set (test_DIR "${CMAKE_CURRENT_SOURCE_DIR}/tests/")
    file (GLOB TESTS RELATIVE "${test_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}/tests/*.cpp")
    foreach (testname ${TESTS})
        string(REPLACE ".cpp" "" targetname "${testname}")
        add_executable (${targetname} "${test_DIR}/${testname}")
        target_link_libraries (${targetname} tnrs-vision-module
          libtnrs-utils.so
          libtnrs-base.so
          libtnrs-user-comm-module.so
          libtnrs-behavior-manager.so
        )
    endforeach (testname ${TESTS})
endif (BUILD_TNRS_TESTS)

if (BUILD_TNRS_EXAMPLES)
    set (examples_DIR "${CMAKE_CURRENT_SOURCE_DIR}/examples/")
    file (GLOB EXAMPLES RELATIVE "${examples_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}/examples/*.cpp")
    foreach (examplename ${EXAMPLES})
        string(REPLACE ".cpp" "" targetname "${examplename}")
        add_executable (${targetname} "${examples_DIR}/${examplename}")
        target_link_libraries (${targetname} tnrs-vision-module
          libtnrs-utils.so
          libtnrs-base.so
          libtnrs-user-comm-module.so
          libtnrs-behavior-manager.so
          libtnrs-motion-module.so
        )
    endforeach (examplename ${EXAMPLES})
endif (BUILD_TNRS_EXAMPLES)
