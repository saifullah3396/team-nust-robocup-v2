cmake_minimum_required(VERSION 2.8.12)

find_package(qibuild)

include_directories("$ENV{PATH_TO_TEAM_NUST_DIR}/src")
include_directories("$ENV{PATH_TO_TEAM_NUST_DIR}/src/TNRSModules")

MACRO(SUBDIRLIST result curdir)
  FILE(GLOB children RELATIVE ${curdir} ${curdir}/*)
  SET(dirlist "")
  FOREACH(child ${children})
    IF(IS_DIRECTORY ${curdir}/${child})
      LIST(APPEND dirlist ${child})
    ENDIF()
  ENDFOREACH()
  SET(${result} ${dirlist})
ENDMACRO()

option(MODULE_IS_REMOTE "The module is compiled as a remote module (ON or OFF)" ON)
option(MODULE_IS_LOCAL_SIMULATED "The module locally simulated or not (ON or OFF)" OFF)
option(USE_NAOQI_MOTION_PROXY "The module uses almotionproxy (ON or OFF)" ON)
option(USE_NAOQI_VIDEO_PROXY "The module uses alvideodeviceproxy (ON or OFF)" ON)

if (USE_NAOQI_MOTION_PROXY)
    add_definitions(-DNAOQI_MOTION_PROXY_AVAILABLE)
endif()

if (USE_NAOQI_VIDEO_PROXY)
    add_definitions(-DNAOQI_VIDEO_PROXY_AVAILABLE)
endif()

set(TNRS_COMPONENTS
    tnrs-utils
    tnrs-config-manager
    tnrs-debug-module
    tnrs-base
    tnrs-comm-module
    tnrs-control-module
    tnrs-behavior-manager
    tnrs-planning-module
    tnrs-random-lib
    tnrs-motion-module
    tnrs-gb-module
    tnrs-localization-module
    tnrs-vision-module
    )

####################################
# Settings
####################################
set (CMAKE_SKIP_ASSEMBLY_SOURCE_RULES OFF)
set (CMAKE_SKIP_PREPROCESSED_SOURCE_RULES OFF)
set (CMAKE_VERBOSE_MAKEFILE ON)
set (CMAKE_RULE_MESSAGES OFF CACHE BOOL "")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

####################################
# Options
####################################
# To be used later for testing code.
SET (TOOLCHAIN_NAME CACHE STRING "Name of the toolchain.")
SET (BUILD_TNRS_TESTS OFF CACHE BOOL "Whether to build tests.")
SET (BUILD_TNRS_EXAMPLES OFF CACHE BOOL "Whether to build example.")
SET (ROBOT_VERSION "V5" CACHE STRING "Robot version")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DDEBUG_BUILD")

if(BUILD_TNRS_TESTS)
  if (MODULE_IS_REMOTE) 
    #add_definitions(-DNAOQI_VIDEO_PROXY_AVAILABLE)
    add_definitions(-DMODULE_IS_REMOTE -DROOT_DIR="$ENV{PATH_TO_TEAM_NUST_DIR}/src")
  elseif (MODULE_IS_LOCAL_SIMULATED) 
    #add_definitions(-DNAOQI_VIDEO_PROXY_AVAILABLE)
    add_definitions(-DMODULE_IS_LOCAL_SIMULATED -DROOT_DIR="$ENV{PATH_TO_TEAM_NUST_DIR}/src")
  else()
    message(FATAL_ERROR "You can not build tests with cross compilation toolchain.")
  endif()
else()
  if (MODULE_IS_REMOTE) 
    add_definitions(-DMODULE_IS_REMOTE -DROOT_DIR="$ENV{PATH_TO_TEAM_NUST_DIR}/src")
  elseif (MODULE_IS_LOCAL_SIMULATED) 
    add_definitions(-DMODULE_IS_LOCAL_SIMULATED -DROOT_DIR="$ENV{PATH_TO_TEAM_NUST_DIR}/src")
  else()
    add_definitions(-DROOT_DIR="$ENV{PATH_TO_TEAM_NUST_DIR}/src")
  endif()
endif()

set(TNRS_BUILD "$ENV{PATH_TO_TEAM_NUST_DIR}/build-${ROBOT_VERSION}")
if (MODULE_IS_REMOTE) 
    if (USE_NAOQI_MOTION_PROXY)
        set(TNRS_BUILD_DEBUG "${TNRS_BUILD}/Debug-Motion/remote")
        set(TNRS_BUILD_RELEASE "${TNRS_BUILD}/Release-Motion/remote")
    else()
        set(TNRS_BUILD_DEBUG "${TNRS_BUILD}/Debug/remote")
        set(TNRS_BUILD_RELEASE "${TNRS_BUILD}/Release/remote")
    endif()
elseif(MODULE_IS_LOCAL_SIMULATED)
    if (USE_NAOQI_MOTION_PROXY)
        set(TNRS_BUILD_DEBUG "${TNRS_BUILD}/Debug-Motion/sim")
        set(TNRS_BUILD_RELEASE "${TNRS_BUILD}/Release-Motion/sim")
    else()
        set(TNRS_BUILD_DEBUG "${TNRS_BUILD}/Debug/sim")
        set(TNRS_BUILD_RELEASE "${TNRS_BUILD}/Release/sim")
    endif()
else()
    if (USE_NAOQI_MOTION_PROXY)
        set(TNRS_BUILD_DEBUG "${TNRS_BUILD}/Debug-Motion/cross")
        set(TNRS_BUILD_RELEASE "${TNRS_BUILD}/Release-Motion/cross")
    else()
        set(TNRS_BUILD_DEBUG "${TNRS_BUILD}/Debug/cross")
        set(TNRS_BUILD_RELEASE "${TNRS_BUILD}/Release/cross")
    endif()
endif()

if (${CMAKE_BUILD_TYPE} STREQUAL "Debug")
  set (FIND_BUILD_DIR ${TNRS_BUILD_DEBUG})
else()
  set (FIND_BUILD_DIR ${TNRS_BUILD_RELEASE})
endif()

link_directories(${FIND_BUILD_DIR}/lib)

set (CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE "${TNRS_BUILD_RELEASE}/bin")
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG "${TNRS_BUILD_DEBUG}/bin")
set (CMAKE_LIBRARY_OUTPUT_DIRECTORY_RELEASE "${TNRS_BUILD_RELEASE}/lib")
set (CMAKE_LIBRARY_OUTPUT_DIRECTORY_DEBUG "${TNRS_BUILD_DEBUG}/lib")

MESSAGE(STATUS "Output Directory: ${CMAKE_BUILD_TYPE}")

link_directories("$ENV{PATH_TO_TEAM_NUST_DIR}/resources/RandomLib/lib")

#message("MODULE PATH" ${CMAKE_MODULE_PATH})

#SET(CXX_WARN_FLAGS_EIGEN -Wall -Wfloat-equal)
#SET(CXX_WARN_FLAGS ${CXX_WARN_FLAGS_EIGEN} -pedantic)
if(${ROBOT_VERSION} STREQUAL "V6")
set(CXX11_FLAG -std=gnu++11)
else()
set(CXX11_FLAG -std=c++11)
endif()

SET(CXX_FLAGS ${CXX_WARN_FLAGS} ${CXX11_FLAG} -Wno-deprecated-declarations -msse -msse2 -mavx -mavx2 -mfma)
add_definitions(${CXX_FLAGS} )
include_directories("$ENV{PATH_TO_TEAM_NUST_DIR}/resources/Eigen")
include_directories("$ENV{PATH_TO_TEAM_NUST_DIR}")
include_directories("$ENV{PATH_TO_TEAM_NUST_DIR}/src")
include_directories("$ENV{PATH_TO_TEAM_NUST_DIR}/src/ProcessingChilds")

# Linking for V6 Robots
if(${ROBOT_VERSION} STREQUAL "V6")
  add_definitions(-DV6_CROSS_BUILD) #allow sse instructions
  include_directories(${YOCTO_SDK_HOST_SYSROOT}/../../../libnaoqi/include)
  link_directories(${YOCTO_SDK_HOST_SYSROOT}/../../../libnaoqi/lib)
else()
  if (NOT MODULE_IS_REMOTE)
    if (NOT MODULE_IS_LOCAL_SIMULATED)
      set(CMAKE_C_COMPILER $ENV{CTC_SYSROOT}/usr/local/bin/gcc)
      set(CMAKE_CXX_COMPILER $ENV{CTC_SYSROOT}/usr/local/bin/g++)
    endif()
  else()
    include_directories($ENV{PATH_TO_TEAM_NUST_DIR}/remote-depends)
    include_directories($ENV{PATH_TO_TEAM_NUST_DIR}/remote-depends)
  endif()
endif()
