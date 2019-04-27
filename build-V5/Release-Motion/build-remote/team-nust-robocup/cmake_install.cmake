# Install script for directory: /home/muptii/Documents/robocup/team-nust-robocup-v2

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "runtime")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/qi" TYPE DIRECTORY FILES "" USE_SOURCE_PERMISSIONS REGEX "/[^/]*\\.pyc$" EXCLUDE REGEX "/\\_\\_pycache\\_\\_$" EXCLUDE)
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "runtime")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/qi" TYPE FILE FILES "/home/muptii/Documents/robocup/team-nust-robocup-v2/build-V5/Release-Motion/build-remote/team-nust-robocup/path.conf")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "runtime")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/qi" TYPE DIRECTORY FILES "" USE_SOURCE_PERMISSIONS REGEX "/[^/]*\\.pyc$" EXCLUDE REGEX "/\\_\\_pycache\\_\\_$" EXCLUDE)
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "runtime")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/qi" TYPE FILE FILES "/home/muptii/Documents/robocup/team-nust-robocup-v2/build-V5/Release-Motion/build-remote/team-nust-robocup/path.conf")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/muptii/Documents/robocup/team-nust-robocup-v2/build-V5/Release-Motion/build-remote/team-nust-robocup/resources/fftw3/cmake_install.cmake")
  include("/home/muptii/Documents/robocup/team-nust-robocup-v2/build-V5/Release-Motion/build-remote/team-nust-robocup/resources/jsoncpp/cmake_install.cmake")
  include("/home/muptii/Documents/robocup/team-nust-robocup-v2/build-V5/Release-Motion/build-remote/team-nust-robocup/resources/nlopt/cmake_install.cmake")
  include("/home/muptii/Documents/robocup/team-nust-robocup-v2/build-V5/Release-Motion/build-remote/team-nust-robocup/resources/qpOASES/cmake_install.cmake")
  include("/home/muptii/Documents/robocup/team-nust-robocup-v2/build-V5/Release-Motion/build-remote/team-nust-robocup/resources/RandomLib/cmake_install.cmake")
  include("/home/muptii/Documents/robocup/team-nust-robocup-v2/build-V5/Release-Motion/build-remote/team-nust-robocup/resources/SbplLib/cmake_install.cmake")
  include("/home/muptii/Documents/robocup/team-nust-robocup-v2/build-V5/Release-Motion/build-remote/team-nust-robocup/src/Utils/cmake_install.cmake")
  include("/home/muptii/Documents/robocup/team-nust-robocup-v2/build-V5/Release-Motion/build-remote/team-nust-robocup/src/TNRSBase/cmake_install.cmake")
  include("/home/muptii/Documents/robocup/team-nust-robocup-v2/build-V5/Release-Motion/build-remote/team-nust-robocup/src/BehaviorManager/cmake_install.cmake")
  include("/home/muptii/Documents/robocup/team-nust-robocup-v2/build-V5/Release-Motion/build-remote/team-nust-robocup/src/TNRSModules/GameCommModule/cmake_install.cmake")
  include("/home/muptii/Documents/robocup/team-nust-robocup-v2/build-V5/Release-Motion/build-remote/team-nust-robocup/src/TNRSModules/UserCommModule/cmake_install.cmake")
  include("/home/muptii/Documents/robocup/team-nust-robocup-v2/build-V5/Release-Motion/build-remote/team-nust-robocup/src/TNRSModules/ControlModule/cmake_install.cmake")
  include("/home/muptii/Documents/robocup/team-nust-robocup-v2/build-V5/Release-Motion/build-remote/team-nust-robocup/src/TNRSModules/GBModule/cmake_install.cmake")
  include("/home/muptii/Documents/robocup/team-nust-robocup-v2/build-V5/Release-Motion/build-remote/team-nust-robocup/src/TNRSModules/MotionModule/cmake_install.cmake")
  include("/home/muptii/Documents/robocup/team-nust-robocup-v2/build-V5/Release-Motion/build-remote/team-nust-robocup/src/TNRSModules/PlanningModule/cmake_install.cmake")
  include("/home/muptii/Documents/robocup/team-nust-robocup-v2/build-V5/Release-Motion/build-remote/team-nust-robocup/src/TNRSModules/VisionModule/cmake_install.cmake")
  include("/home/muptii/Documents/robocup/team-nust-robocup-v2/build-V5/Release-Motion/build-remote/team-nust-robocup/src/TNRSModules/LocalizationModule/cmake_install.cmake")
  include("/home/muptii/Documents/robocup/team-nust-robocup-v2/build-V5/Release-Motion/build-remote/team-nust-robocup/src/TeamNUSTSPL/cmake_install.cmake")
  include("/home/muptii/Documents/robocup/team-nust-robocup-v2/build-V5/Release-Motion/build-remote/team-nust-robocup/src/GameController/src/libgamectrl/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/muptii/Documents/robocup/team-nust-robocup-v2/build-V5/Release-Motion/build-remote/team-nust-robocup/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
