# Install script for directory: /home/umaidzz/Documents/team-nust/team-nust-robocup-v2/resources/jsoncpp/src/lib_json

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

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/usr/lib" TYPE SHARED_LIBRARY FILES
    "/home/umaidzz/Documents/team-nust/team-nust-robocup-v2/build-V5/Release-Motion/cross/lib/libjsoncpp.so.1.8.4"
    "/home/umaidzz/Documents/team-nust/team-nust-robocup-v2/build-V5/Release-Motion/cross/lib/libjsoncpp.so.19"
    "/home/umaidzz/Documents/team-nust/team-nust-robocup-v2/build-V5/Release-Motion/cross/lib/libjsoncpp.so"
    )
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/usr/lib/libjsoncpp.so.1.8.4"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/usr/lib/libjsoncpp.so.19"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/usr/lib/libjsoncpp.so"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      if(CMAKE_INSTALL_DO_STRIP)
        execute_process(COMMAND "/home/umaidzz/Documents/team-nust/ctc-linux32-atom-2.1.4.13/cross/bin/i686-aldebaran-linux-gnu-strip" "${file}")
      endif()
    endif()
  endforeach()
endif()

