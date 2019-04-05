# Install script for directory: /home/umaidzz/Documents/team-nust/team-nust-robocup-v2/resources/nlopt

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/qi" TYPE FILE FILES "/home/umaidzz/Documents/team-nust/team-nust-robocup-v2/build-V5/Release-Motion/build-local/team-nust-robocup/path.conf")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "runtime")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/qi" TYPE DIRECTORY FILES "" USE_SOURCE_PERMISSIONS REGEX "/[^/]*\\.pyc$" EXCLUDE REGEX "/\\_\\_pycache\\_\\_$" EXCLUDE)
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "runtime")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/qi" TYPE FILE FILES "/home/umaidzz/Documents/team-nust/team-nust-robocup-v2/build-V5/Release-Motion/build-local/team-nust-robocup/path.conf")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/usr/include" TYPE FILE FILES
    "/home/umaidzz/Documents/team-nust/team-nust-robocup-v2/build-V5/Release-Motion/build-local/team-nust-robocup/resources/nlopt/api/nlopt.h"
    "/home/umaidzz/Documents/team-nust/team-nust-robocup-v2/build-V5/Release-Motion/build-local/team-nust-robocup/resources/nlopt/api/nlopt.hpp"
    "/home/umaidzz/Documents/team-nust/team-nust-robocup-v2/build-V5/Release-Motion/build-local/team-nust-robocup/resources/nlopt/api/nlopt.f"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/usr/lib" TYPE SHARED_LIBRARY FILES
    "/home/umaidzz/Documents/team-nust/team-nust-robocup-v2/build-V5/Release-Motion/cross/lib/libnlopt.so.0.9.0"
    "/home/umaidzz/Documents/team-nust/team-nust-robocup-v2/build-V5/Release-Motion/cross/lib/libnlopt.so.0"
    "/home/umaidzz/Documents/team-nust/team-nust-robocup-v2/build-V5/Release-Motion/cross/lib/libnlopt.so"
    )
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/usr/lib/libnlopt.so.0.9.0"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/usr/lib/libnlopt.so.0"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/usr/lib/libnlopt.so"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      if(CMAKE_INSTALL_DO_STRIP)
        execute_process(COMMAND "/home/umaidzz/Documents/team-nust/ctc-linux32-atom-2.1.4.13/cross/bin/i686-aldebaran-linux-gnu-strip" "${file}")
      endif()
    endif()
  endforeach()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Development")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/usr/lib/cmake/nlopt/NLoptLibraryDepends.cmake")
    file(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/usr/lib/cmake/nlopt/NLoptLibraryDepends.cmake"
         "/home/umaidzz/Documents/team-nust/team-nust-robocup-v2/build-V5/Release-Motion/build-local/team-nust-robocup/resources/nlopt/CMakeFiles/Export/usr/lib/cmake/nlopt/NLoptLibraryDepends.cmake")
    if(EXPORT_FILE_CHANGED)
      file(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/usr/lib/cmake/nlopt/NLoptLibraryDepends-*.cmake")
      if(OLD_CONFIG_FILES)
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/usr/lib/cmake/nlopt/NLoptLibraryDepends.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        file(REMOVE ${OLD_CONFIG_FILES})
      endif()
    endif()
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/usr/lib/cmake/nlopt" TYPE FILE FILES "/home/umaidzz/Documents/team-nust/team-nust-robocup-v2/build-V5/Release-Motion/build-local/team-nust-robocup/resources/nlopt/CMakeFiles/Export/usr/lib/cmake/nlopt/NLoptLibraryDepends.cmake")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/usr/lib/cmake/nlopt" TYPE FILE FILES "/home/umaidzz/Documents/team-nust/team-nust-robocup-v2/build-V5/Release-Motion/build-local/team-nust-robocup/resources/nlopt/CMakeFiles/Export/usr/lib/cmake/nlopt/NLoptLibraryDepends-release.cmake")
  endif()
endif()

