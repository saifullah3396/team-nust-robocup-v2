# Install script for directory: /home/muptii/Documents/robocup/team-nust-robocup-v2/resources/fftw3

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
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/qi" TYPE FILE FILES "/home/muptii/Documents/robocup/team-nust-robocup-v2/build-V5/Debug-Motion/build-remote/team-nust-robocup/path.conf")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "runtime")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/qi" TYPE DIRECTORY FILES "" USE_SOURCE_PERMISSIONS REGEX "/[^/]*\\.pyc$" EXCLUDE REGEX "/\\_\\_pycache\\_\\_$" EXCLUDE)
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "runtime")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/qi" TYPE FILE FILES "/home/muptii/Documents/robocup/team-nust-robocup-v2/build-V5/Debug-Motion/build-remote/team-nust-robocup/path.conf")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/usr/lib" TYPE SHARED_LIBRARY FILES
    "/home/muptii/Documents/robocup/team-nust-robocup-v2/build-V5/Debug-Motion/build-remote/team-nust-robocup/resources/fftw3/CMakeFiles/CMakeRelink.dir/libfftw3f.so.3"
    "/home/muptii/Documents/robocup/team-nust-robocup-v2/build-V5/Debug-Motion/build-remote/team-nust-robocup/resources/fftw3/CMakeFiles/CMakeRelink.dir/libfftw3f.so.3.5.7"
    "/home/muptii/Documents/robocup/team-nust-robocup-v2/build-V5/Debug-Motion/build-remote/team-nust-robocup/resources/fftw3/CMakeFiles/CMakeRelink.dir/libfftw3f.so"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/usr/lib" TYPE SHARED_LIBRARY FILES
    "/home/muptii/Documents/robocup/team-nust-robocup-v2/build-V5/Debug-Motion/build-remote/team-nust-robocup/resources/fftw3/CMakeFiles/CMakeRelink.dir/libfftw3f_threads.so.3"
    "/home/muptii/Documents/robocup/team-nust-robocup-v2/build-V5/Debug-Motion/build-remote/team-nust-robocup/resources/fftw3/CMakeFiles/CMakeRelink.dir/libfftw3f_threads.so.3.5.7"
    "/home/muptii/Documents/robocup/team-nust-robocup-v2/build-V5/Debug-Motion/build-remote/team-nust-robocup/resources/fftw3/CMakeFiles/CMakeRelink.dir/libfftw3f_threads.so"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/usr/lib" TYPE SHARED_LIBRARY FILES
    "/home/muptii/Documents/robocup/team-nust-robocup-v2/build-V5/Debug-Motion/build-remote/team-nust-robocup/resources/fftw3/CMakeFiles/CMakeRelink.dir/libfftw3f.so.3"
    "/home/muptii/Documents/robocup/team-nust-robocup-v2/build-V5/Debug-Motion/build-remote/team-nust-robocup/resources/fftw3/CMakeFiles/CMakeRelink.dir/libfftw3f.so.3.5.7"
    "/home/muptii/Documents/robocup/team-nust-robocup-v2/build-V5/Debug-Motion/build-remote/team-nust-robocup/resources/fftw3/CMakeFiles/CMakeRelink.dir/libfftw3f.so"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/usr/include" TYPE FILE FILES "/home/muptii/Documents/robocup/team-nust-robocup-v2/resources/fftw3/api/fftw3.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Development")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/usr/lib/pkgconfig" TYPE FILE FILES "/home/muptii/Documents/robocup/team-nust-robocup-v2/build-V5/Debug-Motion/build-remote/team-nust-robocup/resources/fftw3/fftwf.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Development")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/usr/lib/cmake/fftw3f" TYPE FILE FILES
    "/home/muptii/Documents/robocup/team-nust-robocup-v2/build-V5/Debug-Motion/build-remote/team-nust-robocup/resources/fftw3/FFTW3fConfig.cmake"
    "/home/muptii/Documents/robocup/team-nust-robocup-v2/build-V5/Debug-Motion/build-remote/team-nust-robocup/resources/fftw3/FFTW3fConfigVersion.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Development")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/usr/lib/cmake/fftw3f/FFTW3LibraryDepends.cmake")
    file(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/usr/lib/cmake/fftw3f/FFTW3LibraryDepends.cmake"
         "/home/muptii/Documents/robocup/team-nust-robocup-v2/build-V5/Debug-Motion/build-remote/team-nust-robocup/resources/fftw3/CMakeFiles/Export/usr/lib/cmake/fftw3f/FFTW3LibraryDepends.cmake")
    if(EXPORT_FILE_CHANGED)
      file(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/usr/lib/cmake/fftw3f/FFTW3LibraryDepends-*.cmake")
      if(OLD_CONFIG_FILES)
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/usr/lib/cmake/fftw3f/FFTW3LibraryDepends.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        file(REMOVE ${OLD_CONFIG_FILES})
      endif()
    endif()
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/usr/lib/cmake/fftw3f" TYPE FILE FILES "/home/muptii/Documents/robocup/team-nust-robocup-v2/build-V5/Debug-Motion/build-remote/team-nust-robocup/resources/fftw3/CMakeFiles/Export/usr/lib/cmake/fftw3f/FFTW3LibraryDepends.cmake")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/usr/lib/cmake/fftw3f" TYPE FILE FILES "/home/muptii/Documents/robocup/team-nust-robocup-v2/build-V5/Debug-Motion/build-remote/team-nust-robocup/resources/fftw3/CMakeFiles/Export/usr/lib/cmake/fftw3f/FFTW3LibraryDepends-debug.cmake")
  endif()
endif()

