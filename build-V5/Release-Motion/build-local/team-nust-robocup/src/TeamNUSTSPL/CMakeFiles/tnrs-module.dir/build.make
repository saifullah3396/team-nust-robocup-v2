# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Produce verbose output by default.
VERBOSE = 1

# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/muptii/Documents/robocup/team-nust-robocup-v2

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/muptii/Documents/robocup/team-nust-robocup-v2/build-V5/Release-Motion/build-local/team-nust-robocup

# Include any dependencies generated for this target.
include src/TeamNUSTSPL/CMakeFiles/tnrs-module.dir/depend.make

# Include the progress variables for this target.
include src/TeamNUSTSPL/CMakeFiles/tnrs-module.dir/progress.make

# Include the compile flags for this target's objects.
include src/TeamNUSTSPL/CMakeFiles/tnrs-module.dir/flags.make

src/TeamNUSTSPL/CMakeFiles/tnrs-module.dir/src/main.cpp.o: src/TeamNUSTSPL/CMakeFiles/tnrs-module.dir/flags.make
src/TeamNUSTSPL/CMakeFiles/tnrs-module.dir/src/main.cpp.o: ../../../../src/TeamNUSTSPL/src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/muptii/Documents/robocup/team-nust-robocup-v2/build-V5/Release-Motion/build-local/team-nust-robocup/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/TeamNUSTSPL/CMakeFiles/tnrs-module.dir/src/main.cpp.o"
	cd /home/muptii/Documents/robocup/team-nust-robocup-v2/build-V5/Release-Motion/build-local/team-nust-robocup/src/TeamNUSTSPL && /home/muptii/Documents/robocup/ctc-linux32-atom-2.1.4.13/cross/i686-aldebaran-linux-gnu/sysroot/usr/local/bin/g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/tnrs-module.dir/src/main.cpp.o -c /home/muptii/Documents/robocup/team-nust-robocup-v2/src/TeamNUSTSPL/src/main.cpp

src/TeamNUSTSPL/CMakeFiles/tnrs-module.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/tnrs-module.dir/src/main.cpp.i"
	cd /home/muptii/Documents/robocup/team-nust-robocup-v2/build-V5/Release-Motion/build-local/team-nust-robocup/src/TeamNUSTSPL && /home/muptii/Documents/robocup/ctc-linux32-atom-2.1.4.13/cross/i686-aldebaran-linux-gnu/sysroot/usr/local/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/muptii/Documents/robocup/team-nust-robocup-v2/src/TeamNUSTSPL/src/main.cpp > CMakeFiles/tnrs-module.dir/src/main.cpp.i

src/TeamNUSTSPL/CMakeFiles/tnrs-module.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/tnrs-module.dir/src/main.cpp.s"
	cd /home/muptii/Documents/robocup/team-nust-robocup-v2/build-V5/Release-Motion/build-local/team-nust-robocup/src/TeamNUSTSPL && /home/muptii/Documents/robocup/ctc-linux32-atom-2.1.4.13/cross/i686-aldebaran-linux-gnu/sysroot/usr/local/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/muptii/Documents/robocup/team-nust-robocup-v2/src/TeamNUSTSPL/src/main.cpp -o CMakeFiles/tnrs-module.dir/src/main.cpp.s

src/TeamNUSTSPL/CMakeFiles/tnrs-module.dir/src/main.cpp.o.requires:

.PHONY : src/TeamNUSTSPL/CMakeFiles/tnrs-module.dir/src/main.cpp.o.requires

src/TeamNUSTSPL/CMakeFiles/tnrs-module.dir/src/main.cpp.o.provides: src/TeamNUSTSPL/CMakeFiles/tnrs-module.dir/src/main.cpp.o.requires
	$(MAKE) -f src/TeamNUSTSPL/CMakeFiles/tnrs-module.dir/build.make src/TeamNUSTSPL/CMakeFiles/tnrs-module.dir/src/main.cpp.o.provides.build
.PHONY : src/TeamNUSTSPL/CMakeFiles/tnrs-module.dir/src/main.cpp.o.provides

src/TeamNUSTSPL/CMakeFiles/tnrs-module.dir/src/main.cpp.o.provides.build: src/TeamNUSTSPL/CMakeFiles/tnrs-module.dir/src/main.cpp.o


src/TeamNUSTSPL/CMakeFiles/tnrs-module.dir/src/TeamNUSTSPL.cpp.o: src/TeamNUSTSPL/CMakeFiles/tnrs-module.dir/flags.make
src/TeamNUSTSPL/CMakeFiles/tnrs-module.dir/src/TeamNUSTSPL.cpp.o: ../../../../src/TeamNUSTSPL/src/TeamNUSTSPL.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/muptii/Documents/robocup/team-nust-robocup-v2/build-V5/Release-Motion/build-local/team-nust-robocup/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object src/TeamNUSTSPL/CMakeFiles/tnrs-module.dir/src/TeamNUSTSPL.cpp.o"
	cd /home/muptii/Documents/robocup/team-nust-robocup-v2/build-V5/Release-Motion/build-local/team-nust-robocup/src/TeamNUSTSPL && /home/muptii/Documents/robocup/ctc-linux32-atom-2.1.4.13/cross/i686-aldebaran-linux-gnu/sysroot/usr/local/bin/g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/tnrs-module.dir/src/TeamNUSTSPL.cpp.o -c /home/muptii/Documents/robocup/team-nust-robocup-v2/src/TeamNUSTSPL/src/TeamNUSTSPL.cpp

src/TeamNUSTSPL/CMakeFiles/tnrs-module.dir/src/TeamNUSTSPL.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/tnrs-module.dir/src/TeamNUSTSPL.cpp.i"
	cd /home/muptii/Documents/robocup/team-nust-robocup-v2/build-V5/Release-Motion/build-local/team-nust-robocup/src/TeamNUSTSPL && /home/muptii/Documents/robocup/ctc-linux32-atom-2.1.4.13/cross/i686-aldebaran-linux-gnu/sysroot/usr/local/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/muptii/Documents/robocup/team-nust-robocup-v2/src/TeamNUSTSPL/src/TeamNUSTSPL.cpp > CMakeFiles/tnrs-module.dir/src/TeamNUSTSPL.cpp.i

src/TeamNUSTSPL/CMakeFiles/tnrs-module.dir/src/TeamNUSTSPL.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/tnrs-module.dir/src/TeamNUSTSPL.cpp.s"
	cd /home/muptii/Documents/robocup/team-nust-robocup-v2/build-V5/Release-Motion/build-local/team-nust-robocup/src/TeamNUSTSPL && /home/muptii/Documents/robocup/ctc-linux32-atom-2.1.4.13/cross/i686-aldebaran-linux-gnu/sysroot/usr/local/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/muptii/Documents/robocup/team-nust-robocup-v2/src/TeamNUSTSPL/src/TeamNUSTSPL.cpp -o CMakeFiles/tnrs-module.dir/src/TeamNUSTSPL.cpp.s

src/TeamNUSTSPL/CMakeFiles/tnrs-module.dir/src/TeamNUSTSPL.cpp.o.requires:

.PHONY : src/TeamNUSTSPL/CMakeFiles/tnrs-module.dir/src/TeamNUSTSPL.cpp.o.requires

src/TeamNUSTSPL/CMakeFiles/tnrs-module.dir/src/TeamNUSTSPL.cpp.o.provides: src/TeamNUSTSPL/CMakeFiles/tnrs-module.dir/src/TeamNUSTSPL.cpp.o.requires
	$(MAKE) -f src/TeamNUSTSPL/CMakeFiles/tnrs-module.dir/build.make src/TeamNUSTSPL/CMakeFiles/tnrs-module.dir/src/TeamNUSTSPL.cpp.o.provides.build
.PHONY : src/TeamNUSTSPL/CMakeFiles/tnrs-module.dir/src/TeamNUSTSPL.cpp.o.provides

src/TeamNUSTSPL/CMakeFiles/tnrs-module.dir/src/TeamNUSTSPL.cpp.o.provides.build: src/TeamNUSTSPL/CMakeFiles/tnrs-module.dir/src/TeamNUSTSPL.cpp.o


# Object files for target tnrs-module
tnrs__module_OBJECTS = \
"CMakeFiles/tnrs-module.dir/src/main.cpp.o" \
"CMakeFiles/tnrs-module.dir/src/TeamNUSTSPL.cpp.o"

# External object files for target tnrs-module
tnrs__module_EXTERNAL_OBJECTS =

../../cross/lib/libtnrs-module.so: src/TeamNUSTSPL/CMakeFiles/tnrs-module.dir/src/main.cpp.o
../../cross/lib/libtnrs-module.so: src/TeamNUSTSPL/CMakeFiles/tnrs-module.dir/src/TeamNUSTSPL.cpp.o
../../cross/lib/libtnrs-module.so: src/TeamNUSTSPL/CMakeFiles/tnrs-module.dir/build.make
../../cross/lib/libtnrs-module.so: /home/muptii/Documents/robocup/ctc-linux32-atom-2.1.4.13/boost/lib/libboost_iostreams-mt-1_55.so
../../cross/lib/libtnrs-module.so: /home/muptii/Documents/robocup/ctc-linux32-atom-2.1.4.13/libnaoqi/lib/libalvision.so
../../cross/lib/libtnrs-module.so: /home/muptii/Documents/robocup/ctc-linux32-atom-2.1.4.13/libnaoqi/lib/libalextractor.so
../../cross/lib/libtnrs-module.so: /home/muptii/Documents/robocup/ctc-linux32-atom-2.1.4.13/libnaoqi/lib/libalthread.so
../../cross/lib/libtnrs-module.so: /home/muptii/Documents/robocup/ctc-linux32-atom-2.1.4.13/libnaoqi/lib/libalproxies.so
../../cross/lib/libtnrs-module.so: /home/muptii/Documents/robocup/ctc-linux32-atom-2.1.4.13/libnaoqi/lib/libalcommon.so
../../cross/lib/libtnrs-module.so: /home/muptii/Documents/robocup/ctc-linux32-atom-2.1.4.13/boost/lib/libboost_signals-mt-1_55.so
../../cross/lib/libtnrs-module.so: /home/muptii/Documents/robocup/ctc-linux32-atom-2.1.4.13/libnaoqi/lib/librttools.so
../../cross/lib/libtnrs-module.so: /home/muptii/Documents/robocup/ctc-linux32-atom-2.1.4.13/libnaoqi/lib/libalvalue.so
../../cross/lib/libtnrs-module.so: /home/muptii/Documents/robocup/ctc-linux32-atom-2.1.4.13/libnaoqi/lib/libalerror.so
../../cross/lib/libtnrs-module.so: /home/muptii/Documents/robocup/ctc-linux32-atom-2.1.4.13/libnaoqi/lib/libqimessaging.so
../../cross/lib/libtnrs-module.so: /home/muptii/Documents/robocup/ctc-linux32-atom-2.1.4.13/libnaoqi/lib/libqitype.so
../../cross/lib/libtnrs-module.so: /home/muptii/Documents/robocup/ctc-linux32-atom-2.1.4.13/libnaoqi/lib/libqi.so
../../cross/lib/libtnrs-module.so: /home/muptii/Documents/robocup/ctc-linux32-atom-2.1.4.13/boost/lib/libboost_chrono-mt-1_55.so
../../cross/lib/libtnrs-module.so: /home/muptii/Documents/robocup/ctc-linux32-atom-2.1.4.13/boost/lib/libboost_filesystem-mt-1_55.so
../../cross/lib/libtnrs-module.so: /home/muptii/Documents/robocup/ctc-linux32-atom-2.1.4.13/boost/lib/libboost_program_options-mt-1_55.so
../../cross/lib/libtnrs-module.so: /home/muptii/Documents/robocup/ctc-linux32-atom-2.1.4.13/boost/lib/libboost_regex-mt-1_55.so
../../cross/lib/libtnrs-module.so: /home/muptii/Documents/robocup/ctc-linux32-atom-2.1.4.13/boost/lib/libboost_date_time-mt-1_55.so
../../cross/lib/libtnrs-module.so: /home/muptii/Documents/robocup/ctc-linux32-atom-2.1.4.13/boost/lib/libboost_system-mt-1_55.so
../../cross/lib/libtnrs-module.so: /home/muptii/Documents/robocup/ctc-linux32-atom-2.1.4.13/boost/lib/libboost_locale-mt-1_55.so
../../cross/lib/libtnrs-module.so: /home/muptii/Documents/robocup/ctc-linux32-atom-2.1.4.13/boost/lib/libboost_thread-mt-1_55.so
../../cross/lib/libtnrs-module.so: /home/muptii/Documents/robocup/ctc-linux32-atom-2.1.4.13/libnaoqi/lib/libopencv_highgui.so
../../cross/lib/libtnrs-module.so: /home/muptii/Documents/robocup/ctc-linux32-atom-2.1.4.13/libnaoqi/lib/libopencv_imgproc.so
../../cross/lib/libtnrs-module.so: /home/muptii/Documents/robocup/ctc-linux32-atom-2.1.4.13/openni2/lib/libOpenNI2.so
../../cross/lib/libtnrs-module.so: /home/muptii/Documents/robocup/ctc-linux32-atom-2.1.4.13/udev/lib/libudev.so
../../cross/lib/libtnrs-module.so: /home/muptii/Documents/robocup/ctc-linux32-atom-2.1.4.13/usb_1/lib/libusb-1.0.so
../../cross/lib/libtnrs-module.so: /home/muptii/Documents/robocup/ctc-linux32-atom-2.1.4.13/usb_1/lib/libusb-1.0.so.0
../../cross/lib/libtnrs-module.so: /home/muptii/Documents/robocup/ctc-linux32-atom-2.1.4.13/tiff/lib/libtiff.so
../../cross/lib/libtnrs-module.so: /home/muptii/Documents/robocup/ctc-linux32-atom-2.1.4.13/jpeg/lib/libjpeg.so
../../cross/lib/libtnrs-module.so: /home/muptii/Documents/robocup/ctc-linux32-atom-2.1.4.13/ffmpeg/lib/libavcodec.so
../../cross/lib/libtnrs-module.so: /home/muptii/Documents/robocup/ctc-linux32-atom-2.1.4.13/ffmpeg/lib/libavformat.so
../../cross/lib/libtnrs-module.so: /home/muptii/Documents/robocup/ctc-linux32-atom-2.1.4.13/ffmpeg/lib/libavutil.so
../../cross/lib/libtnrs-module.so: /home/muptii/Documents/robocup/ctc-linux32-atom-2.1.4.13/ffmpeg/lib/libswscale.so
../../cross/lib/libtnrs-module.so: /home/muptii/Documents/robocup/ctc-linux32-atom-2.1.4.13/bzip2/lib/libbz2.so
../../cross/lib/libtnrs-module.so: /home/muptii/Documents/robocup/ctc-linux32-atom-2.1.4.13/v4l/lib/libv4l1.so
../../cross/lib/libtnrs-module.so: /home/muptii/Documents/robocup/ctc-linux32-atom-2.1.4.13/v4l/lib/libv4l2.so
../../cross/lib/libtnrs-module.so: /home/muptii/Documents/robocup/ctc-linux32-atom-2.1.4.13/v4l/lib/libv4lconvert.so
../../cross/lib/libtnrs-module.so: /home/muptii/Documents/robocup/ctc-linux32-atom-2.1.4.13/png/lib/libpng.so
../../cross/lib/libtnrs-module.so: /home/muptii/Documents/robocup/ctc-linux32-atom-2.1.4.13/gstreamer/lib/libgstreamer-0.10.so
../../cross/lib/libtnrs-module.so: /home/muptii/Documents/robocup/ctc-linux32-atom-2.1.4.13/gstreamer/lib/libgstbase-0.10.so
../../cross/lib/libtnrs-module.so: /home/muptii/Documents/robocup/ctc-linux32-atom-2.1.4.13/gstreamer/lib/libgstapp-0.10.so
../../cross/lib/libtnrs-module.so: /home/muptii/Documents/robocup/ctc-linux32-atom-2.1.4.13/gstreamer/lib/libgstvideo-0.10.so
../../cross/lib/libtnrs-module.so: /home/muptii/Documents/robocup/ctc-linux32-atom-2.1.4.13/glib2/lib/libgmodule-2.0.so
../../cross/lib/libtnrs-module.so: /home/muptii/Documents/robocup/ctc-linux32-atom-2.1.4.13/cross/i686-aldebaran-linux-gnu/sysroot/usr/lib/libdl.so
../../cross/lib/libtnrs-module.so: /home/muptii/Documents/robocup/ctc-linux32-atom-2.1.4.13/glib2/lib/libgobject-2.0.so
../../cross/lib/libtnrs-module.so: /home/muptii/Documents/robocup/ctc-linux32-atom-2.1.4.13/glib2/lib/libgthread-2.0.so
../../cross/lib/libtnrs-module.so: /home/muptii/Documents/robocup/ctc-linux32-atom-2.1.4.13/glib2/lib/libglib-2.0.so
../../cross/lib/libtnrs-module.so: /home/muptii/Documents/robocup/ctc-linux32-atom-2.1.4.13/cross/i686-aldebaran-linux-gnu/sysroot/usr/lib/libpthread.so
../../cross/lib/libtnrs-module.so: /home/muptii/Documents/robocup/ctc-linux32-atom-2.1.4.13/cross/i686-aldebaran-linux-gnu/sysroot/usr/lib/librt.so
../../cross/lib/libtnrs-module.so: /home/muptii/Documents/robocup/ctc-linux32-atom-2.1.4.13/ffi/lib/libffi.so
../../cross/lib/libtnrs-module.so: /home/muptii/Documents/robocup/ctc-linux32-atom-2.1.4.13/xml2/lib/libxml2.so
../../cross/lib/libtnrs-module.so: /home/muptii/Documents/robocup/ctc-linux32-atom-2.1.4.13/libnaoqi/lib/libopencv_calib3d.so
../../cross/lib/libtnrs-module.so: /home/muptii/Documents/robocup/ctc-linux32-atom-2.1.4.13/libnaoqi/lib/libopencv_objdetect.so
../../cross/lib/libtnrs-module.so: /home/muptii/Documents/robocup/ctc-linux32-atom-2.1.4.13/libnaoqi/lib/libopencv_features2d.so
../../cross/lib/libtnrs-module.so: /home/muptii/Documents/robocup/ctc-linux32-atom-2.1.4.13/libnaoqi/lib/libopencv_flann.so
../../cross/lib/libtnrs-module.so: /home/muptii/Documents/robocup/ctc-linux32-atom-2.1.4.13/libnaoqi/lib/libopencv_core.so
../../cross/lib/libtnrs-module.so: /home/muptii/Documents/robocup/ctc-linux32-atom-2.1.4.13/zlib/lib/libz.so
../../cross/lib/libtnrs-module.so: /home/muptii/Documents/robocup/ctc-linux32-atom-2.1.4.13/tbb/lib/libtbb.so
../../cross/lib/libtnrs-module.so: /home/muptii/Documents/robocup/ctc-linux32-atom-2.1.4.13/libnaoqi/lib/libopencv_video.so
../../cross/lib/libtnrs-module.so: src/TeamNUSTSPL/CMakeFiles/tnrs-module.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/muptii/Documents/robocup/team-nust-robocup-v2/build-V5/Release-Motion/build-local/team-nust-robocup/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX shared library ../../../../cross/lib/libtnrs-module.so"
	cd /home/muptii/Documents/robocup/team-nust-robocup-v2/build-V5/Release-Motion/build-local/team-nust-robocup/src/TeamNUSTSPL && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/tnrs-module.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/TeamNUSTSPL/CMakeFiles/tnrs-module.dir/build: ../../cross/lib/libtnrs-module.so

.PHONY : src/TeamNUSTSPL/CMakeFiles/tnrs-module.dir/build

src/TeamNUSTSPL/CMakeFiles/tnrs-module.dir/requires: src/TeamNUSTSPL/CMakeFiles/tnrs-module.dir/src/main.cpp.o.requires
src/TeamNUSTSPL/CMakeFiles/tnrs-module.dir/requires: src/TeamNUSTSPL/CMakeFiles/tnrs-module.dir/src/TeamNUSTSPL.cpp.o.requires

.PHONY : src/TeamNUSTSPL/CMakeFiles/tnrs-module.dir/requires

src/TeamNUSTSPL/CMakeFiles/tnrs-module.dir/clean:
	cd /home/muptii/Documents/robocup/team-nust-robocup-v2/build-V5/Release-Motion/build-local/team-nust-robocup/src/TeamNUSTSPL && $(CMAKE_COMMAND) -P CMakeFiles/tnrs-module.dir/cmake_clean.cmake
.PHONY : src/TeamNUSTSPL/CMakeFiles/tnrs-module.dir/clean

src/TeamNUSTSPL/CMakeFiles/tnrs-module.dir/depend:
	cd /home/muptii/Documents/robocup/team-nust-robocup-v2/build-V5/Release-Motion/build-local/team-nust-robocup && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/muptii/Documents/robocup/team-nust-robocup-v2 /home/muptii/Documents/robocup/team-nust-robocup-v2/src/TeamNUSTSPL /home/muptii/Documents/robocup/team-nust-robocup-v2/build-V5/Release-Motion/build-local/team-nust-robocup /home/muptii/Documents/robocup/team-nust-robocup-v2/build-V5/Release-Motion/build-local/team-nust-robocup/src/TeamNUSTSPL /home/muptii/Documents/robocup/team-nust-robocup-v2/build-V5/Release-Motion/build-local/team-nust-robocup/src/TeamNUSTSPL/CMakeFiles/tnrs-module.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/TeamNUSTSPL/CMakeFiles/tnrs-module.dir/depend

