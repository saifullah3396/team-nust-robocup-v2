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
CMAKE_BINARY_DIR = /home/muptii/Documents/robocup/team-nust-robocup-v2/build-V5/Debug-Motion/build-remote/team-nust-robocup

# Include any dependencies generated for this target.
include src/Utils/CMakeFiles/DataHoldersTest.dir/depend.make

# Include the progress variables for this target.
include src/Utils/CMakeFiles/DataHoldersTest.dir/progress.make

# Include the compile flags for this target's objects.
include src/Utils/CMakeFiles/DataHoldersTest.dir/flags.make

src/Utils/CMakeFiles/DataHoldersTest.dir/tests/DataHoldersTest.cpp.o: src/Utils/CMakeFiles/DataHoldersTest.dir/flags.make
src/Utils/CMakeFiles/DataHoldersTest.dir/tests/DataHoldersTest.cpp.o: ../../../../src/Utils/tests/DataHoldersTest.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/muptii/Documents/robocup/team-nust-robocup-v2/build-V5/Debug-Motion/build-remote/team-nust-robocup/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/Utils/CMakeFiles/DataHoldersTest.dir/tests/DataHoldersTest.cpp.o"
	cd /home/muptii/Documents/robocup/team-nust-robocup-v2/build-V5/Debug-Motion/build-remote/team-nust-robocup/src/Utils && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/DataHoldersTest.dir/tests/DataHoldersTest.cpp.o -c /home/muptii/Documents/robocup/team-nust-robocup-v2/src/Utils/tests/DataHoldersTest.cpp

src/Utils/CMakeFiles/DataHoldersTest.dir/tests/DataHoldersTest.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/DataHoldersTest.dir/tests/DataHoldersTest.cpp.i"
	cd /home/muptii/Documents/robocup/team-nust-robocup-v2/build-V5/Debug-Motion/build-remote/team-nust-robocup/src/Utils && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/muptii/Documents/robocup/team-nust-robocup-v2/src/Utils/tests/DataHoldersTest.cpp > CMakeFiles/DataHoldersTest.dir/tests/DataHoldersTest.cpp.i

src/Utils/CMakeFiles/DataHoldersTest.dir/tests/DataHoldersTest.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/DataHoldersTest.dir/tests/DataHoldersTest.cpp.s"
	cd /home/muptii/Documents/robocup/team-nust-robocup-v2/build-V5/Debug-Motion/build-remote/team-nust-robocup/src/Utils && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/muptii/Documents/robocup/team-nust-robocup-v2/src/Utils/tests/DataHoldersTest.cpp -o CMakeFiles/DataHoldersTest.dir/tests/DataHoldersTest.cpp.s

src/Utils/CMakeFiles/DataHoldersTest.dir/tests/DataHoldersTest.cpp.o.requires:

.PHONY : src/Utils/CMakeFiles/DataHoldersTest.dir/tests/DataHoldersTest.cpp.o.requires

src/Utils/CMakeFiles/DataHoldersTest.dir/tests/DataHoldersTest.cpp.o.provides: src/Utils/CMakeFiles/DataHoldersTest.dir/tests/DataHoldersTest.cpp.o.requires
	$(MAKE) -f src/Utils/CMakeFiles/DataHoldersTest.dir/build.make src/Utils/CMakeFiles/DataHoldersTest.dir/tests/DataHoldersTest.cpp.o.provides.build
.PHONY : src/Utils/CMakeFiles/DataHoldersTest.dir/tests/DataHoldersTest.cpp.o.provides

src/Utils/CMakeFiles/DataHoldersTest.dir/tests/DataHoldersTest.cpp.o.provides.build: src/Utils/CMakeFiles/DataHoldersTest.dir/tests/DataHoldersTest.cpp.o


# Object files for target DataHoldersTest
DataHoldersTest_OBJECTS = \
"CMakeFiles/DataHoldersTest.dir/tests/DataHoldersTest.cpp.o"

# External object files for target DataHoldersTest
DataHoldersTest_EXTERNAL_OBJECTS =

../../remote/bin/DataHoldersTest: src/Utils/CMakeFiles/DataHoldersTest.dir/tests/DataHoldersTest.cpp.o
../../remote/bin/DataHoldersTest: src/Utils/CMakeFiles/DataHoldersTest.dir/build.make
../../remote/bin/DataHoldersTest: ../../remote/lib/libtnrs-utils.so
../../remote/bin/DataHoldersTest: /home/muptii/Documents/robocup/naoqi-sdk-2.1.4.13-linux64/lib/libalcommon.so
../../remote/bin/DataHoldersTest: /home/muptii/Documents/robocup/naoqi-sdk-2.1.4.13-linux64/lib/libalvalue.so
../../remote/bin/DataHoldersTest: /home/muptii/Documents/robocup/naoqi-sdk-2.1.4.13-linux64/lib/libalerror.so
../../remote/bin/DataHoldersTest: /home/muptii/Documents/robocup/naoqi-sdk-2.1.4.13-linux64/lib/libqimessaging.so
../../remote/bin/DataHoldersTest: /home/muptii/Documents/robocup/naoqi-sdk-2.1.4.13-linux64/lib/libqitype.so
../../remote/bin/DataHoldersTest: /home/muptii/Documents/robocup/naoqi-sdk-2.1.4.13-linux64/lib/libboost_date_time.so
../../remote/bin/DataHoldersTest: /home/muptii/Documents/robocup/naoqi-sdk-2.1.4.13-linux64/lib/libboost_signals.so
../../remote/bin/DataHoldersTest: /home/muptii/Documents/robocup/naoqi-sdk-2.1.4.13-linux64/lib/librttools.so
../../remote/bin/DataHoldersTest: /home/muptii/Documents/robocup/naoqi-sdk-2.1.4.13-linux64/lib/libqi.so
../../remote/bin/DataHoldersTest: /home/muptii/Documents/robocup/naoqi-sdk-2.1.4.13-linux64/lib/libboost_chrono.so
../../remote/bin/DataHoldersTest: /home/muptii/Documents/robocup/naoqi-sdk-2.1.4.13-linux64/lib/libboost_program_options.so
../../remote/bin/DataHoldersTest: /home/muptii/Documents/robocup/naoqi-sdk-2.1.4.13-linux64/lib/libboost_regex.so
../../remote/bin/DataHoldersTest: /home/muptii/Documents/robocup/naoqi-sdk-2.1.4.13-linux64/lib/libboost_locale.so
../../remote/bin/DataHoldersTest: /home/muptii/Documents/robocup/naoqi-sdk-2.1.4.13-linux64/lib/libboost_thread.so
../../remote/bin/DataHoldersTest: /usr/lib/x86_64-linux-gnu/librt.so
../../remote/bin/DataHoldersTest: /usr/lib/x86_64-linux-gnu/libdl.so
../../remote/bin/DataHoldersTest: /home/muptii/Documents/robocup/naoqi-sdk-2.1.4.13-linux64/lib/libboost_filesystem.so
../../remote/bin/DataHoldersTest: /home/muptii/Documents/robocup/naoqi-sdk-2.1.4.13-linux64/lib/libboost_iostreams.so
../../remote/bin/DataHoldersTest: /home/muptii/Documents/robocup/naoqi-sdk-2.1.4.13-linux64/lib/libboost_system.so
../../remote/bin/DataHoldersTest: /home/muptii/Documents/robocup/naoqi-sdk-2.1.4.13-linux64/lib/libopencv_highgui.so
../../remote/bin/DataHoldersTest: /home/muptii/Documents/robocup/naoqi-sdk-2.1.4.13-linux64/lib/libopencv_imgproc.so
../../remote/bin/DataHoldersTest: /home/muptii/Documents/robocup/naoqi-sdk-2.1.4.13-linux64/lib/libopencv_core.so
../../remote/bin/DataHoldersTest: /home/muptii/Documents/robocup/naoqi-sdk-2.1.4.13-linux64/lib/libz.so
../../remote/bin/DataHoldersTest: /usr/lib/x86_64-linux-gnu/libv4l1.so
../../remote/bin/DataHoldersTest: /usr/lib/x86_64-linux-gnu/libv4l2.so
../../remote/bin/DataHoldersTest: /usr/lib/x86_64-linux-gnu/libv4lconvert.so
../../remote/bin/DataHoldersTest: /usr/lib/x86_64-linux-gnu/libjpeg.so
../../remote/bin/DataHoldersTest: src/Utils/CMakeFiles/DataHoldersTest.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/muptii/Documents/robocup/team-nust-robocup-v2/build-V5/Debug-Motion/build-remote/team-nust-robocup/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../../../../remote/bin/DataHoldersTest"
	cd /home/muptii/Documents/robocup/team-nust-robocup-v2/build-V5/Debug-Motion/build-remote/team-nust-robocup/src/Utils && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/DataHoldersTest.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/Utils/CMakeFiles/DataHoldersTest.dir/build: ../../remote/bin/DataHoldersTest

.PHONY : src/Utils/CMakeFiles/DataHoldersTest.dir/build

src/Utils/CMakeFiles/DataHoldersTest.dir/requires: src/Utils/CMakeFiles/DataHoldersTest.dir/tests/DataHoldersTest.cpp.o.requires

.PHONY : src/Utils/CMakeFiles/DataHoldersTest.dir/requires

src/Utils/CMakeFiles/DataHoldersTest.dir/clean:
	cd /home/muptii/Documents/robocup/team-nust-robocup-v2/build-V5/Debug-Motion/build-remote/team-nust-robocup/src/Utils && $(CMAKE_COMMAND) -P CMakeFiles/DataHoldersTest.dir/cmake_clean.cmake
.PHONY : src/Utils/CMakeFiles/DataHoldersTest.dir/clean

src/Utils/CMakeFiles/DataHoldersTest.dir/depend:
	cd /home/muptii/Documents/robocup/team-nust-robocup-v2/build-V5/Debug-Motion/build-remote/team-nust-robocup && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/muptii/Documents/robocup/team-nust-robocup-v2 /home/muptii/Documents/robocup/team-nust-robocup-v2/src/Utils /home/muptii/Documents/robocup/team-nust-robocup-v2/build-V5/Debug-Motion/build-remote/team-nust-robocup /home/muptii/Documents/robocup/team-nust-robocup-v2/build-V5/Debug-Motion/build-remote/team-nust-robocup/src/Utils /home/muptii/Documents/robocup/team-nust-robocup-v2/build-V5/Debug-Motion/build-remote/team-nust-robocup/src/Utils/CMakeFiles/DataHoldersTest.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/Utils/CMakeFiles/DataHoldersTest.dir/depend

