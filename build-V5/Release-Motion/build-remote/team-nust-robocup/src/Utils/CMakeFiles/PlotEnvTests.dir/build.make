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
CMAKE_SOURCE_DIR = /home/umaidzz/Documents/team-nust/team-nust-robocup-v2

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/umaidzz/Documents/team-nust/team-nust-robocup-v2/build-V5/Release-Motion/build-remote/team-nust-robocup

# Include any dependencies generated for this target.
include src/Utils/CMakeFiles/PlotEnvTests.dir/depend.make

# Include the progress variables for this target.
include src/Utils/CMakeFiles/PlotEnvTests.dir/progress.make

# Include the compile flags for this target's objects.
include src/Utils/CMakeFiles/PlotEnvTests.dir/flags.make

src/Utils/CMakeFiles/PlotEnvTests.dir/tests/PlotEnvTests.cpp.o: src/Utils/CMakeFiles/PlotEnvTests.dir/flags.make
src/Utils/CMakeFiles/PlotEnvTests.dir/tests/PlotEnvTests.cpp.o: ../../../../src/Utils/tests/PlotEnvTests.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/umaidzz/Documents/team-nust/team-nust-robocup-v2/build-V5/Release-Motion/build-remote/team-nust-robocup/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/Utils/CMakeFiles/PlotEnvTests.dir/tests/PlotEnvTests.cpp.o"
	cd /home/umaidzz/Documents/team-nust/team-nust-robocup-v2/build-V5/Release-Motion/build-remote/team-nust-robocup/src/Utils && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/PlotEnvTests.dir/tests/PlotEnvTests.cpp.o -c /home/umaidzz/Documents/team-nust/team-nust-robocup-v2/src/Utils/tests/PlotEnvTests.cpp

src/Utils/CMakeFiles/PlotEnvTests.dir/tests/PlotEnvTests.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/PlotEnvTests.dir/tests/PlotEnvTests.cpp.i"
	cd /home/umaidzz/Documents/team-nust/team-nust-robocup-v2/build-V5/Release-Motion/build-remote/team-nust-robocup/src/Utils && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/umaidzz/Documents/team-nust/team-nust-robocup-v2/src/Utils/tests/PlotEnvTests.cpp > CMakeFiles/PlotEnvTests.dir/tests/PlotEnvTests.cpp.i

src/Utils/CMakeFiles/PlotEnvTests.dir/tests/PlotEnvTests.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/PlotEnvTests.dir/tests/PlotEnvTests.cpp.s"
	cd /home/umaidzz/Documents/team-nust/team-nust-robocup-v2/build-V5/Release-Motion/build-remote/team-nust-robocup/src/Utils && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/umaidzz/Documents/team-nust/team-nust-robocup-v2/src/Utils/tests/PlotEnvTests.cpp -o CMakeFiles/PlotEnvTests.dir/tests/PlotEnvTests.cpp.s

src/Utils/CMakeFiles/PlotEnvTests.dir/tests/PlotEnvTests.cpp.o.requires:

.PHONY : src/Utils/CMakeFiles/PlotEnvTests.dir/tests/PlotEnvTests.cpp.o.requires

src/Utils/CMakeFiles/PlotEnvTests.dir/tests/PlotEnvTests.cpp.o.provides: src/Utils/CMakeFiles/PlotEnvTests.dir/tests/PlotEnvTests.cpp.o.requires
	$(MAKE) -f src/Utils/CMakeFiles/PlotEnvTests.dir/build.make src/Utils/CMakeFiles/PlotEnvTests.dir/tests/PlotEnvTests.cpp.o.provides.build
.PHONY : src/Utils/CMakeFiles/PlotEnvTests.dir/tests/PlotEnvTests.cpp.o.provides

src/Utils/CMakeFiles/PlotEnvTests.dir/tests/PlotEnvTests.cpp.o.provides.build: src/Utils/CMakeFiles/PlotEnvTests.dir/tests/PlotEnvTests.cpp.o


# Object files for target PlotEnvTests
PlotEnvTests_OBJECTS = \
"CMakeFiles/PlotEnvTests.dir/tests/PlotEnvTests.cpp.o"

# External object files for target PlotEnvTests
PlotEnvTests_EXTERNAL_OBJECTS =

../../remote/bin/PlotEnvTests: src/Utils/CMakeFiles/PlotEnvTests.dir/tests/PlotEnvTests.cpp.o
../../remote/bin/PlotEnvTests: src/Utils/CMakeFiles/PlotEnvTests.dir/build.make
../../remote/bin/PlotEnvTests: ../../remote/lib/libtnrs-utils.so
../../remote/bin/PlotEnvTests: /home/umaidzz/Documents/team-nust/naoqi-sdk-2.1.4.13-linux64/lib/libalcommon.so
../../remote/bin/PlotEnvTests: /home/umaidzz/Documents/team-nust/naoqi-sdk-2.1.4.13-linux64/lib/libalvalue.so
../../remote/bin/PlotEnvTests: /home/umaidzz/Documents/team-nust/naoqi-sdk-2.1.4.13-linux64/lib/libalerror.so
../../remote/bin/PlotEnvTests: /home/umaidzz/Documents/team-nust/naoqi-sdk-2.1.4.13-linux64/lib/libqimessaging.so
../../remote/bin/PlotEnvTests: /home/umaidzz/Documents/team-nust/naoqi-sdk-2.1.4.13-linux64/lib/libqitype.so
../../remote/bin/PlotEnvTests: /home/umaidzz/Documents/team-nust/naoqi-sdk-2.1.4.13-linux64/lib/libboost_date_time.so
../../remote/bin/PlotEnvTests: /home/umaidzz/Documents/team-nust/naoqi-sdk-2.1.4.13-linux64/lib/libboost_signals.so
../../remote/bin/PlotEnvTests: /home/umaidzz/Documents/team-nust/naoqi-sdk-2.1.4.13-linux64/lib/librttools.so
../../remote/bin/PlotEnvTests: /home/umaidzz/Documents/team-nust/naoqi-sdk-2.1.4.13-linux64/lib/libqi.so
../../remote/bin/PlotEnvTests: /home/umaidzz/Documents/team-nust/naoqi-sdk-2.1.4.13-linux64/lib/libboost_chrono.so
../../remote/bin/PlotEnvTests: /home/umaidzz/Documents/team-nust/naoqi-sdk-2.1.4.13-linux64/lib/libboost_program_options.so
../../remote/bin/PlotEnvTests: /home/umaidzz/Documents/team-nust/naoqi-sdk-2.1.4.13-linux64/lib/libboost_regex.so
../../remote/bin/PlotEnvTests: /home/umaidzz/Documents/team-nust/naoqi-sdk-2.1.4.13-linux64/lib/libboost_locale.so
../../remote/bin/PlotEnvTests: /home/umaidzz/Documents/team-nust/naoqi-sdk-2.1.4.13-linux64/lib/libboost_thread.so
../../remote/bin/PlotEnvTests: /usr/lib/x86_64-linux-gnu/librt.so
../../remote/bin/PlotEnvTests: /usr/lib/x86_64-linux-gnu/libdl.so
../../remote/bin/PlotEnvTests: /home/umaidzz/Documents/team-nust/naoqi-sdk-2.1.4.13-linux64/lib/libboost_filesystem.so
../../remote/bin/PlotEnvTests: /home/umaidzz/Documents/team-nust/naoqi-sdk-2.1.4.13-linux64/lib/libboost_iostreams.so
../../remote/bin/PlotEnvTests: /home/umaidzz/Documents/team-nust/naoqi-sdk-2.1.4.13-linux64/lib/libboost_system.so
../../remote/bin/PlotEnvTests: /home/umaidzz/Documents/team-nust/naoqi-sdk-2.1.4.13-linux64/lib/libopencv_highgui.so
../../remote/bin/PlotEnvTests: /home/umaidzz/Documents/team-nust/naoqi-sdk-2.1.4.13-linux64/lib/libopencv_imgproc.so
../../remote/bin/PlotEnvTests: /home/umaidzz/Documents/team-nust/naoqi-sdk-2.1.4.13-linux64/lib/libopencv_core.so
../../remote/bin/PlotEnvTests: /home/umaidzz/Documents/team-nust/naoqi-sdk-2.1.4.13-linux64/lib/libz.so
../../remote/bin/PlotEnvTests: /usr/lib/x86_64-linux-gnu/libv4l1.so
../../remote/bin/PlotEnvTests: /usr/lib/x86_64-linux-gnu/libv4l2.so
../../remote/bin/PlotEnvTests: /usr/lib/x86_64-linux-gnu/libv4lconvert.so
../../remote/bin/PlotEnvTests: /usr/lib/x86_64-linux-gnu/libjpeg.so
../../remote/bin/PlotEnvTests: src/Utils/CMakeFiles/PlotEnvTests.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/umaidzz/Documents/team-nust/team-nust-robocup-v2/build-V5/Release-Motion/build-remote/team-nust-robocup/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../../../../remote/bin/PlotEnvTests"
	cd /home/umaidzz/Documents/team-nust/team-nust-robocup-v2/build-V5/Release-Motion/build-remote/team-nust-robocup/src/Utils && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/PlotEnvTests.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/Utils/CMakeFiles/PlotEnvTests.dir/build: ../../remote/bin/PlotEnvTests

.PHONY : src/Utils/CMakeFiles/PlotEnvTests.dir/build

src/Utils/CMakeFiles/PlotEnvTests.dir/requires: src/Utils/CMakeFiles/PlotEnvTests.dir/tests/PlotEnvTests.cpp.o.requires

.PHONY : src/Utils/CMakeFiles/PlotEnvTests.dir/requires

src/Utils/CMakeFiles/PlotEnvTests.dir/clean:
	cd /home/umaidzz/Documents/team-nust/team-nust-robocup-v2/build-V5/Release-Motion/build-remote/team-nust-robocup/src/Utils && $(CMAKE_COMMAND) -P CMakeFiles/PlotEnvTests.dir/cmake_clean.cmake
.PHONY : src/Utils/CMakeFiles/PlotEnvTests.dir/clean

src/Utils/CMakeFiles/PlotEnvTests.dir/depend:
	cd /home/umaidzz/Documents/team-nust/team-nust-robocup-v2/build-V5/Release-Motion/build-remote/team-nust-robocup && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/umaidzz/Documents/team-nust/team-nust-robocup-v2 /home/umaidzz/Documents/team-nust/team-nust-robocup-v2/src/Utils /home/umaidzz/Documents/team-nust/team-nust-robocup-v2/build-V5/Release-Motion/build-remote/team-nust-robocup /home/umaidzz/Documents/team-nust/team-nust-robocup-v2/build-V5/Release-Motion/build-remote/team-nust-robocup/src/Utils /home/umaidzz/Documents/team-nust/team-nust-robocup-v2/build-V5/Release-Motion/build-remote/team-nust-robocup/src/Utils/CMakeFiles/PlotEnvTests.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/Utils/CMakeFiles/PlotEnvTests.dir/depend
