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
CMAKE_BINARY_DIR = /home/umaidzz/Documents/team-nust/team-nust-robocup-v2/build-V5/Release-Motion/build-local/team-nust-robocup

# Include any dependencies generated for this target.
include resources/qpOASES/CMakeFiles/example4.dir/depend.make

# Include the progress variables for this target.
include resources/qpOASES/CMakeFiles/example4.dir/progress.make

# Include the compile flags for this target's objects.
include resources/qpOASES/CMakeFiles/example4.dir/flags.make

resources/qpOASES/CMakeFiles/example4.dir/examples/example4.cpp.o: resources/qpOASES/CMakeFiles/example4.dir/flags.make
resources/qpOASES/CMakeFiles/example4.dir/examples/example4.cpp.o: ../../../../resources/qpOASES/examples/example4.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/umaidzz/Documents/team-nust/team-nust-robocup-v2/build-V5/Release-Motion/build-local/team-nust-robocup/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object resources/qpOASES/CMakeFiles/example4.dir/examples/example4.cpp.o"
	cd /home/umaidzz/Documents/team-nust/team-nust-robocup-v2/build-V5/Release-Motion/build-local/team-nust-robocup/resources/qpOASES && /home/umaidzz/Documents/team-nust/ctc-linux32-atom-2.1.4.13/cross/i686-aldebaran-linux-gnu/sysroot/usr/local/bin/g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/example4.dir/examples/example4.cpp.o -c /home/umaidzz/Documents/team-nust/team-nust-robocup-v2/resources/qpOASES/examples/example4.cpp

resources/qpOASES/CMakeFiles/example4.dir/examples/example4.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/example4.dir/examples/example4.cpp.i"
	cd /home/umaidzz/Documents/team-nust/team-nust-robocup-v2/build-V5/Release-Motion/build-local/team-nust-robocup/resources/qpOASES && /home/umaidzz/Documents/team-nust/ctc-linux32-atom-2.1.4.13/cross/i686-aldebaran-linux-gnu/sysroot/usr/local/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/umaidzz/Documents/team-nust/team-nust-robocup-v2/resources/qpOASES/examples/example4.cpp > CMakeFiles/example4.dir/examples/example4.cpp.i

resources/qpOASES/CMakeFiles/example4.dir/examples/example4.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/example4.dir/examples/example4.cpp.s"
	cd /home/umaidzz/Documents/team-nust/team-nust-robocup-v2/build-V5/Release-Motion/build-local/team-nust-robocup/resources/qpOASES && /home/umaidzz/Documents/team-nust/ctc-linux32-atom-2.1.4.13/cross/i686-aldebaran-linux-gnu/sysroot/usr/local/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/umaidzz/Documents/team-nust/team-nust-robocup-v2/resources/qpOASES/examples/example4.cpp -o CMakeFiles/example4.dir/examples/example4.cpp.s

resources/qpOASES/CMakeFiles/example4.dir/examples/example4.cpp.o.requires:

.PHONY : resources/qpOASES/CMakeFiles/example4.dir/examples/example4.cpp.o.requires

resources/qpOASES/CMakeFiles/example4.dir/examples/example4.cpp.o.provides: resources/qpOASES/CMakeFiles/example4.dir/examples/example4.cpp.o.requires
	$(MAKE) -f resources/qpOASES/CMakeFiles/example4.dir/build.make resources/qpOASES/CMakeFiles/example4.dir/examples/example4.cpp.o.provides.build
.PHONY : resources/qpOASES/CMakeFiles/example4.dir/examples/example4.cpp.o.provides

resources/qpOASES/CMakeFiles/example4.dir/examples/example4.cpp.o.provides.build: resources/qpOASES/CMakeFiles/example4.dir/examples/example4.cpp.o


# Object files for target example4
example4_OBJECTS = \
"CMakeFiles/example4.dir/examples/example4.cpp.o"

# External object files for target example4
example4_EXTERNAL_OBJECTS =

../../cross/bin/example4: resources/qpOASES/CMakeFiles/example4.dir/examples/example4.cpp.o
../../cross/bin/example4: resources/qpOASES/CMakeFiles/example4.dir/build.make
../../cross/bin/example4: ../../cross/lib/libqpOASES.so.3.2
../../cross/bin/example4: resources/qpOASES/CMakeFiles/example4.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/umaidzz/Documents/team-nust/team-nust-robocup-v2/build-V5/Release-Motion/build-local/team-nust-robocup/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../../../../cross/bin/example4"
	cd /home/umaidzz/Documents/team-nust/team-nust-robocup-v2/build-V5/Release-Motion/build-local/team-nust-robocup/resources/qpOASES && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/example4.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
resources/qpOASES/CMakeFiles/example4.dir/build: ../../cross/bin/example4

.PHONY : resources/qpOASES/CMakeFiles/example4.dir/build

resources/qpOASES/CMakeFiles/example4.dir/requires: resources/qpOASES/CMakeFiles/example4.dir/examples/example4.cpp.o.requires

.PHONY : resources/qpOASES/CMakeFiles/example4.dir/requires

resources/qpOASES/CMakeFiles/example4.dir/clean:
	cd /home/umaidzz/Documents/team-nust/team-nust-robocup-v2/build-V5/Release-Motion/build-local/team-nust-robocup/resources/qpOASES && $(CMAKE_COMMAND) -P CMakeFiles/example4.dir/cmake_clean.cmake
.PHONY : resources/qpOASES/CMakeFiles/example4.dir/clean

resources/qpOASES/CMakeFiles/example4.dir/depend:
	cd /home/umaidzz/Documents/team-nust/team-nust-robocup-v2/build-V5/Release-Motion/build-local/team-nust-robocup && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/umaidzz/Documents/team-nust/team-nust-robocup-v2 /home/umaidzz/Documents/team-nust/team-nust-robocup-v2/resources/qpOASES /home/umaidzz/Documents/team-nust/team-nust-robocup-v2/build-V5/Release-Motion/build-local/team-nust-robocup /home/umaidzz/Documents/team-nust/team-nust-robocup-v2/build-V5/Release-Motion/build-local/team-nust-robocup/resources/qpOASES /home/umaidzz/Documents/team-nust/team-nust-robocup-v2/build-V5/Release-Motion/build-local/team-nust-robocup/resources/qpOASES/CMakeFiles/example4.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : resources/qpOASES/CMakeFiles/example4.dir/depend

