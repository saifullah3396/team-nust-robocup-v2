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
include resources/qpOASES/CMakeFiles/example1b.dir/depend.make

# Include the progress variables for this target.
include resources/qpOASES/CMakeFiles/example1b.dir/progress.make

# Include the compile flags for this target's objects.
include resources/qpOASES/CMakeFiles/example1b.dir/flags.make

resources/qpOASES/CMakeFiles/example1b.dir/examples/example1b.cpp.o: resources/qpOASES/CMakeFiles/example1b.dir/flags.make
resources/qpOASES/CMakeFiles/example1b.dir/examples/example1b.cpp.o: ../../../../resources/qpOASES/examples/example1b.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/umaidzz/Documents/team-nust/team-nust-robocup-v2/build-V5/Release-Motion/build-local/team-nust-robocup/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object resources/qpOASES/CMakeFiles/example1b.dir/examples/example1b.cpp.o"
	cd /home/umaidzz/Documents/team-nust/team-nust-robocup-v2/build-V5/Release-Motion/build-local/team-nust-robocup/resources/qpOASES && /home/umaidzz/Documents/team-nust/ctc-linux32-atom-2.1.4.13/cross/i686-aldebaran-linux-gnu/sysroot/usr/local/bin/g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/example1b.dir/examples/example1b.cpp.o -c /home/umaidzz/Documents/team-nust/team-nust-robocup-v2/resources/qpOASES/examples/example1b.cpp

resources/qpOASES/CMakeFiles/example1b.dir/examples/example1b.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/example1b.dir/examples/example1b.cpp.i"
	cd /home/umaidzz/Documents/team-nust/team-nust-robocup-v2/build-V5/Release-Motion/build-local/team-nust-robocup/resources/qpOASES && /home/umaidzz/Documents/team-nust/ctc-linux32-atom-2.1.4.13/cross/i686-aldebaran-linux-gnu/sysroot/usr/local/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/umaidzz/Documents/team-nust/team-nust-robocup-v2/resources/qpOASES/examples/example1b.cpp > CMakeFiles/example1b.dir/examples/example1b.cpp.i

resources/qpOASES/CMakeFiles/example1b.dir/examples/example1b.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/example1b.dir/examples/example1b.cpp.s"
	cd /home/umaidzz/Documents/team-nust/team-nust-robocup-v2/build-V5/Release-Motion/build-local/team-nust-robocup/resources/qpOASES && /home/umaidzz/Documents/team-nust/ctc-linux32-atom-2.1.4.13/cross/i686-aldebaran-linux-gnu/sysroot/usr/local/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/umaidzz/Documents/team-nust/team-nust-robocup-v2/resources/qpOASES/examples/example1b.cpp -o CMakeFiles/example1b.dir/examples/example1b.cpp.s

resources/qpOASES/CMakeFiles/example1b.dir/examples/example1b.cpp.o.requires:

.PHONY : resources/qpOASES/CMakeFiles/example1b.dir/examples/example1b.cpp.o.requires

resources/qpOASES/CMakeFiles/example1b.dir/examples/example1b.cpp.o.provides: resources/qpOASES/CMakeFiles/example1b.dir/examples/example1b.cpp.o.requires
	$(MAKE) -f resources/qpOASES/CMakeFiles/example1b.dir/build.make resources/qpOASES/CMakeFiles/example1b.dir/examples/example1b.cpp.o.provides.build
.PHONY : resources/qpOASES/CMakeFiles/example1b.dir/examples/example1b.cpp.o.provides

resources/qpOASES/CMakeFiles/example1b.dir/examples/example1b.cpp.o.provides.build: resources/qpOASES/CMakeFiles/example1b.dir/examples/example1b.cpp.o


# Object files for target example1b
example1b_OBJECTS = \
"CMakeFiles/example1b.dir/examples/example1b.cpp.o"

# External object files for target example1b
example1b_EXTERNAL_OBJECTS =

../../cross/bin/example1b: resources/qpOASES/CMakeFiles/example1b.dir/examples/example1b.cpp.o
../../cross/bin/example1b: resources/qpOASES/CMakeFiles/example1b.dir/build.make
../../cross/bin/example1b: ../../cross/lib/libqpOASES.so.3.2
../../cross/bin/example1b: resources/qpOASES/CMakeFiles/example1b.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/umaidzz/Documents/team-nust/team-nust-robocup-v2/build-V5/Release-Motion/build-local/team-nust-robocup/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../../../../cross/bin/example1b"
	cd /home/umaidzz/Documents/team-nust/team-nust-robocup-v2/build-V5/Release-Motion/build-local/team-nust-robocup/resources/qpOASES && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/example1b.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
resources/qpOASES/CMakeFiles/example1b.dir/build: ../../cross/bin/example1b

.PHONY : resources/qpOASES/CMakeFiles/example1b.dir/build

resources/qpOASES/CMakeFiles/example1b.dir/requires: resources/qpOASES/CMakeFiles/example1b.dir/examples/example1b.cpp.o.requires

.PHONY : resources/qpOASES/CMakeFiles/example1b.dir/requires

resources/qpOASES/CMakeFiles/example1b.dir/clean:
	cd /home/umaidzz/Documents/team-nust/team-nust-robocup-v2/build-V5/Release-Motion/build-local/team-nust-robocup/resources/qpOASES && $(CMAKE_COMMAND) -P CMakeFiles/example1b.dir/cmake_clean.cmake
.PHONY : resources/qpOASES/CMakeFiles/example1b.dir/clean

resources/qpOASES/CMakeFiles/example1b.dir/depend:
	cd /home/umaidzz/Documents/team-nust/team-nust-robocup-v2/build-V5/Release-Motion/build-local/team-nust-robocup && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/umaidzz/Documents/team-nust/team-nust-robocup-v2 /home/umaidzz/Documents/team-nust/team-nust-robocup-v2/resources/qpOASES /home/umaidzz/Documents/team-nust/team-nust-robocup-v2/build-V5/Release-Motion/build-local/team-nust-robocup /home/umaidzz/Documents/team-nust/team-nust-robocup-v2/build-V5/Release-Motion/build-local/team-nust-robocup/resources/qpOASES /home/umaidzz/Documents/team-nust/team-nust-robocup-v2/build-V5/Release-Motion/build-local/team-nust-robocup/resources/qpOASES/CMakeFiles/example1b.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : resources/qpOASES/CMakeFiles/example1b.dir/depend

