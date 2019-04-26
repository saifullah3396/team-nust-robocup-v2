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
include resources/RandomLib/CMakeFiles/tnrs-random-lib.dir/depend.make

# Include the progress variables for this target.
include resources/RandomLib/CMakeFiles/tnrs-random-lib.dir/progress.make

# Include the compile flags for this target's objects.
include resources/RandomLib/CMakeFiles/tnrs-random-lib.dir/flags.make

resources/RandomLib/CMakeFiles/tnrs-random-lib.dir/src/Random.cpp.o: resources/RandomLib/CMakeFiles/tnrs-random-lib.dir/flags.make
resources/RandomLib/CMakeFiles/tnrs-random-lib.dir/src/Random.cpp.o: ../../../../resources/RandomLib/src/Random.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/umaidzz/Documents/team-nust/team-nust-robocup-v2/build-V5/Release-Motion/build-remote/team-nust-robocup/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object resources/RandomLib/CMakeFiles/tnrs-random-lib.dir/src/Random.cpp.o"
	cd /home/umaidzz/Documents/team-nust/team-nust-robocup-v2/build-V5/Release-Motion/build-remote/team-nust-robocup/resources/RandomLib && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/tnrs-random-lib.dir/src/Random.cpp.o -c /home/umaidzz/Documents/team-nust/team-nust-robocup-v2/resources/RandomLib/src/Random.cpp

resources/RandomLib/CMakeFiles/tnrs-random-lib.dir/src/Random.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/tnrs-random-lib.dir/src/Random.cpp.i"
	cd /home/umaidzz/Documents/team-nust/team-nust-robocup-v2/build-V5/Release-Motion/build-remote/team-nust-robocup/resources/RandomLib && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/umaidzz/Documents/team-nust/team-nust-robocup-v2/resources/RandomLib/src/Random.cpp > CMakeFiles/tnrs-random-lib.dir/src/Random.cpp.i

resources/RandomLib/CMakeFiles/tnrs-random-lib.dir/src/Random.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/tnrs-random-lib.dir/src/Random.cpp.s"
	cd /home/umaidzz/Documents/team-nust/team-nust-robocup-v2/build-V5/Release-Motion/build-remote/team-nust-robocup/resources/RandomLib && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/umaidzz/Documents/team-nust/team-nust-robocup-v2/resources/RandomLib/src/Random.cpp -o CMakeFiles/tnrs-random-lib.dir/src/Random.cpp.s

resources/RandomLib/CMakeFiles/tnrs-random-lib.dir/src/Random.cpp.o.requires:

.PHONY : resources/RandomLib/CMakeFiles/tnrs-random-lib.dir/src/Random.cpp.o.requires

resources/RandomLib/CMakeFiles/tnrs-random-lib.dir/src/Random.cpp.o.provides: resources/RandomLib/CMakeFiles/tnrs-random-lib.dir/src/Random.cpp.o.requires
	$(MAKE) -f resources/RandomLib/CMakeFiles/tnrs-random-lib.dir/build.make resources/RandomLib/CMakeFiles/tnrs-random-lib.dir/src/Random.cpp.o.provides.build
.PHONY : resources/RandomLib/CMakeFiles/tnrs-random-lib.dir/src/Random.cpp.o.provides

resources/RandomLib/CMakeFiles/tnrs-random-lib.dir/src/Random.cpp.o.provides.build: resources/RandomLib/CMakeFiles/tnrs-random-lib.dir/src/Random.cpp.o


# Object files for target tnrs-random-lib
tnrs__random__lib_OBJECTS = \
"CMakeFiles/tnrs-random-lib.dir/src/Random.cpp.o"

# External object files for target tnrs-random-lib
tnrs__random__lib_EXTERNAL_OBJECTS =

../../remote/lib/libtnrs-random-lib.so: resources/RandomLib/CMakeFiles/tnrs-random-lib.dir/src/Random.cpp.o
../../remote/lib/libtnrs-random-lib.so: resources/RandomLib/CMakeFiles/tnrs-random-lib.dir/build.make
../../remote/lib/libtnrs-random-lib.so: resources/RandomLib/CMakeFiles/tnrs-random-lib.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/umaidzz/Documents/team-nust/team-nust-robocup-v2/build-V5/Release-Motion/build-remote/team-nust-robocup/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library ../../../../remote/lib/libtnrs-random-lib.so"
	cd /home/umaidzz/Documents/team-nust/team-nust-robocup-v2/build-V5/Release-Motion/build-remote/team-nust-robocup/resources/RandomLib && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/tnrs-random-lib.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
resources/RandomLib/CMakeFiles/tnrs-random-lib.dir/build: ../../remote/lib/libtnrs-random-lib.so

.PHONY : resources/RandomLib/CMakeFiles/tnrs-random-lib.dir/build

resources/RandomLib/CMakeFiles/tnrs-random-lib.dir/requires: resources/RandomLib/CMakeFiles/tnrs-random-lib.dir/src/Random.cpp.o.requires

.PHONY : resources/RandomLib/CMakeFiles/tnrs-random-lib.dir/requires

resources/RandomLib/CMakeFiles/tnrs-random-lib.dir/clean:
	cd /home/umaidzz/Documents/team-nust/team-nust-robocup-v2/build-V5/Release-Motion/build-remote/team-nust-robocup/resources/RandomLib && $(CMAKE_COMMAND) -P CMakeFiles/tnrs-random-lib.dir/cmake_clean.cmake
.PHONY : resources/RandomLib/CMakeFiles/tnrs-random-lib.dir/clean

resources/RandomLib/CMakeFiles/tnrs-random-lib.dir/depend:
	cd /home/umaidzz/Documents/team-nust/team-nust-robocup-v2/build-V5/Release-Motion/build-remote/team-nust-robocup && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/umaidzz/Documents/team-nust/team-nust-robocup-v2 /home/umaidzz/Documents/team-nust/team-nust-robocup-v2/resources/RandomLib /home/umaidzz/Documents/team-nust/team-nust-robocup-v2/build-V5/Release-Motion/build-remote/team-nust-robocup /home/umaidzz/Documents/team-nust/team-nust-robocup-v2/build-V5/Release-Motion/build-remote/team-nust-robocup/resources/RandomLib /home/umaidzz/Documents/team-nust/team-nust-robocup-v2/build-V5/Release-Motion/build-remote/team-nust-robocup/resources/RandomLib/CMakeFiles/tnrs-random-lib.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : resources/RandomLib/CMakeFiles/tnrs-random-lib.dir/depend

