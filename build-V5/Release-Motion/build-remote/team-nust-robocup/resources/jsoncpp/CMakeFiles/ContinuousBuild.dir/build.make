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

# Utility rule file for ContinuousBuild.

# Include the progress variables for this target.
include resources/jsoncpp/CMakeFiles/ContinuousBuild.dir/progress.make

resources/jsoncpp/CMakeFiles/ContinuousBuild:
	cd /home/umaidzz/Documents/team-nust/team-nust-robocup-v2/build-V5/Release-Motion/build-remote/team-nust-robocup/resources/jsoncpp && /usr/bin/ctest -D ContinuousBuild

ContinuousBuild: resources/jsoncpp/CMakeFiles/ContinuousBuild
ContinuousBuild: resources/jsoncpp/CMakeFiles/ContinuousBuild.dir/build.make

.PHONY : ContinuousBuild

# Rule to build all files generated by this target.
resources/jsoncpp/CMakeFiles/ContinuousBuild.dir/build: ContinuousBuild

.PHONY : resources/jsoncpp/CMakeFiles/ContinuousBuild.dir/build

resources/jsoncpp/CMakeFiles/ContinuousBuild.dir/clean:
	cd /home/umaidzz/Documents/team-nust/team-nust-robocup-v2/build-V5/Release-Motion/build-remote/team-nust-robocup/resources/jsoncpp && $(CMAKE_COMMAND) -P CMakeFiles/ContinuousBuild.dir/cmake_clean.cmake
.PHONY : resources/jsoncpp/CMakeFiles/ContinuousBuild.dir/clean

resources/jsoncpp/CMakeFiles/ContinuousBuild.dir/depend:
	cd /home/umaidzz/Documents/team-nust/team-nust-robocup-v2/build-V5/Release-Motion/build-remote/team-nust-robocup && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/umaidzz/Documents/team-nust/team-nust-robocup-v2 /home/umaidzz/Documents/team-nust/team-nust-robocup-v2/resources/jsoncpp /home/umaidzz/Documents/team-nust/team-nust-robocup-v2/build-V5/Release-Motion/build-remote/team-nust-robocup /home/umaidzz/Documents/team-nust/team-nust-robocup-v2/build-V5/Release-Motion/build-remote/team-nust-robocup/resources/jsoncpp /home/umaidzz/Documents/team-nust/team-nust-robocup-v2/build-V5/Release-Motion/build-remote/team-nust-robocup/resources/jsoncpp/CMakeFiles/ContinuousBuild.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : resources/jsoncpp/CMakeFiles/ContinuousBuild.dir/depend

