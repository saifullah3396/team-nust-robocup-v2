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
CMAKE_SOURCE_DIR = /home/sensei/team-nust-robocup/Src/GameController/src/libgamectrl

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/sensei/team-nust-robocup/Src/GameController/src/libgamectrl/build-remote

# Include any dependencies generated for this target.
include CMakeFiles/gamectrl.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/gamectrl.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/gamectrl.dir/flags.make

CMakeFiles/gamectrl.dir/GameCtrl.cpp.o: CMakeFiles/gamectrl.dir/flags.make
CMakeFiles/gamectrl.dir/GameCtrl.cpp.o: ../GameCtrl.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/sensei/team-nust-robocup/Src/GameController/src/libgamectrl/build-remote/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/gamectrl.dir/GameCtrl.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/gamectrl.dir/GameCtrl.cpp.o -c /home/sensei/team-nust-robocup/Src/GameController/src/libgamectrl/GameCtrl.cpp

CMakeFiles/gamectrl.dir/GameCtrl.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/gamectrl.dir/GameCtrl.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/sensei/team-nust-robocup/Src/GameController/src/libgamectrl/GameCtrl.cpp > CMakeFiles/gamectrl.dir/GameCtrl.cpp.i

CMakeFiles/gamectrl.dir/GameCtrl.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/gamectrl.dir/GameCtrl.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/sensei/team-nust-robocup/Src/GameController/src/libgamectrl/GameCtrl.cpp -o CMakeFiles/gamectrl.dir/GameCtrl.cpp.s

CMakeFiles/gamectrl.dir/GameCtrl.cpp.o.requires:

.PHONY : CMakeFiles/gamectrl.dir/GameCtrl.cpp.o.requires

CMakeFiles/gamectrl.dir/GameCtrl.cpp.o.provides: CMakeFiles/gamectrl.dir/GameCtrl.cpp.o.requires
	$(MAKE) -f CMakeFiles/gamectrl.dir/build.make CMakeFiles/gamectrl.dir/GameCtrl.cpp.o.provides.build
.PHONY : CMakeFiles/gamectrl.dir/GameCtrl.cpp.o.provides

CMakeFiles/gamectrl.dir/GameCtrl.cpp.o.provides.build: CMakeFiles/gamectrl.dir/GameCtrl.cpp.o


CMakeFiles/gamectrl.dir/UdpComm.cpp.o: CMakeFiles/gamectrl.dir/flags.make
CMakeFiles/gamectrl.dir/UdpComm.cpp.o: ../UdpComm.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/sensei/team-nust-robocup/Src/GameController/src/libgamectrl/build-remote/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/gamectrl.dir/UdpComm.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/gamectrl.dir/UdpComm.cpp.o -c /home/sensei/team-nust-robocup/Src/GameController/src/libgamectrl/UdpComm.cpp

CMakeFiles/gamectrl.dir/UdpComm.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/gamectrl.dir/UdpComm.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/sensei/team-nust-robocup/Src/GameController/src/libgamectrl/UdpComm.cpp > CMakeFiles/gamectrl.dir/UdpComm.cpp.i

CMakeFiles/gamectrl.dir/UdpComm.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/gamectrl.dir/UdpComm.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/sensei/team-nust-robocup/Src/GameController/src/libgamectrl/UdpComm.cpp -o CMakeFiles/gamectrl.dir/UdpComm.cpp.s

CMakeFiles/gamectrl.dir/UdpComm.cpp.o.requires:

.PHONY : CMakeFiles/gamectrl.dir/UdpComm.cpp.o.requires

CMakeFiles/gamectrl.dir/UdpComm.cpp.o.provides: CMakeFiles/gamectrl.dir/UdpComm.cpp.o.requires
	$(MAKE) -f CMakeFiles/gamectrl.dir/build.make CMakeFiles/gamectrl.dir/UdpComm.cpp.o.provides.build
.PHONY : CMakeFiles/gamectrl.dir/UdpComm.cpp.o.provides

CMakeFiles/gamectrl.dir/UdpComm.cpp.o.provides.build: CMakeFiles/gamectrl.dir/UdpComm.cpp.o


# Object files for target gamectrl
gamectrl_OBJECTS = \
"CMakeFiles/gamectrl.dir/GameCtrl.cpp.o" \
"CMakeFiles/gamectrl.dir/UdpComm.cpp.o"

# External object files for target gamectrl
gamectrl_EXTERNAL_OBJECTS =

sdk/lib/naoqi/libgamectrl.so: CMakeFiles/gamectrl.dir/GameCtrl.cpp.o
sdk/lib/naoqi/libgamectrl.so: CMakeFiles/gamectrl.dir/UdpComm.cpp.o
sdk/lib/naoqi/libgamectrl.so: CMakeFiles/gamectrl.dir/build.make
sdk/lib/naoqi/libgamectrl.so: /home/sensei/Downloads/naoqi-sdk-2.1.4.13-linux64/lib/libalcommon.so
sdk/lib/naoqi/libgamectrl.so: /home/sensei/Downloads/naoqi-sdk-2.1.4.13-linux64/lib/libalvalue.so
sdk/lib/naoqi/libgamectrl.so: /home/sensei/Downloads/naoqi-sdk-2.1.4.13-linux64/lib/libalerror.so
sdk/lib/naoqi/libgamectrl.so: /home/sensei/Downloads/naoqi-sdk-2.1.4.13-linux64/lib/libqimessaging.so
sdk/lib/naoqi/libgamectrl.so: /home/sensei/Downloads/naoqi-sdk-2.1.4.13-linux64/lib/libqitype.so
sdk/lib/naoqi/libgamectrl.so: /home/sensei/Downloads/naoqi-sdk-2.1.4.13-linux64/lib/libboost_date_time.so
sdk/lib/naoqi/libgamectrl.so: /home/sensei/Downloads/naoqi-sdk-2.1.4.13-linux64/lib/libboost_signals.so
sdk/lib/naoqi/libgamectrl.so: /home/sensei/Downloads/naoqi-sdk-2.1.4.13-linux64/lib/librttools.so
sdk/lib/naoqi/libgamectrl.so: /home/sensei/Downloads/naoqi-sdk-2.1.4.13-linux64/lib/libqi.so
sdk/lib/naoqi/libgamectrl.so: /home/sensei/Downloads/naoqi-sdk-2.1.4.13-linux64/lib/libboost_chrono.so
sdk/lib/naoqi/libgamectrl.so: /home/sensei/Downloads/naoqi-sdk-2.1.4.13-linux64/lib/libboost_filesystem.so
sdk/lib/naoqi/libgamectrl.so: /home/sensei/Downloads/naoqi-sdk-2.1.4.13-linux64/lib/libboost_system.so
sdk/lib/naoqi/libgamectrl.so: /home/sensei/Downloads/naoqi-sdk-2.1.4.13-linux64/lib/libboost_program_options.so
sdk/lib/naoqi/libgamectrl.so: /home/sensei/Downloads/naoqi-sdk-2.1.4.13-linux64/lib/libboost_regex.so
sdk/lib/naoqi/libgamectrl.so: /home/sensei/Downloads/naoqi-sdk-2.1.4.13-linux64/lib/libboost_locale.so
sdk/lib/naoqi/libgamectrl.so: /home/sensei/Downloads/naoqi-sdk-2.1.4.13-linux64/lib/libboost_thread.so
sdk/lib/naoqi/libgamectrl.so: /usr/lib/x86_64-linux-gnu/librt.so
sdk/lib/naoqi/libgamectrl.so: /usr/lib/x86_64-linux-gnu/libdl.so
sdk/lib/naoqi/libgamectrl.so: CMakeFiles/gamectrl.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/sensei/team-nust-robocup/Src/GameController/src/libgamectrl/build-remote/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX shared library sdk/lib/naoqi/libgamectrl.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/gamectrl.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/gamectrl.dir/build: sdk/lib/naoqi/libgamectrl.so

.PHONY : CMakeFiles/gamectrl.dir/build

# Object files for target gamectrl
gamectrl_OBJECTS = \
"CMakeFiles/gamectrl.dir/GameCtrl.cpp.o" \
"CMakeFiles/gamectrl.dir/UdpComm.cpp.o"

# External object files for target gamectrl
gamectrl_EXTERNAL_OBJECTS =

CMakeFiles/CMakeRelink.dir/libgamectrl.so: CMakeFiles/gamectrl.dir/GameCtrl.cpp.o
CMakeFiles/CMakeRelink.dir/libgamectrl.so: CMakeFiles/gamectrl.dir/UdpComm.cpp.o
CMakeFiles/CMakeRelink.dir/libgamectrl.so: CMakeFiles/gamectrl.dir/build.make
CMakeFiles/CMakeRelink.dir/libgamectrl.so: /home/sensei/Downloads/naoqi-sdk-2.1.4.13-linux64/lib/libalcommon.so
CMakeFiles/CMakeRelink.dir/libgamectrl.so: /home/sensei/Downloads/naoqi-sdk-2.1.4.13-linux64/lib/libalvalue.so
CMakeFiles/CMakeRelink.dir/libgamectrl.so: /home/sensei/Downloads/naoqi-sdk-2.1.4.13-linux64/lib/libalerror.so
CMakeFiles/CMakeRelink.dir/libgamectrl.so: /home/sensei/Downloads/naoqi-sdk-2.1.4.13-linux64/lib/libqimessaging.so
CMakeFiles/CMakeRelink.dir/libgamectrl.so: /home/sensei/Downloads/naoqi-sdk-2.1.4.13-linux64/lib/libqitype.so
CMakeFiles/CMakeRelink.dir/libgamectrl.so: /home/sensei/Downloads/naoqi-sdk-2.1.4.13-linux64/lib/libboost_date_time.so
CMakeFiles/CMakeRelink.dir/libgamectrl.so: /home/sensei/Downloads/naoqi-sdk-2.1.4.13-linux64/lib/libboost_signals.so
CMakeFiles/CMakeRelink.dir/libgamectrl.so: /home/sensei/Downloads/naoqi-sdk-2.1.4.13-linux64/lib/librttools.so
CMakeFiles/CMakeRelink.dir/libgamectrl.so: /home/sensei/Downloads/naoqi-sdk-2.1.4.13-linux64/lib/libqi.so
CMakeFiles/CMakeRelink.dir/libgamectrl.so: /home/sensei/Downloads/naoqi-sdk-2.1.4.13-linux64/lib/libboost_chrono.so
CMakeFiles/CMakeRelink.dir/libgamectrl.so: /home/sensei/Downloads/naoqi-sdk-2.1.4.13-linux64/lib/libboost_filesystem.so
CMakeFiles/CMakeRelink.dir/libgamectrl.so: /home/sensei/Downloads/naoqi-sdk-2.1.4.13-linux64/lib/libboost_system.so
CMakeFiles/CMakeRelink.dir/libgamectrl.so: /home/sensei/Downloads/naoqi-sdk-2.1.4.13-linux64/lib/libboost_program_options.so
CMakeFiles/CMakeRelink.dir/libgamectrl.so: /home/sensei/Downloads/naoqi-sdk-2.1.4.13-linux64/lib/libboost_regex.so
CMakeFiles/CMakeRelink.dir/libgamectrl.so: /home/sensei/Downloads/naoqi-sdk-2.1.4.13-linux64/lib/libboost_locale.so
CMakeFiles/CMakeRelink.dir/libgamectrl.so: /home/sensei/Downloads/naoqi-sdk-2.1.4.13-linux64/lib/libboost_thread.so
CMakeFiles/CMakeRelink.dir/libgamectrl.so: /usr/lib/x86_64-linux-gnu/librt.so
CMakeFiles/CMakeRelink.dir/libgamectrl.so: /usr/lib/x86_64-linux-gnu/libdl.so
CMakeFiles/CMakeRelink.dir/libgamectrl.so: CMakeFiles/gamectrl.dir/relink.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/sensei/team-nust-robocup/Src/GameController/src/libgamectrl/build-remote/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX shared library CMakeFiles/CMakeRelink.dir/libgamectrl.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/gamectrl.dir/relink.txt --verbose=$(VERBOSE)

# Rule to relink during preinstall.
CMakeFiles/gamectrl.dir/preinstall: CMakeFiles/CMakeRelink.dir/libgamectrl.so

.PHONY : CMakeFiles/gamectrl.dir/preinstall

CMakeFiles/gamectrl.dir/requires: CMakeFiles/gamectrl.dir/GameCtrl.cpp.o.requires
CMakeFiles/gamectrl.dir/requires: CMakeFiles/gamectrl.dir/UdpComm.cpp.o.requires

.PHONY : CMakeFiles/gamectrl.dir/requires

CMakeFiles/gamectrl.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/gamectrl.dir/cmake_clean.cmake
.PHONY : CMakeFiles/gamectrl.dir/clean

CMakeFiles/gamectrl.dir/depend:
	cd /home/sensei/team-nust-robocup/Src/GameController/src/libgamectrl/build-remote && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/sensei/team-nust-robocup/Src/GameController/src/libgamectrl /home/sensei/team-nust-robocup/Src/GameController/src/libgamectrl /home/sensei/team-nust-robocup/Src/GameController/src/libgamectrl/build-remote /home/sensei/team-nust-robocup/Src/GameController/src/libgamectrl/build-remote /home/sensei/team-nust-robocup/Src/GameController/src/libgamectrl/build-remote/CMakeFiles/gamectrl.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/gamectrl.dir/depend

