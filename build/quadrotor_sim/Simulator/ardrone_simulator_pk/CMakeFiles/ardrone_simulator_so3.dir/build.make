# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /home/bini/versioned/uav_ros/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/bini/versioned/uav_ros/build

# Include any dependencies generated for this target.
include quadrotor_sim/Simulator/ardrone_simulator_pk/CMakeFiles/ardrone_simulator_so3.dir/depend.make

# Include the progress variables for this target.
include quadrotor_sim/Simulator/ardrone_simulator_pk/CMakeFiles/ardrone_simulator_so3.dir/progress.make

# Include the compile flags for this target's objects.
include quadrotor_sim/Simulator/ardrone_simulator_pk/CMakeFiles/ardrone_simulator_so3.dir/flags.make

quadrotor_sim/Simulator/ardrone_simulator_pk/CMakeFiles/ardrone_simulator_so3.dir/src/ardrone_simulator_so3.cpp.o: quadrotor_sim/Simulator/ardrone_simulator_pk/CMakeFiles/ardrone_simulator_so3.dir/flags.make
quadrotor_sim/Simulator/ardrone_simulator_pk/CMakeFiles/ardrone_simulator_so3.dir/src/ardrone_simulator_so3.cpp.o: /home/bini/versioned/uav_ros/src/quadrotor_sim/Simulator/ardrone_simulator_pk/src/ardrone_simulator_so3.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/bini/versioned/uav_ros/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object quadrotor_sim/Simulator/ardrone_simulator_pk/CMakeFiles/ardrone_simulator_so3.dir/src/ardrone_simulator_so3.cpp.o"
	cd /home/bini/versioned/uav_ros/build/quadrotor_sim/Simulator/ardrone_simulator_pk && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ardrone_simulator_so3.dir/src/ardrone_simulator_so3.cpp.o -c /home/bini/versioned/uav_ros/src/quadrotor_sim/Simulator/ardrone_simulator_pk/src/ardrone_simulator_so3.cpp

quadrotor_sim/Simulator/ardrone_simulator_pk/CMakeFiles/ardrone_simulator_so3.dir/src/ardrone_simulator_so3.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ardrone_simulator_so3.dir/src/ardrone_simulator_so3.cpp.i"
	cd /home/bini/versioned/uav_ros/build/quadrotor_sim/Simulator/ardrone_simulator_pk && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/bini/versioned/uav_ros/src/quadrotor_sim/Simulator/ardrone_simulator_pk/src/ardrone_simulator_so3.cpp > CMakeFiles/ardrone_simulator_so3.dir/src/ardrone_simulator_so3.cpp.i

quadrotor_sim/Simulator/ardrone_simulator_pk/CMakeFiles/ardrone_simulator_so3.dir/src/ardrone_simulator_so3.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ardrone_simulator_so3.dir/src/ardrone_simulator_so3.cpp.s"
	cd /home/bini/versioned/uav_ros/build/quadrotor_sim/Simulator/ardrone_simulator_pk && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/bini/versioned/uav_ros/src/quadrotor_sim/Simulator/ardrone_simulator_pk/src/ardrone_simulator_so3.cpp -o CMakeFiles/ardrone_simulator_so3.dir/src/ardrone_simulator_so3.cpp.s

quadrotor_sim/Simulator/ardrone_simulator_pk/CMakeFiles/ardrone_simulator_so3.dir/src/ardrone_simulator_so3.cpp.o.requires:

.PHONY : quadrotor_sim/Simulator/ardrone_simulator_pk/CMakeFiles/ardrone_simulator_so3.dir/src/ardrone_simulator_so3.cpp.o.requires

quadrotor_sim/Simulator/ardrone_simulator_pk/CMakeFiles/ardrone_simulator_so3.dir/src/ardrone_simulator_so3.cpp.o.provides: quadrotor_sim/Simulator/ardrone_simulator_pk/CMakeFiles/ardrone_simulator_so3.dir/src/ardrone_simulator_so3.cpp.o.requires
	$(MAKE) -f quadrotor_sim/Simulator/ardrone_simulator_pk/CMakeFiles/ardrone_simulator_so3.dir/build.make quadrotor_sim/Simulator/ardrone_simulator_pk/CMakeFiles/ardrone_simulator_so3.dir/src/ardrone_simulator_so3.cpp.o.provides.build
.PHONY : quadrotor_sim/Simulator/ardrone_simulator_pk/CMakeFiles/ardrone_simulator_so3.dir/src/ardrone_simulator_so3.cpp.o.provides

quadrotor_sim/Simulator/ardrone_simulator_pk/CMakeFiles/ardrone_simulator_so3.dir/src/ardrone_simulator_so3.cpp.o.provides.build: quadrotor_sim/Simulator/ardrone_simulator_pk/CMakeFiles/ardrone_simulator_so3.dir/src/ardrone_simulator_so3.cpp.o


# Object files for target ardrone_simulator_so3
ardrone_simulator_so3_OBJECTS = \
"CMakeFiles/ardrone_simulator_so3.dir/src/ardrone_simulator_so3.cpp.o"

# External object files for target ardrone_simulator_so3
ardrone_simulator_so3_EXTERNAL_OBJECTS =

/home/bini/versioned/uav_ros/devel/lib/ardrone_simulator_pk/ardrone_simulator_so3: quadrotor_sim/Simulator/ardrone_simulator_pk/CMakeFiles/ardrone_simulator_so3.dir/src/ardrone_simulator_so3.cpp.o
/home/bini/versioned/uav_ros/devel/lib/ardrone_simulator_pk/ardrone_simulator_so3: quadrotor_sim/Simulator/ardrone_simulator_pk/CMakeFiles/ardrone_simulator_so3.dir/build.make
/home/bini/versioned/uav_ros/devel/lib/ardrone_simulator_pk/ardrone_simulator_so3: /opt/ros/melodic/lib/libroscpp.so
/home/bini/versioned/uav_ros/devel/lib/ardrone_simulator_pk/ardrone_simulator_so3: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/bini/versioned/uav_ros/devel/lib/ardrone_simulator_pk/ardrone_simulator_so3: /opt/ros/melodic/lib/librosconsole.so
/home/bini/versioned/uav_ros/devel/lib/ardrone_simulator_pk/ardrone_simulator_so3: /opt/ros/melodic/lib/librosconsole_log4cxx.so
/home/bini/versioned/uav_ros/devel/lib/ardrone_simulator_pk/ardrone_simulator_so3: /opt/ros/melodic/lib/librosconsole_backend_interface.so
/home/bini/versioned/uav_ros/devel/lib/ardrone_simulator_pk/ardrone_simulator_so3: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/bini/versioned/uav_ros/devel/lib/ardrone_simulator_pk/ardrone_simulator_so3: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/bini/versioned/uav_ros/devel/lib/ardrone_simulator_pk/ardrone_simulator_so3: /opt/ros/melodic/lib/libxmlrpcpp.so
/home/bini/versioned/uav_ros/devel/lib/ardrone_simulator_pk/ardrone_simulator_so3: /opt/ros/melodic/lib/libroscpp_serialization.so
/home/bini/versioned/uav_ros/devel/lib/ardrone_simulator_pk/ardrone_simulator_so3: /opt/ros/melodic/lib/librostime.so
/home/bini/versioned/uav_ros/devel/lib/ardrone_simulator_pk/ardrone_simulator_so3: /opt/ros/melodic/lib/libcpp_common.so
/home/bini/versioned/uav_ros/devel/lib/ardrone_simulator_pk/ardrone_simulator_so3: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/bini/versioned/uav_ros/devel/lib/ardrone_simulator_pk/ardrone_simulator_so3: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/bini/versioned/uav_ros/devel/lib/ardrone_simulator_pk/ardrone_simulator_so3: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
/home/bini/versioned/uav_ros/devel/lib/ardrone_simulator_pk/ardrone_simulator_so3: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/bini/versioned/uav_ros/devel/lib/ardrone_simulator_pk/ardrone_simulator_so3: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
/home/bini/versioned/uav_ros/devel/lib/ardrone_simulator_pk/ardrone_simulator_so3: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/bini/versioned/uav_ros/devel/lib/ardrone_simulator_pk/ardrone_simulator_so3: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/bini/versioned/uav_ros/devel/lib/ardrone_simulator_pk/ardrone_simulator_so3: /home/bini/versioned/uav_ros/devel/lib/libardrone_dynamics.so
/home/bini/versioned/uav_ros/devel/lib/ardrone_simulator_pk/ardrone_simulator_so3: quadrotor_sim/Simulator/ardrone_simulator_pk/CMakeFiles/ardrone_simulator_so3.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/bini/versioned/uav_ros/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/bini/versioned/uav_ros/devel/lib/ardrone_simulator_pk/ardrone_simulator_so3"
	cd /home/bini/versioned/uav_ros/build/quadrotor_sim/Simulator/ardrone_simulator_pk && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ardrone_simulator_so3.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
quadrotor_sim/Simulator/ardrone_simulator_pk/CMakeFiles/ardrone_simulator_so3.dir/build: /home/bini/versioned/uav_ros/devel/lib/ardrone_simulator_pk/ardrone_simulator_so3

.PHONY : quadrotor_sim/Simulator/ardrone_simulator_pk/CMakeFiles/ardrone_simulator_so3.dir/build

quadrotor_sim/Simulator/ardrone_simulator_pk/CMakeFiles/ardrone_simulator_so3.dir/requires: quadrotor_sim/Simulator/ardrone_simulator_pk/CMakeFiles/ardrone_simulator_so3.dir/src/ardrone_simulator_so3.cpp.o.requires

.PHONY : quadrotor_sim/Simulator/ardrone_simulator_pk/CMakeFiles/ardrone_simulator_so3.dir/requires

quadrotor_sim/Simulator/ardrone_simulator_pk/CMakeFiles/ardrone_simulator_so3.dir/clean:
	cd /home/bini/versioned/uav_ros/build/quadrotor_sim/Simulator/ardrone_simulator_pk && $(CMAKE_COMMAND) -P CMakeFiles/ardrone_simulator_so3.dir/cmake_clean.cmake
.PHONY : quadrotor_sim/Simulator/ardrone_simulator_pk/CMakeFiles/ardrone_simulator_so3.dir/clean

quadrotor_sim/Simulator/ardrone_simulator_pk/CMakeFiles/ardrone_simulator_so3.dir/depend:
	cd /home/bini/versioned/uav_ros/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/bini/versioned/uav_ros/src /home/bini/versioned/uav_ros/src/quadrotor_sim/Simulator/ardrone_simulator_pk /home/bini/versioned/uav_ros/build /home/bini/versioned/uav_ros/build/quadrotor_sim/Simulator/ardrone_simulator_pk /home/bini/versioned/uav_ros/build/quadrotor_sim/Simulator/ardrone_simulator_pk/CMakeFiles/ardrone_simulator_so3.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : quadrotor_sim/Simulator/ardrone_simulator_pk/CMakeFiles/ardrone_simulator_so3.dir/depend

