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

# Utility rule file for quadrotor_msgs_generate_messages_py.

# Include the progress variables for this target.
include quadrotor_sim/quadrotor_msgs/CMakeFiles/quadrotor_msgs_generate_messages_py.dir/progress.make

quadrotor_sim/quadrotor_msgs/CMakeFiles/quadrotor_msgs_generate_messages_py: /home/bini/versioned/uav_ros/devel/lib/python2.7/dist-packages/quadrotor_msgs/msg/_OutputData.py
quadrotor_sim/quadrotor_msgs/CMakeFiles/quadrotor_msgs_generate_messages_py: /home/bini/versioned/uav_ros/devel/lib/python2.7/dist-packages/quadrotor_msgs/msg/_StatusData.py
quadrotor_sim/quadrotor_msgs/CMakeFiles/quadrotor_msgs_generate_messages_py: /home/bini/versioned/uav_ros/devel/lib/python2.7/dist-packages/quadrotor_msgs/msg/_PositionCommand.py
quadrotor_sim/quadrotor_msgs/CMakeFiles/quadrotor_msgs_generate_messages_py: /home/bini/versioned/uav_ros/devel/lib/python2.7/dist-packages/quadrotor_msgs/msg/_Corrections.py
quadrotor_sim/quadrotor_msgs/CMakeFiles/quadrotor_msgs_generate_messages_py: /home/bini/versioned/uav_ros/devel/lib/python2.7/dist-packages/quadrotor_msgs/msg/_LinearCommand.py
quadrotor_sim/quadrotor_msgs/CMakeFiles/quadrotor_msgs_generate_messages_py: /home/bini/versioned/uav_ros/devel/lib/python2.7/dist-packages/quadrotor_msgs/msg/_SO3Command.py
quadrotor_sim/quadrotor_msgs/CMakeFiles/quadrotor_msgs_generate_messages_py: /home/bini/versioned/uav_ros/devel/lib/python2.7/dist-packages/quadrotor_msgs/msg/_TRPYCommand.py
quadrotor_sim/quadrotor_msgs/CMakeFiles/quadrotor_msgs_generate_messages_py: /home/bini/versioned/uav_ros/devel/lib/python2.7/dist-packages/quadrotor_msgs/msg/_Serial.py
quadrotor_sim/quadrotor_msgs/CMakeFiles/quadrotor_msgs_generate_messages_py: /home/bini/versioned/uav_ros/devel/lib/python2.7/dist-packages/quadrotor_msgs/msg/_AuxCommand.py
quadrotor_sim/quadrotor_msgs/CMakeFiles/quadrotor_msgs_generate_messages_py: /home/bini/versioned/uav_ros/devel/lib/python2.7/dist-packages/quadrotor_msgs/msg/__init__.py


/home/bini/versioned/uav_ros/devel/lib/python2.7/dist-packages/quadrotor_msgs/msg/_OutputData.py: /opt/ros/melodic/lib/genpy/genmsg_py.py
/home/bini/versioned/uav_ros/devel/lib/python2.7/dist-packages/quadrotor_msgs/msg/_OutputData.py: /home/bini/versioned/uav_ros/src/quadrotor_sim/quadrotor_msgs/msg/OutputData.msg
/home/bini/versioned/uav_ros/devel/lib/python2.7/dist-packages/quadrotor_msgs/msg/_OutputData.py: /opt/ros/melodic/share/geometry_msgs/msg/Vector3.msg
/home/bini/versioned/uav_ros/devel/lib/python2.7/dist-packages/quadrotor_msgs/msg/_OutputData.py: /opt/ros/melodic/share/geometry_msgs/msg/Quaternion.msg
/home/bini/versioned/uav_ros/devel/lib/python2.7/dist-packages/quadrotor_msgs/msg/_OutputData.py: /opt/ros/melodic/share/std_msgs/msg/Header.msg
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/bini/versioned/uav_ros/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating Python from MSG quadrotor_msgs/OutputData"
	cd /home/bini/versioned/uav_ros/build/quadrotor_sim/quadrotor_msgs && ../../catkin_generated/env_cached.sh /usr/bin/python2 /opt/ros/melodic/share/genpy/cmake/../../../lib/genpy/genmsg_py.py /home/bini/versioned/uav_ros/src/quadrotor_sim/quadrotor_msgs/msg/OutputData.msg -Iquadrotor_msgs:/home/bini/versioned/uav_ros/src/quadrotor_sim/quadrotor_msgs/msg -Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg -Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg -Iquadrotor_msgs:/home/bini/versioned/uav_ros/src/quadrotor_sim/quadrotor_msgs/msg -p quadrotor_msgs -o /home/bini/versioned/uav_ros/devel/lib/python2.7/dist-packages/quadrotor_msgs/msg

/home/bini/versioned/uav_ros/devel/lib/python2.7/dist-packages/quadrotor_msgs/msg/_StatusData.py: /opt/ros/melodic/lib/genpy/genmsg_py.py
/home/bini/versioned/uav_ros/devel/lib/python2.7/dist-packages/quadrotor_msgs/msg/_StatusData.py: /home/bini/versioned/uav_ros/src/quadrotor_sim/quadrotor_msgs/msg/StatusData.msg
/home/bini/versioned/uav_ros/devel/lib/python2.7/dist-packages/quadrotor_msgs/msg/_StatusData.py: /opt/ros/melodic/share/std_msgs/msg/Header.msg
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/bini/versioned/uav_ros/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Generating Python from MSG quadrotor_msgs/StatusData"
	cd /home/bini/versioned/uav_ros/build/quadrotor_sim/quadrotor_msgs && ../../catkin_generated/env_cached.sh /usr/bin/python2 /opt/ros/melodic/share/genpy/cmake/../../../lib/genpy/genmsg_py.py /home/bini/versioned/uav_ros/src/quadrotor_sim/quadrotor_msgs/msg/StatusData.msg -Iquadrotor_msgs:/home/bini/versioned/uav_ros/src/quadrotor_sim/quadrotor_msgs/msg -Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg -Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg -Iquadrotor_msgs:/home/bini/versioned/uav_ros/src/quadrotor_sim/quadrotor_msgs/msg -p quadrotor_msgs -o /home/bini/versioned/uav_ros/devel/lib/python2.7/dist-packages/quadrotor_msgs/msg

/home/bini/versioned/uav_ros/devel/lib/python2.7/dist-packages/quadrotor_msgs/msg/_PositionCommand.py: /opt/ros/melodic/lib/genpy/genmsg_py.py
/home/bini/versioned/uav_ros/devel/lib/python2.7/dist-packages/quadrotor_msgs/msg/_PositionCommand.py: /home/bini/versioned/uav_ros/src/quadrotor_sim/quadrotor_msgs/msg/PositionCommand.msg
/home/bini/versioned/uav_ros/devel/lib/python2.7/dist-packages/quadrotor_msgs/msg/_PositionCommand.py: /opt/ros/melodic/share/geometry_msgs/msg/Vector3.msg
/home/bini/versioned/uav_ros/devel/lib/python2.7/dist-packages/quadrotor_msgs/msg/_PositionCommand.py: /opt/ros/melodic/share/geometry_msgs/msg/Point.msg
/home/bini/versioned/uav_ros/devel/lib/python2.7/dist-packages/quadrotor_msgs/msg/_PositionCommand.py: /opt/ros/melodic/share/std_msgs/msg/Header.msg
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/bini/versioned/uav_ros/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Generating Python from MSG quadrotor_msgs/PositionCommand"
	cd /home/bini/versioned/uav_ros/build/quadrotor_sim/quadrotor_msgs && ../../catkin_generated/env_cached.sh /usr/bin/python2 /opt/ros/melodic/share/genpy/cmake/../../../lib/genpy/genmsg_py.py /home/bini/versioned/uav_ros/src/quadrotor_sim/quadrotor_msgs/msg/PositionCommand.msg -Iquadrotor_msgs:/home/bini/versioned/uav_ros/src/quadrotor_sim/quadrotor_msgs/msg -Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg -Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg -Iquadrotor_msgs:/home/bini/versioned/uav_ros/src/quadrotor_sim/quadrotor_msgs/msg -p quadrotor_msgs -o /home/bini/versioned/uav_ros/devel/lib/python2.7/dist-packages/quadrotor_msgs/msg

/home/bini/versioned/uav_ros/devel/lib/python2.7/dist-packages/quadrotor_msgs/msg/_Corrections.py: /opt/ros/melodic/lib/genpy/genmsg_py.py
/home/bini/versioned/uav_ros/devel/lib/python2.7/dist-packages/quadrotor_msgs/msg/_Corrections.py: /home/bini/versioned/uav_ros/src/quadrotor_sim/quadrotor_msgs/msg/Corrections.msg
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/bini/versioned/uav_ros/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Generating Python from MSG quadrotor_msgs/Corrections"
	cd /home/bini/versioned/uav_ros/build/quadrotor_sim/quadrotor_msgs && ../../catkin_generated/env_cached.sh /usr/bin/python2 /opt/ros/melodic/share/genpy/cmake/../../../lib/genpy/genmsg_py.py /home/bini/versioned/uav_ros/src/quadrotor_sim/quadrotor_msgs/msg/Corrections.msg -Iquadrotor_msgs:/home/bini/versioned/uav_ros/src/quadrotor_sim/quadrotor_msgs/msg -Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg -Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg -Iquadrotor_msgs:/home/bini/versioned/uav_ros/src/quadrotor_sim/quadrotor_msgs/msg -p quadrotor_msgs -o /home/bini/versioned/uav_ros/devel/lib/python2.7/dist-packages/quadrotor_msgs/msg

/home/bini/versioned/uav_ros/devel/lib/python2.7/dist-packages/quadrotor_msgs/msg/_LinearCommand.py: /opt/ros/melodic/lib/genpy/genmsg_py.py
/home/bini/versioned/uav_ros/devel/lib/python2.7/dist-packages/quadrotor_msgs/msg/_LinearCommand.py: /home/bini/versioned/uav_ros/src/quadrotor_sim/quadrotor_msgs/msg/LinearCommand.msg
/home/bini/versioned/uav_ros/devel/lib/python2.7/dist-packages/quadrotor_msgs/msg/_LinearCommand.py: /opt/ros/melodic/share/geometry_msgs/msg/Vector3.msg
/home/bini/versioned/uav_ros/devel/lib/python2.7/dist-packages/quadrotor_msgs/msg/_LinearCommand.py: /opt/ros/melodic/share/geometry_msgs/msg/Quaternion.msg
/home/bini/versioned/uav_ros/devel/lib/python2.7/dist-packages/quadrotor_msgs/msg/_LinearCommand.py: /opt/ros/melodic/share/std_msgs/msg/Header.msg
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/bini/versioned/uav_ros/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Generating Python from MSG quadrotor_msgs/LinearCommand"
	cd /home/bini/versioned/uav_ros/build/quadrotor_sim/quadrotor_msgs && ../../catkin_generated/env_cached.sh /usr/bin/python2 /opt/ros/melodic/share/genpy/cmake/../../../lib/genpy/genmsg_py.py /home/bini/versioned/uav_ros/src/quadrotor_sim/quadrotor_msgs/msg/LinearCommand.msg -Iquadrotor_msgs:/home/bini/versioned/uav_ros/src/quadrotor_sim/quadrotor_msgs/msg -Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg -Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg -Iquadrotor_msgs:/home/bini/versioned/uav_ros/src/quadrotor_sim/quadrotor_msgs/msg -p quadrotor_msgs -o /home/bini/versioned/uav_ros/devel/lib/python2.7/dist-packages/quadrotor_msgs/msg

/home/bini/versioned/uav_ros/devel/lib/python2.7/dist-packages/quadrotor_msgs/msg/_SO3Command.py: /opt/ros/melodic/lib/genpy/genmsg_py.py
/home/bini/versioned/uav_ros/devel/lib/python2.7/dist-packages/quadrotor_msgs/msg/_SO3Command.py: /home/bini/versioned/uav_ros/src/quadrotor_sim/quadrotor_msgs/msg/SO3Command.msg
/home/bini/versioned/uav_ros/devel/lib/python2.7/dist-packages/quadrotor_msgs/msg/_SO3Command.py: /home/bini/versioned/uav_ros/src/quadrotor_sim/quadrotor_msgs/msg/AuxCommand.msg
/home/bini/versioned/uav_ros/devel/lib/python2.7/dist-packages/quadrotor_msgs/msg/_SO3Command.py: /opt/ros/melodic/share/geometry_msgs/msg/Vector3.msg
/home/bini/versioned/uav_ros/devel/lib/python2.7/dist-packages/quadrotor_msgs/msg/_SO3Command.py: /opt/ros/melodic/share/geometry_msgs/msg/Quaternion.msg
/home/bini/versioned/uav_ros/devel/lib/python2.7/dist-packages/quadrotor_msgs/msg/_SO3Command.py: /opt/ros/melodic/share/std_msgs/msg/Header.msg
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/bini/versioned/uav_ros/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Generating Python from MSG quadrotor_msgs/SO3Command"
	cd /home/bini/versioned/uav_ros/build/quadrotor_sim/quadrotor_msgs && ../../catkin_generated/env_cached.sh /usr/bin/python2 /opt/ros/melodic/share/genpy/cmake/../../../lib/genpy/genmsg_py.py /home/bini/versioned/uav_ros/src/quadrotor_sim/quadrotor_msgs/msg/SO3Command.msg -Iquadrotor_msgs:/home/bini/versioned/uav_ros/src/quadrotor_sim/quadrotor_msgs/msg -Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg -Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg -Iquadrotor_msgs:/home/bini/versioned/uav_ros/src/quadrotor_sim/quadrotor_msgs/msg -p quadrotor_msgs -o /home/bini/versioned/uav_ros/devel/lib/python2.7/dist-packages/quadrotor_msgs/msg

/home/bini/versioned/uav_ros/devel/lib/python2.7/dist-packages/quadrotor_msgs/msg/_TRPYCommand.py: /opt/ros/melodic/lib/genpy/genmsg_py.py
/home/bini/versioned/uav_ros/devel/lib/python2.7/dist-packages/quadrotor_msgs/msg/_TRPYCommand.py: /home/bini/versioned/uav_ros/src/quadrotor_sim/quadrotor_msgs/msg/TRPYCommand.msg
/home/bini/versioned/uav_ros/devel/lib/python2.7/dist-packages/quadrotor_msgs/msg/_TRPYCommand.py: /opt/ros/melodic/share/std_msgs/msg/Header.msg
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/bini/versioned/uav_ros/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Generating Python from MSG quadrotor_msgs/TRPYCommand"
	cd /home/bini/versioned/uav_ros/build/quadrotor_sim/quadrotor_msgs && ../../catkin_generated/env_cached.sh /usr/bin/python2 /opt/ros/melodic/share/genpy/cmake/../../../lib/genpy/genmsg_py.py /home/bini/versioned/uav_ros/src/quadrotor_sim/quadrotor_msgs/msg/TRPYCommand.msg -Iquadrotor_msgs:/home/bini/versioned/uav_ros/src/quadrotor_sim/quadrotor_msgs/msg -Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg -Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg -Iquadrotor_msgs:/home/bini/versioned/uav_ros/src/quadrotor_sim/quadrotor_msgs/msg -p quadrotor_msgs -o /home/bini/versioned/uav_ros/devel/lib/python2.7/dist-packages/quadrotor_msgs/msg

/home/bini/versioned/uav_ros/devel/lib/python2.7/dist-packages/quadrotor_msgs/msg/_Serial.py: /opt/ros/melodic/lib/genpy/genmsg_py.py
/home/bini/versioned/uav_ros/devel/lib/python2.7/dist-packages/quadrotor_msgs/msg/_Serial.py: /home/bini/versioned/uav_ros/src/quadrotor_sim/quadrotor_msgs/msg/Serial.msg
/home/bini/versioned/uav_ros/devel/lib/python2.7/dist-packages/quadrotor_msgs/msg/_Serial.py: /opt/ros/melodic/share/std_msgs/msg/Header.msg
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/bini/versioned/uav_ros/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Generating Python from MSG quadrotor_msgs/Serial"
	cd /home/bini/versioned/uav_ros/build/quadrotor_sim/quadrotor_msgs && ../../catkin_generated/env_cached.sh /usr/bin/python2 /opt/ros/melodic/share/genpy/cmake/../../../lib/genpy/genmsg_py.py /home/bini/versioned/uav_ros/src/quadrotor_sim/quadrotor_msgs/msg/Serial.msg -Iquadrotor_msgs:/home/bini/versioned/uav_ros/src/quadrotor_sim/quadrotor_msgs/msg -Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg -Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg -Iquadrotor_msgs:/home/bini/versioned/uav_ros/src/quadrotor_sim/quadrotor_msgs/msg -p quadrotor_msgs -o /home/bini/versioned/uav_ros/devel/lib/python2.7/dist-packages/quadrotor_msgs/msg

/home/bini/versioned/uav_ros/devel/lib/python2.7/dist-packages/quadrotor_msgs/msg/_AuxCommand.py: /opt/ros/melodic/lib/genpy/genmsg_py.py
/home/bini/versioned/uav_ros/devel/lib/python2.7/dist-packages/quadrotor_msgs/msg/_AuxCommand.py: /home/bini/versioned/uav_ros/src/quadrotor_sim/quadrotor_msgs/msg/AuxCommand.msg
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/bini/versioned/uav_ros/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Generating Python from MSG quadrotor_msgs/AuxCommand"
	cd /home/bini/versioned/uav_ros/build/quadrotor_sim/quadrotor_msgs && ../../catkin_generated/env_cached.sh /usr/bin/python2 /opt/ros/melodic/share/genpy/cmake/../../../lib/genpy/genmsg_py.py /home/bini/versioned/uav_ros/src/quadrotor_sim/quadrotor_msgs/msg/AuxCommand.msg -Iquadrotor_msgs:/home/bini/versioned/uav_ros/src/quadrotor_sim/quadrotor_msgs/msg -Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg -Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg -Iquadrotor_msgs:/home/bini/versioned/uav_ros/src/quadrotor_sim/quadrotor_msgs/msg -p quadrotor_msgs -o /home/bini/versioned/uav_ros/devel/lib/python2.7/dist-packages/quadrotor_msgs/msg

/home/bini/versioned/uav_ros/devel/lib/python2.7/dist-packages/quadrotor_msgs/msg/__init__.py: /opt/ros/melodic/lib/genpy/genmsg_py.py
/home/bini/versioned/uav_ros/devel/lib/python2.7/dist-packages/quadrotor_msgs/msg/__init__.py: /home/bini/versioned/uav_ros/devel/lib/python2.7/dist-packages/quadrotor_msgs/msg/_OutputData.py
/home/bini/versioned/uav_ros/devel/lib/python2.7/dist-packages/quadrotor_msgs/msg/__init__.py: /home/bini/versioned/uav_ros/devel/lib/python2.7/dist-packages/quadrotor_msgs/msg/_StatusData.py
/home/bini/versioned/uav_ros/devel/lib/python2.7/dist-packages/quadrotor_msgs/msg/__init__.py: /home/bini/versioned/uav_ros/devel/lib/python2.7/dist-packages/quadrotor_msgs/msg/_PositionCommand.py
/home/bini/versioned/uav_ros/devel/lib/python2.7/dist-packages/quadrotor_msgs/msg/__init__.py: /home/bini/versioned/uav_ros/devel/lib/python2.7/dist-packages/quadrotor_msgs/msg/_Corrections.py
/home/bini/versioned/uav_ros/devel/lib/python2.7/dist-packages/quadrotor_msgs/msg/__init__.py: /home/bini/versioned/uav_ros/devel/lib/python2.7/dist-packages/quadrotor_msgs/msg/_LinearCommand.py
/home/bini/versioned/uav_ros/devel/lib/python2.7/dist-packages/quadrotor_msgs/msg/__init__.py: /home/bini/versioned/uav_ros/devel/lib/python2.7/dist-packages/quadrotor_msgs/msg/_SO3Command.py
/home/bini/versioned/uav_ros/devel/lib/python2.7/dist-packages/quadrotor_msgs/msg/__init__.py: /home/bini/versioned/uav_ros/devel/lib/python2.7/dist-packages/quadrotor_msgs/msg/_TRPYCommand.py
/home/bini/versioned/uav_ros/devel/lib/python2.7/dist-packages/quadrotor_msgs/msg/__init__.py: /home/bini/versioned/uav_ros/devel/lib/python2.7/dist-packages/quadrotor_msgs/msg/_Serial.py
/home/bini/versioned/uav_ros/devel/lib/python2.7/dist-packages/quadrotor_msgs/msg/__init__.py: /home/bini/versioned/uav_ros/devel/lib/python2.7/dist-packages/quadrotor_msgs/msg/_AuxCommand.py
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/bini/versioned/uav_ros/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Generating Python msg __init__.py for quadrotor_msgs"
	cd /home/bini/versioned/uav_ros/build/quadrotor_sim/quadrotor_msgs && ../../catkin_generated/env_cached.sh /usr/bin/python2 /opt/ros/melodic/share/genpy/cmake/../../../lib/genpy/genmsg_py.py -o /home/bini/versioned/uav_ros/devel/lib/python2.7/dist-packages/quadrotor_msgs/msg --initpy

quadrotor_msgs_generate_messages_py: quadrotor_sim/quadrotor_msgs/CMakeFiles/quadrotor_msgs_generate_messages_py
quadrotor_msgs_generate_messages_py: /home/bini/versioned/uav_ros/devel/lib/python2.7/dist-packages/quadrotor_msgs/msg/_OutputData.py
quadrotor_msgs_generate_messages_py: /home/bini/versioned/uav_ros/devel/lib/python2.7/dist-packages/quadrotor_msgs/msg/_StatusData.py
quadrotor_msgs_generate_messages_py: /home/bini/versioned/uav_ros/devel/lib/python2.7/dist-packages/quadrotor_msgs/msg/_PositionCommand.py
quadrotor_msgs_generate_messages_py: /home/bini/versioned/uav_ros/devel/lib/python2.7/dist-packages/quadrotor_msgs/msg/_Corrections.py
quadrotor_msgs_generate_messages_py: /home/bini/versioned/uav_ros/devel/lib/python2.7/dist-packages/quadrotor_msgs/msg/_LinearCommand.py
quadrotor_msgs_generate_messages_py: /home/bini/versioned/uav_ros/devel/lib/python2.7/dist-packages/quadrotor_msgs/msg/_SO3Command.py
quadrotor_msgs_generate_messages_py: /home/bini/versioned/uav_ros/devel/lib/python2.7/dist-packages/quadrotor_msgs/msg/_TRPYCommand.py
quadrotor_msgs_generate_messages_py: /home/bini/versioned/uav_ros/devel/lib/python2.7/dist-packages/quadrotor_msgs/msg/_Serial.py
quadrotor_msgs_generate_messages_py: /home/bini/versioned/uav_ros/devel/lib/python2.7/dist-packages/quadrotor_msgs/msg/_AuxCommand.py
quadrotor_msgs_generate_messages_py: /home/bini/versioned/uav_ros/devel/lib/python2.7/dist-packages/quadrotor_msgs/msg/__init__.py
quadrotor_msgs_generate_messages_py: quadrotor_sim/quadrotor_msgs/CMakeFiles/quadrotor_msgs_generate_messages_py.dir/build.make

.PHONY : quadrotor_msgs_generate_messages_py

# Rule to build all files generated by this target.
quadrotor_sim/quadrotor_msgs/CMakeFiles/quadrotor_msgs_generate_messages_py.dir/build: quadrotor_msgs_generate_messages_py

.PHONY : quadrotor_sim/quadrotor_msgs/CMakeFiles/quadrotor_msgs_generate_messages_py.dir/build

quadrotor_sim/quadrotor_msgs/CMakeFiles/quadrotor_msgs_generate_messages_py.dir/clean:
	cd /home/bini/versioned/uav_ros/build/quadrotor_sim/quadrotor_msgs && $(CMAKE_COMMAND) -P CMakeFiles/quadrotor_msgs_generate_messages_py.dir/cmake_clean.cmake
.PHONY : quadrotor_sim/quadrotor_msgs/CMakeFiles/quadrotor_msgs_generate_messages_py.dir/clean

quadrotor_sim/quadrotor_msgs/CMakeFiles/quadrotor_msgs_generate_messages_py.dir/depend:
	cd /home/bini/versioned/uav_ros/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/bini/versioned/uav_ros/src /home/bini/versioned/uav_ros/src/quadrotor_sim/quadrotor_msgs /home/bini/versioned/uav_ros/build /home/bini/versioned/uav_ros/build/quadrotor_sim/quadrotor_msgs /home/bini/versioned/uav_ros/build/quadrotor_sim/quadrotor_msgs/CMakeFiles/quadrotor_msgs_generate_messages_py.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : quadrotor_sim/quadrotor_msgs/CMakeFiles/quadrotor_msgs_generate_messages_py.dir/depend

