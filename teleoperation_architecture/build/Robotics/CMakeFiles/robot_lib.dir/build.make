# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.24

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/li/internet/teleoperation_architecture

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/li/internet/teleoperation_architecture/build

# Include any dependencies generated for this target.
include Robotics/CMakeFiles/robot_lib.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include Robotics/CMakeFiles/robot_lib.dir/compiler_depend.make

# Include the progress variables for this target.
include Robotics/CMakeFiles/robot_lib.dir/progress.make

# Include the compile flags for this target's objects.
include Robotics/CMakeFiles/robot_lib.dir/flags.make

Robotics/CMakeFiles/robot_lib.dir/Control/PID/PIDController.cpp.o: Robotics/CMakeFiles/robot_lib.dir/flags.make
Robotics/CMakeFiles/robot_lib.dir/Control/PID/PIDController.cpp.o: /home/li/internet/teleoperation_architecture/Robotics/Control/PID/PIDController.cpp
Robotics/CMakeFiles/robot_lib.dir/Control/PID/PIDController.cpp.o: Robotics/CMakeFiles/robot_lib.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/li/internet/teleoperation_architecture/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object Robotics/CMakeFiles/robot_lib.dir/Control/PID/PIDController.cpp.o"
	cd /home/li/internet/teleoperation_architecture/build/Robotics && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT Robotics/CMakeFiles/robot_lib.dir/Control/PID/PIDController.cpp.o -MF CMakeFiles/robot_lib.dir/Control/PID/PIDController.cpp.o.d -o CMakeFiles/robot_lib.dir/Control/PID/PIDController.cpp.o -c /home/li/internet/teleoperation_architecture/Robotics/Control/PID/PIDController.cpp

Robotics/CMakeFiles/robot_lib.dir/Control/PID/PIDController.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/robot_lib.dir/Control/PID/PIDController.cpp.i"
	cd /home/li/internet/teleoperation_architecture/build/Robotics && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/li/internet/teleoperation_architecture/Robotics/Control/PID/PIDController.cpp > CMakeFiles/robot_lib.dir/Control/PID/PIDController.cpp.i

Robotics/CMakeFiles/robot_lib.dir/Control/PID/PIDController.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/robot_lib.dir/Control/PID/PIDController.cpp.s"
	cd /home/li/internet/teleoperation_architecture/build/Robotics && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/li/internet/teleoperation_architecture/Robotics/Control/PID/PIDController.cpp -o CMakeFiles/robot_lib.dir/Control/PID/PIDController.cpp.s

Robotics/CMakeFiles/robot_lib.dir/RobotAdaptor/Force_dimension_robot/Falcon_robot_fd.cpp.o: Robotics/CMakeFiles/robot_lib.dir/flags.make
Robotics/CMakeFiles/robot_lib.dir/RobotAdaptor/Force_dimension_robot/Falcon_robot_fd.cpp.o: /home/li/internet/teleoperation_architecture/Robotics/RobotAdaptor/Force_dimension_robot/Falcon_robot_fd.cpp
Robotics/CMakeFiles/robot_lib.dir/RobotAdaptor/Force_dimension_robot/Falcon_robot_fd.cpp.o: Robotics/CMakeFiles/robot_lib.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/li/internet/teleoperation_architecture/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object Robotics/CMakeFiles/robot_lib.dir/RobotAdaptor/Force_dimension_robot/Falcon_robot_fd.cpp.o"
	cd /home/li/internet/teleoperation_architecture/build/Robotics && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT Robotics/CMakeFiles/robot_lib.dir/RobotAdaptor/Force_dimension_robot/Falcon_robot_fd.cpp.o -MF CMakeFiles/robot_lib.dir/RobotAdaptor/Force_dimension_robot/Falcon_robot_fd.cpp.o.d -o CMakeFiles/robot_lib.dir/RobotAdaptor/Force_dimension_robot/Falcon_robot_fd.cpp.o -c /home/li/internet/teleoperation_architecture/Robotics/RobotAdaptor/Force_dimension_robot/Falcon_robot_fd.cpp

Robotics/CMakeFiles/robot_lib.dir/RobotAdaptor/Force_dimension_robot/Falcon_robot_fd.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/robot_lib.dir/RobotAdaptor/Force_dimension_robot/Falcon_robot_fd.cpp.i"
	cd /home/li/internet/teleoperation_architecture/build/Robotics && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/li/internet/teleoperation_architecture/Robotics/RobotAdaptor/Force_dimension_robot/Falcon_robot_fd.cpp > CMakeFiles/robot_lib.dir/RobotAdaptor/Force_dimension_robot/Falcon_robot_fd.cpp.i

Robotics/CMakeFiles/robot_lib.dir/RobotAdaptor/Force_dimension_robot/Falcon_robot_fd.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/robot_lib.dir/RobotAdaptor/Force_dimension_robot/Falcon_robot_fd.cpp.s"
	cd /home/li/internet/teleoperation_architecture/build/Robotics && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/li/internet/teleoperation_architecture/Robotics/RobotAdaptor/Force_dimension_robot/Falcon_robot_fd.cpp -o CMakeFiles/robot_lib.dir/RobotAdaptor/Force_dimension_robot/Falcon_robot_fd.cpp.s

# Object files for target robot_lib
robot_lib_OBJECTS = \
"CMakeFiles/robot_lib.dir/Control/PID/PIDController.cpp.o" \
"CMakeFiles/robot_lib.dir/RobotAdaptor/Force_dimension_robot/Falcon_robot_fd.cpp.o"

# External object files for target robot_lib
robot_lib_EXTERNAL_OBJECTS =

Robotics/librobot_lib.a: Robotics/CMakeFiles/robot_lib.dir/Control/PID/PIDController.cpp.o
Robotics/librobot_lib.a: Robotics/CMakeFiles/robot_lib.dir/RobotAdaptor/Force_dimension_robot/Falcon_robot_fd.cpp.o
Robotics/librobot_lib.a: Robotics/CMakeFiles/robot_lib.dir/build.make
Robotics/librobot_lib.a: Robotics/CMakeFiles/robot_lib.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/li/internet/teleoperation_architecture/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX static library librobot_lib.a"
	cd /home/li/internet/teleoperation_architecture/build/Robotics && $(CMAKE_COMMAND) -P CMakeFiles/robot_lib.dir/cmake_clean_target.cmake
	cd /home/li/internet/teleoperation_architecture/build/Robotics && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/robot_lib.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
Robotics/CMakeFiles/robot_lib.dir/build: Robotics/librobot_lib.a
.PHONY : Robotics/CMakeFiles/robot_lib.dir/build

Robotics/CMakeFiles/robot_lib.dir/clean:
	cd /home/li/internet/teleoperation_architecture/build/Robotics && $(CMAKE_COMMAND) -P CMakeFiles/robot_lib.dir/cmake_clean.cmake
.PHONY : Robotics/CMakeFiles/robot_lib.dir/clean

Robotics/CMakeFiles/robot_lib.dir/depend:
	cd /home/li/internet/teleoperation_architecture/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/li/internet/teleoperation_architecture /home/li/internet/teleoperation_architecture/Robotics /home/li/internet/teleoperation_architecture/build /home/li/internet/teleoperation_architecture/build/Robotics /home/li/internet/teleoperation_architecture/build/Robotics/CMakeFiles/robot_lib.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : Robotics/CMakeFiles/robot_lib.dir/depend

