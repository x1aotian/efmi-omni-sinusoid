# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.17

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
CMAKE_COMMAND = /home/child1/SomeApps/clion-2020.2.4/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /home/child1/SomeApps/clion-2020.2.4/bin/cmake/linux/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/child1/MARS/efmi-omni-sinusoid

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/child1/MARS/efmi-omni-sinusoid/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/OmniSineBase.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/OmniSineBase.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/OmniSineBase.dir/flags.make

CMakeFiles/OmniSineBase.dir/src/CameraModel.cpp.o: CMakeFiles/OmniSineBase.dir/flags.make
CMakeFiles/OmniSineBase.dir/src/CameraModel.cpp.o: ../src/CameraModel.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/child1/MARS/efmi-omni-sinusoid/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/OmniSineBase.dir/src/CameraModel.cpp.o"
	g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/OmniSineBase.dir/src/CameraModel.cpp.o -c /home/child1/MARS/efmi-omni-sinusoid/src/CameraModel.cpp

CMakeFiles/OmniSineBase.dir/src/CameraModel.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/OmniSineBase.dir/src/CameraModel.cpp.i"
	g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/child1/MARS/efmi-omni-sinusoid/src/CameraModel.cpp > CMakeFiles/OmniSineBase.dir/src/CameraModel.cpp.i

CMakeFiles/OmniSineBase.dir/src/CameraModel.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/OmniSineBase.dir/src/CameraModel.cpp.s"
	g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/child1/MARS/efmi-omni-sinusoid/src/CameraModel.cpp -o CMakeFiles/OmniSineBase.dir/src/CameraModel.cpp.s

CMakeFiles/OmniSineBase.dir/src/OCamModel.cpp.o: CMakeFiles/OmniSineBase.dir/flags.make
CMakeFiles/OmniSineBase.dir/src/OCamModel.cpp.o: ../src/OCamModel.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/child1/MARS/efmi-omni-sinusoid/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/OmniSineBase.dir/src/OCamModel.cpp.o"
	g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/OmniSineBase.dir/src/OCamModel.cpp.o -c /home/child1/MARS/efmi-omni-sinusoid/src/OCamModel.cpp

CMakeFiles/OmniSineBase.dir/src/OCamModel.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/OmniSineBase.dir/src/OCamModel.cpp.i"
	g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/child1/MARS/efmi-omni-sinusoid/src/OCamModel.cpp > CMakeFiles/OmniSineBase.dir/src/OCamModel.cpp.i

CMakeFiles/OmniSineBase.dir/src/OCamModel.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/OmniSineBase.dir/src/OCamModel.cpp.s"
	g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/child1/MARS/efmi-omni-sinusoid/src/OCamModel.cpp -o CMakeFiles/OmniSineBase.dir/src/OCamModel.cpp.s

CMakeFiles/OmniSineBase.dir/src/PCamModel.cpp.o: CMakeFiles/OmniSineBase.dir/flags.make
CMakeFiles/OmniSineBase.dir/src/PCamModel.cpp.o: ../src/PCamModel.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/child1/MARS/efmi-omni-sinusoid/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/OmniSineBase.dir/src/PCamModel.cpp.o"
	g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/OmniSineBase.dir/src/PCamModel.cpp.o -c /home/child1/MARS/efmi-omni-sinusoid/src/PCamModel.cpp

CMakeFiles/OmniSineBase.dir/src/PCamModel.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/OmniSineBase.dir/src/PCamModel.cpp.i"
	g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/child1/MARS/efmi-omni-sinusoid/src/PCamModel.cpp > CMakeFiles/OmniSineBase.dir/src/PCamModel.cpp.i

CMakeFiles/OmniSineBase.dir/src/PCamModel.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/OmniSineBase.dir/src/PCamModel.cpp.s"
	g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/child1/MARS/efmi-omni-sinusoid/src/PCamModel.cpp -o CMakeFiles/OmniSineBase.dir/src/PCamModel.cpp.s

CMakeFiles/OmniSineBase.dir/src/CCamModel.cpp.o: CMakeFiles/OmniSineBase.dir/flags.make
CMakeFiles/OmniSineBase.dir/src/CCamModel.cpp.o: ../src/CCamModel.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/child1/MARS/efmi-omni-sinusoid/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/OmniSineBase.dir/src/CCamModel.cpp.o"
	g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/OmniSineBase.dir/src/CCamModel.cpp.o -c /home/child1/MARS/efmi-omni-sinusoid/src/CCamModel.cpp

CMakeFiles/OmniSineBase.dir/src/CCamModel.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/OmniSineBase.dir/src/CCamModel.cpp.i"
	g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/child1/MARS/efmi-omni-sinusoid/src/CCamModel.cpp > CMakeFiles/OmniSineBase.dir/src/CCamModel.cpp.i

CMakeFiles/OmniSineBase.dir/src/CCamModel.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/OmniSineBase.dir/src/CCamModel.cpp.s"
	g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/child1/MARS/efmi-omni-sinusoid/src/CCamModel.cpp -o CMakeFiles/OmniSineBase.dir/src/CCamModel.cpp.s

CMakeFiles/OmniSineBase.dir/src/Frame.cpp.o: CMakeFiles/OmniSineBase.dir/flags.make
CMakeFiles/OmniSineBase.dir/src/Frame.cpp.o: ../src/Frame.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/child1/MARS/efmi-omni-sinusoid/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/OmniSineBase.dir/src/Frame.cpp.o"
	g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/OmniSineBase.dir/src/Frame.cpp.o -c /home/child1/MARS/efmi-omni-sinusoid/src/Frame.cpp

CMakeFiles/OmniSineBase.dir/src/Frame.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/OmniSineBase.dir/src/Frame.cpp.i"
	g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/child1/MARS/efmi-omni-sinusoid/src/Frame.cpp > CMakeFiles/OmniSineBase.dir/src/Frame.cpp.i

CMakeFiles/OmniSineBase.dir/src/Frame.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/OmniSineBase.dir/src/Frame.cpp.s"
	g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/child1/MARS/efmi-omni-sinusoid/src/Frame.cpp -o CMakeFiles/OmniSineBase.dir/src/Frame.cpp.s

CMakeFiles/OmniSineBase.dir/src/CFrame.cpp.o: CMakeFiles/OmniSineBase.dir/flags.make
CMakeFiles/OmniSineBase.dir/src/CFrame.cpp.o: ../src/CFrame.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/child1/MARS/efmi-omni-sinusoid/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/OmniSineBase.dir/src/CFrame.cpp.o"
	g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/OmniSineBase.dir/src/CFrame.cpp.o -c /home/child1/MARS/efmi-omni-sinusoid/src/CFrame.cpp

CMakeFiles/OmniSineBase.dir/src/CFrame.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/OmniSineBase.dir/src/CFrame.cpp.i"
	g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/child1/MARS/efmi-omni-sinusoid/src/CFrame.cpp > CMakeFiles/OmniSineBase.dir/src/CFrame.cpp.i

CMakeFiles/OmniSineBase.dir/src/CFrame.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/OmniSineBase.dir/src/CFrame.cpp.s"
	g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/child1/MARS/efmi-omni-sinusoid/src/CFrame.cpp -o CMakeFiles/OmniSineBase.dir/src/CFrame.cpp.s

CMakeFiles/OmniSineBase.dir/src/Tracking.cpp.o: CMakeFiles/OmniSineBase.dir/flags.make
CMakeFiles/OmniSineBase.dir/src/Tracking.cpp.o: ../src/Tracking.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/child1/MARS/efmi-omni-sinusoid/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object CMakeFiles/OmniSineBase.dir/src/Tracking.cpp.o"
	g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/OmniSineBase.dir/src/Tracking.cpp.o -c /home/child1/MARS/efmi-omni-sinusoid/src/Tracking.cpp

CMakeFiles/OmniSineBase.dir/src/Tracking.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/OmniSineBase.dir/src/Tracking.cpp.i"
	g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/child1/MARS/efmi-omni-sinusoid/src/Tracking.cpp > CMakeFiles/OmniSineBase.dir/src/Tracking.cpp.i

CMakeFiles/OmniSineBase.dir/src/Tracking.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/OmniSineBase.dir/src/Tracking.cpp.s"
	g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/child1/MARS/efmi-omni-sinusoid/src/Tracking.cpp -o CMakeFiles/OmniSineBase.dir/src/Tracking.cpp.s

CMakeFiles/OmniSineBase.dir/src/CTracking.cpp.o: CMakeFiles/OmniSineBase.dir/flags.make
CMakeFiles/OmniSineBase.dir/src/CTracking.cpp.o: ../src/CTracking.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/child1/MARS/efmi-omni-sinusoid/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object CMakeFiles/OmniSineBase.dir/src/CTracking.cpp.o"
	g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/OmniSineBase.dir/src/CTracking.cpp.o -c /home/child1/MARS/efmi-omni-sinusoid/src/CTracking.cpp

CMakeFiles/OmniSineBase.dir/src/CTracking.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/OmniSineBase.dir/src/CTracking.cpp.i"
	g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/child1/MARS/efmi-omni-sinusoid/src/CTracking.cpp > CMakeFiles/OmniSineBase.dir/src/CTracking.cpp.i

CMakeFiles/OmniSineBase.dir/src/CTracking.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/OmniSineBase.dir/src/CTracking.cpp.s"
	g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/child1/MARS/efmi-omni-sinusoid/src/CTracking.cpp -o CMakeFiles/OmniSineBase.dir/src/CTracking.cpp.s

CMakeFiles/OmniSineBase.dir/src/CSystem.cpp.o: CMakeFiles/OmniSineBase.dir/flags.make
CMakeFiles/OmniSineBase.dir/src/CSystem.cpp.o: ../src/CSystem.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/child1/MARS/efmi-omni-sinusoid/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building CXX object CMakeFiles/OmniSineBase.dir/src/CSystem.cpp.o"
	g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/OmniSineBase.dir/src/CSystem.cpp.o -c /home/child1/MARS/efmi-omni-sinusoid/src/CSystem.cpp

CMakeFiles/OmniSineBase.dir/src/CSystem.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/OmniSineBase.dir/src/CSystem.cpp.i"
	g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/child1/MARS/efmi-omni-sinusoid/src/CSystem.cpp > CMakeFiles/OmniSineBase.dir/src/CSystem.cpp.i

CMakeFiles/OmniSineBase.dir/src/CSystem.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/OmniSineBase.dir/src/CSystem.cpp.s"
	g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/child1/MARS/efmi-omni-sinusoid/src/CSystem.cpp -o CMakeFiles/OmniSineBase.dir/src/CSystem.cpp.s

CMakeFiles/OmniSineBase.dir/src/misc_1.cpp.o: CMakeFiles/OmniSineBase.dir/flags.make
CMakeFiles/OmniSineBase.dir/src/misc_1.cpp.o: ../src/misc_1.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/child1/MARS/efmi-omni-sinusoid/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Building CXX object CMakeFiles/OmniSineBase.dir/src/misc_1.cpp.o"
	g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/OmniSineBase.dir/src/misc_1.cpp.o -c /home/child1/MARS/efmi-omni-sinusoid/src/misc_1.cpp

CMakeFiles/OmniSineBase.dir/src/misc_1.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/OmniSineBase.dir/src/misc_1.cpp.i"
	g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/child1/MARS/efmi-omni-sinusoid/src/misc_1.cpp > CMakeFiles/OmniSineBase.dir/src/misc_1.cpp.i

CMakeFiles/OmniSineBase.dir/src/misc_1.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/OmniSineBase.dir/src/misc_1.cpp.s"
	g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/child1/MARS/efmi-omni-sinusoid/src/misc_1.cpp -o CMakeFiles/OmniSineBase.dir/src/misc_1.cpp.s

# Object files for target OmniSineBase
OmniSineBase_OBJECTS = \
"CMakeFiles/OmniSineBase.dir/src/CameraModel.cpp.o" \
"CMakeFiles/OmniSineBase.dir/src/OCamModel.cpp.o" \
"CMakeFiles/OmniSineBase.dir/src/PCamModel.cpp.o" \
"CMakeFiles/OmniSineBase.dir/src/CCamModel.cpp.o" \
"CMakeFiles/OmniSineBase.dir/src/Frame.cpp.o" \
"CMakeFiles/OmniSineBase.dir/src/CFrame.cpp.o" \
"CMakeFiles/OmniSineBase.dir/src/Tracking.cpp.o" \
"CMakeFiles/OmniSineBase.dir/src/CTracking.cpp.o" \
"CMakeFiles/OmniSineBase.dir/src/CSystem.cpp.o" \
"CMakeFiles/OmniSineBase.dir/src/misc_1.cpp.o"

# External object files for target OmniSineBase
OmniSineBase_EXTERNAL_OBJECTS =

../lib/libOmniSineBase.a: CMakeFiles/OmniSineBase.dir/src/CameraModel.cpp.o
../lib/libOmniSineBase.a: CMakeFiles/OmniSineBase.dir/src/OCamModel.cpp.o
../lib/libOmniSineBase.a: CMakeFiles/OmniSineBase.dir/src/PCamModel.cpp.o
../lib/libOmniSineBase.a: CMakeFiles/OmniSineBase.dir/src/CCamModel.cpp.o
../lib/libOmniSineBase.a: CMakeFiles/OmniSineBase.dir/src/Frame.cpp.o
../lib/libOmniSineBase.a: CMakeFiles/OmniSineBase.dir/src/CFrame.cpp.o
../lib/libOmniSineBase.a: CMakeFiles/OmniSineBase.dir/src/Tracking.cpp.o
../lib/libOmniSineBase.a: CMakeFiles/OmniSineBase.dir/src/CTracking.cpp.o
../lib/libOmniSineBase.a: CMakeFiles/OmniSineBase.dir/src/CSystem.cpp.o
../lib/libOmniSineBase.a: CMakeFiles/OmniSineBase.dir/src/misc_1.cpp.o
../lib/libOmniSineBase.a: CMakeFiles/OmniSineBase.dir/build.make
../lib/libOmniSineBase.a: CMakeFiles/OmniSineBase.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/child1/MARS/efmi-omni-sinusoid/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_11) "Linking CXX static library ../lib/libOmniSineBase.a"
	$(CMAKE_COMMAND) -P CMakeFiles/OmniSineBase.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/OmniSineBase.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/OmniSineBase.dir/build: ../lib/libOmniSineBase.a

.PHONY : CMakeFiles/OmniSineBase.dir/build

CMakeFiles/OmniSineBase.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/OmniSineBase.dir/cmake_clean.cmake
.PHONY : CMakeFiles/OmniSineBase.dir/clean

CMakeFiles/OmniSineBase.dir/depend:
	cd /home/child1/MARS/efmi-omni-sinusoid/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/child1/MARS/efmi-omni-sinusoid /home/child1/MARS/efmi-omni-sinusoid /home/child1/MARS/efmi-omni-sinusoid/cmake-build-debug /home/child1/MARS/efmi-omni-sinusoid/cmake-build-debug /home/child1/MARS/efmi-omni-sinusoid/cmake-build-debug/CMakeFiles/OmniSineBase.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/OmniSineBase.dir/depend

