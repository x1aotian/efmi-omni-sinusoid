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
include CMakeFiles/TransFlow.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/TransFlow.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/TransFlow.dir/flags.make

CMakeFiles/TransFlow.dir/test/main_transflow.cpp.o: CMakeFiles/TransFlow.dir/flags.make
CMakeFiles/TransFlow.dir/test/main_transflow.cpp.o: ../test/main_transflow.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/child1/MARS/efmi-omni-sinusoid/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/TransFlow.dir/test/main_transflow.cpp.o"
	g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/TransFlow.dir/test/main_transflow.cpp.o -c /home/child1/MARS/efmi-omni-sinusoid/test/main_transflow.cpp

CMakeFiles/TransFlow.dir/test/main_transflow.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/TransFlow.dir/test/main_transflow.cpp.i"
	g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/child1/MARS/efmi-omni-sinusoid/test/main_transflow.cpp > CMakeFiles/TransFlow.dir/test/main_transflow.cpp.i

CMakeFiles/TransFlow.dir/test/main_transflow.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/TransFlow.dir/test/main_transflow.cpp.s"
	g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/child1/MARS/efmi-omni-sinusoid/test/main_transflow.cpp -o CMakeFiles/TransFlow.dir/test/main_transflow.cpp.s

# Object files for target TransFlow
TransFlow_OBJECTS = \
"CMakeFiles/TransFlow.dir/test/main_transflow.cpp.o"

# External object files for target TransFlow
TransFlow_EXTERNAL_OBJECTS =

../bin/TransFlow: CMakeFiles/TransFlow.dir/test/main_transflow.cpp.o
../bin/TransFlow: CMakeFiles/TransFlow.dir/build.make
../bin/TransFlow: ../lib/libOmniSineBase.a
../bin/TransFlow: ../lib/libFMIBase.a
../bin/TransFlow: /usr/lib/x86_64-linux-gnu/libQt5Widgets.so.5.12.8
../bin/TransFlow: /usr/lib/x86_64-linux-gnu/libQt5Gui.so.5.12.8
../bin/TransFlow: /usr/lib/x86_64-linux-gnu/libQt5Core.so.5.12.8
../bin/TransFlow: /usr/lib/x86_64-linux-gnu/libgsl.so
../bin/TransFlow: /usr/lib/x86_64-linux-gnu/libgslcblas.so
../bin/TransFlow: /usr/local/lib/libopencv_dnn.so.4.4.0
../bin/TransFlow: /usr/local/lib/libopencv_gapi.so.4.4.0
../bin/TransFlow: /usr/local/lib/libopencv_highgui.so.4.4.0
../bin/TransFlow: /usr/local/lib/libopencv_ml.so.4.4.0
../bin/TransFlow: /usr/local/lib/libopencv_objdetect.so.4.4.0
../bin/TransFlow: /usr/local/lib/libopencv_photo.so.4.4.0
../bin/TransFlow: /usr/local/lib/libopencv_stitching.so.4.4.0
../bin/TransFlow: /usr/local/lib/libopencv_video.so.4.4.0
../bin/TransFlow: /usr/local/lib/libopencv_calib3d.so.4.4.0
../bin/TransFlow: /usr/local/lib/libopencv_features2d.so.4.4.0
../bin/TransFlow: /usr/local/lib/libopencv_flann.so.4.4.0
../bin/TransFlow: /usr/local/lib/libopencv_videoio.so.4.4.0
../bin/TransFlow: /usr/local/lib/libopencv_imgcodecs.so.4.4.0
../bin/TransFlow: /usr/local/lib/libopencv_imgproc.so.4.4.0
../bin/TransFlow: /usr/local/lib/libopencv_core.so.4.4.0
../bin/TransFlow: /usr/local/lib/libceres.a
../bin/TransFlow: /usr/lib/x86_64-linux-gnu/libglog.so
../bin/TransFlow: /usr/lib/x86_64-linux-gnu/libgflags.so.2.2.2
../bin/TransFlow: /usr/lib/x86_64-linux-gnu/libspqr.so
../bin/TransFlow: /usr/lib/x86_64-linux-gnu/libtbbmalloc.so
../bin/TransFlow: /usr/lib/x86_64-linux-gnu/libtbb.so
../bin/TransFlow: /usr/lib/x86_64-linux-gnu/libcholmod.so
../bin/TransFlow: /usr/lib/x86_64-linux-gnu/libccolamd.so
../bin/TransFlow: /usr/lib/x86_64-linux-gnu/libcamd.so
../bin/TransFlow: /usr/lib/x86_64-linux-gnu/libcolamd.so
../bin/TransFlow: /usr/lib/x86_64-linux-gnu/libamd.so
../bin/TransFlow: /usr/lib/x86_64-linux-gnu/liblapack.so
../bin/TransFlow: /usr/lib/x86_64-linux-gnu/libf77blas.so
../bin/TransFlow: /usr/lib/x86_64-linux-gnu/libatlas.so
../bin/TransFlow: /usr/lib/x86_64-linux-gnu/libsuitesparseconfig.so
../bin/TransFlow: /usr/lib/x86_64-linux-gnu/librt.so
../bin/TransFlow: /usr/lib/x86_64-linux-gnu/libcxsparse.so
../bin/TransFlow: /usr/lib/x86_64-linux-gnu/libtbbmalloc.so
../bin/TransFlow: /usr/lib/x86_64-linux-gnu/libtbb.so
../bin/TransFlow: /usr/lib/x86_64-linux-gnu/libcholmod.so
../bin/TransFlow: /usr/lib/x86_64-linux-gnu/libccolamd.so
../bin/TransFlow: /usr/lib/x86_64-linux-gnu/libcamd.so
../bin/TransFlow: /usr/lib/x86_64-linux-gnu/libcolamd.so
../bin/TransFlow: /usr/lib/x86_64-linux-gnu/libamd.so
../bin/TransFlow: /usr/lib/x86_64-linux-gnu/liblapack.so
../bin/TransFlow: /usr/lib/x86_64-linux-gnu/libf77blas.so
../bin/TransFlow: /usr/lib/x86_64-linux-gnu/libatlas.so
../bin/TransFlow: /usr/lib/x86_64-linux-gnu/libsuitesparseconfig.so
../bin/TransFlow: /usr/lib/x86_64-linux-gnu/librt.so
../bin/TransFlow: /usr/lib/x86_64-linux-gnu/libcxsparse.so
../bin/TransFlow: CMakeFiles/TransFlow.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/child1/MARS/efmi-omni-sinusoid/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../bin/TransFlow"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/TransFlow.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/TransFlow.dir/build: ../bin/TransFlow

.PHONY : CMakeFiles/TransFlow.dir/build

CMakeFiles/TransFlow.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/TransFlow.dir/cmake_clean.cmake
.PHONY : CMakeFiles/TransFlow.dir/clean

CMakeFiles/TransFlow.dir/depend:
	cd /home/child1/MARS/efmi-omni-sinusoid/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/child1/MARS/efmi-omni-sinusoid /home/child1/MARS/efmi-omni-sinusoid /home/child1/MARS/efmi-omni-sinusoid/cmake-build-debug /home/child1/MARS/efmi-omni-sinusoid/cmake-build-debug /home/child1/MARS/efmi-omni-sinusoid/cmake-build-debug/CMakeFiles/TransFlow.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/TransFlow.dir/depend

