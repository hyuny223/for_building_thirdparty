# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_SOURCE_DIR = /team-SLAM

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /team-SLAM/build

# Include any dependencies generated for this target.
include modules/module3/CMakeFiles/module3.dir/depend.make

# Include the progress variables for this target.
include modules/module3/CMakeFiles/module3.dir/progress.make

# Include the compile flags for this target's objects.
include modules/module3/CMakeFiles/module3.dir/flags.make

modules/module3/CMakeFiles/module3.dir/src/motionEstimation.cpp.o: modules/module3/CMakeFiles/module3.dir/flags.make
modules/module3/CMakeFiles/module3.dir/src/motionEstimation.cpp.o: ../modules/module3/src/motionEstimation.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/team-SLAM/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object modules/module3/CMakeFiles/module3.dir/src/motionEstimation.cpp.o"
	cd /team-SLAM/build/modules/module3 && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/module3.dir/src/motionEstimation.cpp.o -c /team-SLAM/modules/module3/src/motionEstimation.cpp

modules/module3/CMakeFiles/module3.dir/src/motionEstimation.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/module3.dir/src/motionEstimation.cpp.i"
	cd /team-SLAM/build/modules/module3 && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /team-SLAM/modules/module3/src/motionEstimation.cpp > CMakeFiles/module3.dir/src/motionEstimation.cpp.i

modules/module3/CMakeFiles/module3.dir/src/motionEstimation.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/module3.dir/src/motionEstimation.cpp.s"
	cd /team-SLAM/build/modules/module3 && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /team-SLAM/modules/module3/src/motionEstimation.cpp -o CMakeFiles/module3.dir/src/motionEstimation.cpp.s

modules/module3/CMakeFiles/module3.dir/src/similarity.cpp.o: modules/module3/CMakeFiles/module3.dir/flags.make
modules/module3/CMakeFiles/module3.dir/src/similarity.cpp.o: ../modules/module3/src/similarity.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/team-SLAM/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object modules/module3/CMakeFiles/module3.dir/src/similarity.cpp.o"
	cd /team-SLAM/build/modules/module3 && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/module3.dir/src/similarity.cpp.o -c /team-SLAM/modules/module3/src/similarity.cpp

modules/module3/CMakeFiles/module3.dir/src/similarity.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/module3.dir/src/similarity.cpp.i"
	cd /team-SLAM/build/modules/module3 && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /team-SLAM/modules/module3/src/similarity.cpp > CMakeFiles/module3.dir/src/similarity.cpp.i

modules/module3/CMakeFiles/module3.dir/src/similarity.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/module3.dir/src/similarity.cpp.s"
	cd /team-SLAM/build/modules/module3 && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /team-SLAM/modules/module3/src/similarity.cpp -o CMakeFiles/module3.dir/src/similarity.cpp.s

# Object files for target module3
module3_OBJECTS = \
"CMakeFiles/module3.dir/src/motionEstimation.cpp.o" \
"CMakeFiles/module3.dir/src/similarity.cpp.o"

# External object files for target module3
module3_EXTERNAL_OBJECTS =

modules/module3/libmodule3.so: modules/module3/CMakeFiles/module3.dir/src/motionEstimation.cpp.o
modules/module3/libmodule3.so: modules/module3/CMakeFiles/module3.dir/src/similarity.cpp.o
modules/module3/libmodule3.so: modules/module3/CMakeFiles/module3.dir/build.make
modules/module3/libmodule3.so: ../Thirdparty/opencv/install/lib/libopencv_dnn.so.4.4.0
modules/module3/libmodule3.so: ../Thirdparty/opencv/install/lib/libopencv_gapi.so.4.4.0
modules/module3/libmodule3.so: ../Thirdparty/opencv/install/lib/libopencv_highgui.so.4.4.0
modules/module3/libmodule3.so: ../Thirdparty/opencv/install/lib/libopencv_ml.so.4.4.0
modules/module3/libmodule3.so: ../Thirdparty/opencv/install/lib/libopencv_objdetect.so.4.4.0
modules/module3/libmodule3.so: ../Thirdparty/opencv/install/lib/libopencv_photo.so.4.4.0
modules/module3/libmodule3.so: ../Thirdparty/opencv/install/lib/libopencv_stitching.so.4.4.0
modules/module3/libmodule3.so: ../Thirdparty/opencv/install/lib/libopencv_video.so.4.4.0
modules/module3/libmodule3.so: ../Thirdparty/opencv/install/lib/libopencv_videoio.so.4.4.0
modules/module3/libmodule3.so: ../Thirdparty/opencv/install/lib/libopencv_imgcodecs.so.4.4.0
modules/module3/libmodule3.so: ../Thirdparty/opencv/install/lib/libopencv_calib3d.so.4.4.0
modules/module3/libmodule3.so: ../Thirdparty/opencv/install/lib/libopencv_features2d.so.4.4.0
modules/module3/libmodule3.so: ../Thirdparty/opencv/install/lib/libopencv_flann.so.4.4.0
modules/module3/libmodule3.so: ../Thirdparty/opencv/install/lib/libopencv_imgproc.so.4.4.0
modules/module3/libmodule3.so: ../Thirdparty/opencv/install/lib/libopencv_core.so.4.4.0
modules/module3/libmodule3.so: modules/module3/CMakeFiles/module3.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/team-SLAM/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX shared library libmodule3.so"
	cd /team-SLAM/build/modules/module3 && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/module3.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
modules/module3/CMakeFiles/module3.dir/build: modules/module3/libmodule3.so

.PHONY : modules/module3/CMakeFiles/module3.dir/build

modules/module3/CMakeFiles/module3.dir/clean:
	cd /team-SLAM/build/modules/module3 && $(CMAKE_COMMAND) -P CMakeFiles/module3.dir/cmake_clean.cmake
.PHONY : modules/module3/CMakeFiles/module3.dir/clean

modules/module3/CMakeFiles/module3.dir/depend:
	cd /team-SLAM/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /team-SLAM /team-SLAM/modules/module3 /team-SLAM/build /team-SLAM/build/modules/module3 /team-SLAM/build/modules/module3/CMakeFiles/module3.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : modules/module3/CMakeFiles/module3.dir/depend

