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
include modules/module5/CMakeFiles/module5.dir/depend.make

# Include the progress variables for this target.
include modules/module5/CMakeFiles/module5.dir/progress.make

# Include the compile flags for this target's objects.
include modules/module5/CMakeFiles/module5.dir/flags.make

modules/module5/CMakeFiles/module5.dir/src/draw.cpp.o: modules/module5/CMakeFiles/module5.dir/flags.make
modules/module5/CMakeFiles/module5.dir/src/draw.cpp.o: ../modules/module5/src/draw.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/team-SLAM/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object modules/module5/CMakeFiles/module5.dir/src/draw.cpp.o"
	cd /team-SLAM/build/modules/module5 && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/module5.dir/src/draw.cpp.o -c /team-SLAM/modules/module5/src/draw.cpp

modules/module5/CMakeFiles/module5.dir/src/draw.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/module5.dir/src/draw.cpp.i"
	cd /team-SLAM/build/modules/module5 && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /team-SLAM/modules/module5/src/draw.cpp > CMakeFiles/module5.dir/src/draw.cpp.i

modules/module5/CMakeFiles/module5.dir/src/draw.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/module5.dir/src/draw.cpp.s"
	cd /team-SLAM/build/modules/module5 && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /team-SLAM/modules/module5/src/draw.cpp -o CMakeFiles/module5.dir/src/draw.cpp.s

# Object files for target module5
module5_OBJECTS = \
"CMakeFiles/module5.dir/src/draw.cpp.o"

# External object files for target module5
module5_EXTERNAL_OBJECTS =

modules/module5/libmodule5.so: modules/module5/CMakeFiles/module5.dir/src/draw.cpp.o
modules/module5/libmodule5.so: modules/module5/CMakeFiles/module5.dir/build.make
modules/module5/libmodule5.so: ../Thirdparty/opencv/install/lib/libopencv_dnn.so.4.4.0
modules/module5/libmodule5.so: ../Thirdparty/opencv/install/lib/libopencv_gapi.so.4.4.0
modules/module5/libmodule5.so: ../Thirdparty/opencv/install/lib/libopencv_highgui.so.4.4.0
modules/module5/libmodule5.so: ../Thirdparty/opencv/install/lib/libopencv_ml.so.4.4.0
modules/module5/libmodule5.so: ../Thirdparty/opencv/install/lib/libopencv_objdetect.so.4.4.0
modules/module5/libmodule5.so: ../Thirdparty/opencv/install/lib/libopencv_photo.so.4.4.0
modules/module5/libmodule5.so: ../Thirdparty/opencv/install/lib/libopencv_stitching.so.4.4.0
modules/module5/libmodule5.so: ../Thirdparty/opencv/install/lib/libopencv_video.so.4.4.0
modules/module5/libmodule5.so: ../Thirdparty/opencv/install/lib/libopencv_videoio.so.4.4.0
modules/module5/libmodule5.so: ../Thirdparty/pangolin/install/lib/libpangolin.so
modules/module5/libmodule5.so: ../Thirdparty/spdlog/install/lib/libspdlog.a
modules/module5/libmodule5.so: ../Thirdparty/opencv/install/lib/libopencv_imgcodecs.so.4.4.0
modules/module5/libmodule5.so: ../Thirdparty/opencv/install/lib/libopencv_calib3d.so.4.4.0
modules/module5/libmodule5.so: ../Thirdparty/opencv/install/lib/libopencv_features2d.so.4.4.0
modules/module5/libmodule5.so: ../Thirdparty/opencv/install/lib/libopencv_flann.so.4.4.0
modules/module5/libmodule5.so: ../Thirdparty/opencv/install/lib/libopencv_imgproc.so.4.4.0
modules/module5/libmodule5.so: ../Thirdparty/opencv/install/lib/libopencv_core.so.4.4.0
modules/module5/libmodule5.so: /usr/lib/x86_64-linux-gnu/libOpenGL.so
modules/module5/libmodule5.so: /usr/lib/x86_64-linux-gnu/libGLX.so
modules/module5/libmodule5.so: /usr/lib/x86_64-linux-gnu/libGLU.so
modules/module5/libmodule5.so: /usr/lib/x86_64-linux-gnu/libGLEW.so
modules/module5/libmodule5.so: /usr/lib/x86_64-linux-gnu/libEGL.so
modules/module5/libmodule5.so: /usr/lib/x86_64-linux-gnu/libSM.so
modules/module5/libmodule5.so: /usr/lib/x86_64-linux-gnu/libICE.so
modules/module5/libmodule5.so: /usr/lib/x86_64-linux-gnu/libX11.so
modules/module5/libmodule5.so: /usr/lib/x86_64-linux-gnu/libXext.so
modules/module5/libmodule5.so: /usr/lib/x86_64-linux-gnu/libOpenGL.so
modules/module5/libmodule5.so: /usr/lib/x86_64-linux-gnu/libGLX.so
modules/module5/libmodule5.so: /usr/lib/x86_64-linux-gnu/libGLU.so
modules/module5/libmodule5.so: /usr/lib/x86_64-linux-gnu/libGLEW.so
modules/module5/libmodule5.so: /usr/lib/x86_64-linux-gnu/libEGL.so
modules/module5/libmodule5.so: /usr/lib/x86_64-linux-gnu/libSM.so
modules/module5/libmodule5.so: /usr/lib/x86_64-linux-gnu/libICE.so
modules/module5/libmodule5.so: /usr/lib/x86_64-linux-gnu/libX11.so
modules/module5/libmodule5.so: /usr/lib/x86_64-linux-gnu/libXext.so
modules/module5/libmodule5.so: /usr/lib/x86_64-linux-gnu/libavcodec.so
modules/module5/libmodule5.so: /usr/lib/x86_64-linux-gnu/libavformat.so
modules/module5/libmodule5.so: /usr/lib/x86_64-linux-gnu/libavutil.so
modules/module5/libmodule5.so: /usr/lib/x86_64-linux-gnu/libswscale.so
modules/module5/libmodule5.so: /usr/lib/x86_64-linux-gnu/libavdevice.so
modules/module5/libmodule5.so: /usr/lib/x86_64-linux-gnu/libpng.so
modules/module5/libmodule5.so: /usr/lib/x86_64-linux-gnu/libz.so
modules/module5/libmodule5.so: /usr/lib/x86_64-linux-gnu/libjpeg.so
modules/module5/libmodule5.so: /usr/lib/x86_64-linux-gnu/libtiff.so
modules/module5/libmodule5.so: /usr/lib/x86_64-linux-gnu/libzstd.so
modules/module5/libmodule5.so: /usr/lib/x86_64-linux-gnu/liblz4.so
modules/module5/libmodule5.so: modules/module5/CMakeFiles/module5.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/team-SLAM/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library libmodule5.so"
	cd /team-SLAM/build/modules/module5 && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/module5.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
modules/module5/CMakeFiles/module5.dir/build: modules/module5/libmodule5.so

.PHONY : modules/module5/CMakeFiles/module5.dir/build

modules/module5/CMakeFiles/module5.dir/clean:
	cd /team-SLAM/build/modules/module5 && $(CMAKE_COMMAND) -P CMakeFiles/module5.dir/cmake_clean.cmake
.PHONY : modules/module5/CMakeFiles/module5.dir/clean

modules/module5/CMakeFiles/module5.dir/depend:
	cd /team-SLAM/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /team-SLAM /team-SLAM/modules/module5 /team-SLAM/build /team-SLAM/build/modules/module5 /team-SLAM/build/modules/module5/CMakeFiles/module5.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : modules/module5/CMakeFiles/module5.dir/depend

