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
include CMakeFiles/untitled.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/untitled.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/untitled.dir/flags.make

CMakeFiles/untitled.dir/main.cpp.o: CMakeFiles/untitled.dir/flags.make
CMakeFiles/untitled.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/team-SLAM/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/untitled.dir/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/untitled.dir/main.cpp.o -c /team-SLAM/main.cpp

CMakeFiles/untitled.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/untitled.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /team-SLAM/main.cpp > CMakeFiles/untitled.dir/main.cpp.i

CMakeFiles/untitled.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/untitled.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /team-SLAM/main.cpp -o CMakeFiles/untitled.dir/main.cpp.s

# Object files for target untitled
untitled_OBJECTS = \
"CMakeFiles/untitled.dir/main.cpp.o"

# External object files for target untitled
untitled_EXTERNAL_OBJECTS =

untitled: CMakeFiles/untitled.dir/main.cpp.o
untitled: CMakeFiles/untitled.dir/build.make
untitled: ../Thirdparty/ceres/install/lib/libceres.a
untitled: ../Thirdparty/pcl/install/lib/libpcl_surface.so
untitled: ../Thirdparty/pcl/install/lib/libpcl_keypoints.so
untitled: ../Thirdparty/pcl/install/lib/libpcl_tracking.so
untitled: ../Thirdparty/pcl/install/lib/libpcl_recognition.so
untitled: ../Thirdparty/pcl/install/lib/libpcl_stereo.so
untitled: ../Thirdparty/pcl/install/lib/libpcl_outofcore.so
untitled: ../Thirdparty/pcl/install/lib/libpcl_people.so
untitled: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
untitled: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
untitled: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
untitled: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so.1.71.0
untitled: /usr/lib/x86_64-linux-gnu/libboost_serialization.so.1.71.0
untitled: /usr/lib/x86_64-linux-gnu/libfreetype.so
untitled: /usr/lib/x86_64-linux-gnu/libz.so
untitled: /usr/lib/x86_64-linux-gnu/libjpeg.so
untitled: /usr/lib/x86_64-linux-gnu/libpng.so
untitled: /usr/lib/x86_64-linux-gnu/libtiff.so
untitled: /usr/lib/x86_64-linux-gnu/libexpat.so
untitled: /usr/lib/x86_64-linux-gnu/libflann_cpp.so
untitled: /usr/lib/x86_64-linux-gnu/libqhull_r.so
untitled: ../Thirdparty/spdlog/install/lib/libspdlog.a
untitled: modules/module1/libmodule1.so
untitled: modules/module2/libmodule2.so
untitled: modules/module3/libmodule3.so
untitled: modules/module4/libmodule4.so
untitled: modules/module5/libmodule5.so
untitled: ../Thirdparty/pcl/install/lib/libpcl_registration.so
untitled: ../Thirdparty/pcl/install/lib/libpcl_segmentation.so
untitled: ../Thirdparty/pcl/install/lib/libpcl_features.so
untitled: ../Thirdparty/pcl/install/lib/libpcl_filters.so
untitled: ../Thirdparty/pcl/install/lib/libpcl_sample_consensus.so
untitled: ../Thirdparty/pcl/install/lib/libpcl_ml.so
untitled: ../Thirdparty/pcl/install/lib/libpcl_visualization.so
untitled: ../Thirdparty/pcl/install/lib/libpcl_search.so
untitled: ../Thirdparty/pcl/install/lib/libpcl_kdtree.so
untitled: ../Thirdparty/pcl/install/lib/libpcl_io.so
untitled: ../Thirdparty/pcl/install/lib/libpcl_octree.so
untitled: /usr/lib/x86_64-linux-gnu/libvtkChartsCore-7.1.so.7.1p.1
untitled: /usr/lib/x86_64-linux-gnu/libvtkInfovisCore-7.1.so.7.1p.1
untitled: /usr/lib/x86_64-linux-gnu/libvtkInteractionImage-7.1.so.7.1p.1
untitled: /usr/lib/x86_64-linux-gnu/libexpat.so
untitled: /usr/lib/x86_64-linux-gnu/libvtkIOGeometry-7.1.so.7.1p.1
untitled: /usr/lib/x86_64-linux-gnu/libvtkIOLegacy-7.1.so.7.1p.1
untitled: /usr/lib/x86_64-linux-gnu/libvtkIOPLY-7.1.so.7.1p.1
untitled: /usr/lib/x86_64-linux-gnu/libvtkRenderingLOD-7.1.so.7.1p.1
untitled: /usr/lib/x86_64-linux-gnu/libvtkViewsContext2D-7.1.so.7.1p.1
untitled: /usr/lib/x86_64-linux-gnu/libvtkViewsCore-7.1.so.7.1p.1
untitled: /usr/lib/x86_64-linux-gnu/libvtkInteractionWidgets-7.1.so.7.1p.1
untitled: /usr/lib/x86_64-linux-gnu/libvtkFiltersModeling-7.1.so.7.1p.1
untitled: /usr/lib/x86_64-linux-gnu/libvtkInteractionStyle-7.1.so.7.1p.1
untitled: /usr/lib/x86_64-linux-gnu/libvtkFiltersExtraction-7.1.so.7.1p.1
untitled: /usr/lib/x86_64-linux-gnu/libvtkFiltersStatistics-7.1.so.7.1p.1
untitled: /usr/lib/x86_64-linux-gnu/libvtkImagingFourier-7.1.so.7.1p.1
untitled: /usr/lib/x86_64-linux-gnu/libvtkalglib-7.1.so.7.1p.1
untitled: /usr/lib/x86_64-linux-gnu/libvtkFiltersHybrid-7.1.so.7.1p.1
untitled: /usr/lib/x86_64-linux-gnu/libvtkImagingGeneral-7.1.so.7.1p.1
untitled: /usr/lib/x86_64-linux-gnu/libvtkImagingSources-7.1.so.7.1p.1
untitled: /usr/lib/x86_64-linux-gnu/libvtkImagingHybrid-7.1.so.7.1p.1
untitled: /usr/lib/x86_64-linux-gnu/libvtkRenderingAnnotation-7.1.so.7.1p.1
untitled: /usr/lib/x86_64-linux-gnu/libvtkImagingColor-7.1.so.7.1p.1
untitled: /usr/lib/x86_64-linux-gnu/libvtkRenderingVolume-7.1.so.7.1p.1
untitled: /usr/lib/x86_64-linux-gnu/libvtkIOXML-7.1.so.7.1p.1
untitled: /usr/lib/x86_64-linux-gnu/libvtkIOXMLParser-7.1.so.7.1p.1
untitled: /usr/lib/x86_64-linux-gnu/libvtkIOCore-7.1.so.7.1p.1
untitled: /usr/lib/x86_64-linux-gnu/libvtkRenderingContextOpenGL2-7.1.so.7.1p.1
untitled: /usr/lib/x86_64-linux-gnu/libvtkRenderingContext2D-7.1.so.7.1p.1
untitled: /usr/lib/x86_64-linux-gnu/libvtkRenderingFreeType-7.1.so.7.1p.1
untitled: /usr/lib/x86_64-linux-gnu/libfreetype.so
untitled: /usr/lib/x86_64-linux-gnu/libvtkRenderingOpenGL2-7.1.so.7.1p.1
untitled: /usr/lib/x86_64-linux-gnu/libvtkImagingCore-7.1.so.7.1p.1
untitled: /usr/lib/x86_64-linux-gnu/libvtkRenderingCore-7.1.so.7.1p.1
untitled: /usr/lib/x86_64-linux-gnu/libvtkCommonColor-7.1.so.7.1p.1
untitled: /usr/lib/x86_64-linux-gnu/libvtkFiltersGeometry-7.1.so.7.1p.1
untitled: /usr/lib/x86_64-linux-gnu/libvtkFiltersSources-7.1.so.7.1p.1
untitled: /usr/lib/x86_64-linux-gnu/libvtkFiltersGeneral-7.1.so.7.1p.1
untitled: /usr/lib/x86_64-linux-gnu/libvtkCommonComputationalGeometry-7.1.so.7.1p.1
untitled: /usr/lib/x86_64-linux-gnu/libvtkFiltersCore-7.1.so.7.1p.1
untitled: /usr/lib/x86_64-linux-gnu/libvtkIOImage-7.1.so.7.1p.1
untitled: /usr/lib/x86_64-linux-gnu/libvtkCommonExecutionModel-7.1.so.7.1p.1
untitled: /usr/lib/x86_64-linux-gnu/libvtkCommonDataModel-7.1.so.7.1p.1
untitled: /usr/lib/x86_64-linux-gnu/libvtkCommonTransforms-7.1.so.7.1p.1
untitled: /usr/lib/x86_64-linux-gnu/libvtkCommonMisc-7.1.so.7.1p.1
untitled: /usr/lib/x86_64-linux-gnu/libvtkCommonMath-7.1.so.7.1p.1
untitled: /usr/lib/x86_64-linux-gnu/libvtkCommonSystem-7.1.so.7.1p.1
untitled: /usr/lib/x86_64-linux-gnu/libvtkCommonCore-7.1.so.7.1p.1
untitled: /usr/lib/x86_64-linux-gnu/libvtksys-7.1.so.7.1p.1
untitled: /usr/lib/x86_64-linux-gnu/libvtkDICOMParser-7.1.so.7.1p.1
untitled: /usr/lib/x86_64-linux-gnu/libvtkmetaio-7.1.so.7.1p.1
untitled: /usr/lib/x86_64-linux-gnu/libXt.so
untitled: ../Thirdparty/pcl/install/lib/libpcl_common.so
untitled: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
untitled: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
untitled: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
untitled: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so.1.71.0
untitled: /usr/lib/x86_64-linux-gnu/libboost_serialization.so.1.71.0
untitled: ../Thirdparty/ceres/install/lib/libceres.a
untitled: /usr/lib/x86_64-linux-gnu/libglog.so
untitled: /usr/lib/x86_64-linux-gnu/libgflags.so.2.2.2
untitled: /usr/lib/x86_64-linux-gnu/libspqr.so
untitled: /usr/lib/x86_64-linux-gnu/libcholmod.so
untitled: /usr/lib/x86_64-linux-gnu/libamd.so
untitled: /usr/lib/x86_64-linux-gnu/libcamd.so
untitled: /usr/lib/x86_64-linux-gnu/libccolamd.so
untitled: /usr/lib/x86_64-linux-gnu/libcolamd.so
untitled: /usr/lib/x86_64-linux-gnu/libsuitesparseconfig.so
untitled: /usr/lib/x86_64-linux-gnu/liblapack.so
untitled: /usr/lib/x86_64-linux-gnu/libf77blas.so
untitled: /usr/lib/x86_64-linux-gnu/libatlas.so
untitled: ../Thirdparty/opencv/install/lib/libopencv_dnn.so.4.4.0
untitled: ../Thirdparty/opencv/install/lib/libopencv_gapi.so.4.4.0
untitled: ../Thirdparty/opencv/install/lib/libopencv_highgui.so.4.4.0
untitled: ../Thirdparty/opencv/install/lib/libopencv_ml.so.4.4.0
untitled: ../Thirdparty/opencv/install/lib/libopencv_objdetect.so.4.4.0
untitled: ../Thirdparty/opencv/install/lib/libopencv_photo.so.4.4.0
untitled: ../Thirdparty/opencv/install/lib/libopencv_stitching.so.4.4.0
untitled: ../Thirdparty/opencv/install/lib/libopencv_video.so.4.4.0
untitled: ../Thirdparty/opencv/install/lib/libopencv_calib3d.so.4.4.0
untitled: ../Thirdparty/opencv/install/lib/libopencv_features2d.so.4.4.0
untitled: ../Thirdparty/opencv/install/lib/libopencv_flann.so.4.4.0
untitled: ../Thirdparty/opencv/install/lib/libopencv_videoio.so.4.4.0
untitled: ../Thirdparty/opencv/install/lib/libopencv_imgcodecs.so.4.4.0
untitled: ../Thirdparty/opencv/install/lib/libopencv_imgproc.so.4.4.0
untitled: ../Thirdparty/opencv/install/lib/libopencv_core.so.4.4.0
untitled: ../Thirdparty/pangolin/install/lib/libpangolin.so
untitled: /usr/lib/x86_64-linux-gnu/libOpenGL.so
untitled: /usr/lib/x86_64-linux-gnu/libGLX.so
untitled: /usr/lib/x86_64-linux-gnu/libGLU.so
untitled: /usr/lib/x86_64-linux-gnu/libEGL.so
untitled: /usr/lib/x86_64-linux-gnu/libOpenGL.so
untitled: /usr/lib/x86_64-linux-gnu/libGLX.so
untitled: /usr/lib/x86_64-linux-gnu/libGLU.so
untitled: /usr/lib/x86_64-linux-gnu/libEGL.so
untitled: /usr/lib/x86_64-linux-gnu/libGLEW.so
untitled: /usr/lib/x86_64-linux-gnu/libSM.so
untitled: /usr/lib/x86_64-linux-gnu/libICE.so
untitled: /usr/lib/x86_64-linux-gnu/libX11.so
untitled: /usr/lib/x86_64-linux-gnu/libXext.so
untitled: /usr/lib/x86_64-linux-gnu/libavcodec.so
untitled: /usr/lib/x86_64-linux-gnu/libavformat.so
untitled: /usr/lib/x86_64-linux-gnu/libavutil.so
untitled: /usr/lib/x86_64-linux-gnu/libswscale.so
untitled: /usr/lib/x86_64-linux-gnu/libavdevice.so
untitled: /usr/lib/x86_64-linux-gnu/libz.so
untitled: /usr/lib/x86_64-linux-gnu/libjpeg.so
untitled: /usr/lib/x86_64-linux-gnu/libpng.so
untitled: /usr/lib/x86_64-linux-gnu/libtiff.so
untitled: /usr/lib/x86_64-linux-gnu/libzstd.so
untitled: /usr/lib/x86_64-linux-gnu/liblz4.so
untitled: ../Thirdparty/spdlog/install/lib/libspdlog.a
untitled: CMakeFiles/untitled.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/team-SLAM/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable untitled"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/untitled.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/untitled.dir/build: untitled

.PHONY : CMakeFiles/untitled.dir/build

CMakeFiles/untitled.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/untitled.dir/cmake_clean.cmake
.PHONY : CMakeFiles/untitled.dir/clean

CMakeFiles/untitled.dir/depend:
	cd /team-SLAM/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /team-SLAM /team-SLAM /team-SLAM/build /team-SLAM/build /team-SLAM/build/CMakeFiles/untitled.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/untitled.dir/depend

