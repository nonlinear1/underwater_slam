# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.8

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
CMAKE_COMMAND = /home/daniel/.myapp/clion-2017.1.3/bin/cmake/bin/cmake

# The command to remove a file.
RM = /home/daniel/.myapp/clion-2017.1.3/bin/cmake/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/daniel/CLionProjects/pcl_icp

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/daniel/CLionProjects/pcl_icp/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/mytest.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/mytest.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/mytest.dir/flags.make

CMakeFiles/mytest.dir/test.cpp.o: CMakeFiles/mytest.dir/flags.make
CMakeFiles/mytest.dir/test.cpp.o: ../test.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/daniel/CLionProjects/pcl_icp/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/mytest.dir/test.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/mytest.dir/test.cpp.o -c /home/daniel/CLionProjects/pcl_icp/test.cpp

CMakeFiles/mytest.dir/test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mytest.dir/test.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/daniel/CLionProjects/pcl_icp/test.cpp > CMakeFiles/mytest.dir/test.cpp.i

CMakeFiles/mytest.dir/test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mytest.dir/test.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/daniel/CLionProjects/pcl_icp/test.cpp -o CMakeFiles/mytest.dir/test.cpp.s

CMakeFiles/mytest.dir/test.cpp.o.requires:

.PHONY : CMakeFiles/mytest.dir/test.cpp.o.requires

CMakeFiles/mytest.dir/test.cpp.o.provides: CMakeFiles/mytest.dir/test.cpp.o.requires
	$(MAKE) -f CMakeFiles/mytest.dir/build.make CMakeFiles/mytest.dir/test.cpp.o.provides.build
.PHONY : CMakeFiles/mytest.dir/test.cpp.o.provides

CMakeFiles/mytest.dir/test.cpp.o.provides.build: CMakeFiles/mytest.dir/test.cpp.o


# Object files for target mytest
mytest_OBJECTS = \
"CMakeFiles/mytest.dir/test.cpp.o"

# External object files for target mytest
mytest_EXTERNAL_OBJECTS =

mytest: CMakeFiles/mytest.dir/test.cpp.o
mytest: CMakeFiles/mytest.dir/build.make
mytest: /usr/lib/x86_64-linux-gnu/libboost_system.so
mytest: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
mytest: /usr/lib/x86_64-linux-gnu/libboost_thread.so
mytest: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
mytest: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
mytest: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
mytest: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
mytest: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
mytest: /usr/lib/x86_64-linux-gnu/libboost_regex.so
mytest: /usr/local/lib/libpcl_common.so
mytest: /usr/local/lib/libpcl_octree.so
mytest: /usr/lib/libOpenNI.so
mytest: /usr/lib/libOpenNI2.so
mytest: /usr/local/lib/libpcl_io.so
mytest: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
mytest: /usr/local/lib/libpcl_kdtree.so
mytest: /usr/local/lib/libpcl_search.so
mytest: /usr/lib/x86_64-linux-gnu/libqhull.so
mytest: /usr/local/lib/libpcl_surface.so
mytest: /usr/local/lib/libpcl_sample_consensus.so
mytest: /usr/local/lib/libpcl_filters.so
mytest: /usr/local/lib/libpcl_features.so
mytest: /usr/local/lib/libpcl_visualization.so
mytest: /usr/local/lib/libpcl_ml.so
mytest: /usr/local/lib/libpcl_segmentation.so
mytest: /usr/local/lib/libpcl_registration.so
mytest: /usr/local/lib/libpcl_recognition.so
mytest: /usr/local/lib/libpcl_keypoints.so
mytest: /usr/local/lib/libpcl_people.so
mytest: /usr/local/lib/libpcl_outofcore.so
mytest: /usr/local/lib/libpcl_stereo.so
mytest: /usr/local/lib/libpcl_tracking.so
mytest: /usr/lib/x86_64-linux-gnu/libboost_system.so
mytest: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
mytest: /usr/lib/x86_64-linux-gnu/libboost_thread.so
mytest: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
mytest: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
mytest: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
mytest: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
mytest: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
mytest: /usr/lib/x86_64-linux-gnu/libboost_regex.so
mytest: /usr/lib/x86_64-linux-gnu/libqhull.so
mytest: /usr/lib/libOpenNI.so
mytest: /usr/lib/libOpenNI2.so
mytest: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
mytest: /usr/lib/libvtkGenericFiltering.so.5.8.0
mytest: /usr/lib/libvtkGeovis.so.5.8.0
mytest: /usr/lib/libvtkCharts.so.5.8.0
mytest: /usr/local/lib/libpcl_common.so
mytest: /usr/local/lib/libpcl_octree.so
mytest: /usr/local/lib/libpcl_io.so
mytest: /usr/local/lib/libpcl_kdtree.so
mytest: /usr/local/lib/libpcl_search.so
mytest: /usr/local/lib/libpcl_surface.so
mytest: /usr/local/lib/libpcl_sample_consensus.so
mytest: /usr/local/lib/libpcl_filters.so
mytest: /usr/local/lib/libpcl_features.so
mytest: /usr/local/lib/libpcl_visualization.so
mytest: /usr/local/lib/libpcl_ml.so
mytest: /usr/local/lib/libpcl_segmentation.so
mytest: /usr/local/lib/libpcl_registration.so
mytest: /usr/local/lib/libpcl_recognition.so
mytest: /usr/local/lib/libpcl_keypoints.so
mytest: /usr/local/lib/libpcl_people.so
mytest: /usr/local/lib/libpcl_outofcore.so
mytest: /usr/local/lib/libpcl_stereo.so
mytest: /usr/local/lib/libpcl_tracking.so
mytest: /usr/lib/libvtkViews.so.5.8.0
mytest: /usr/lib/libvtkInfovis.so.5.8.0
mytest: /usr/lib/libvtkWidgets.so.5.8.0
mytest: /usr/lib/libvtkVolumeRendering.so.5.8.0
mytest: /usr/lib/libvtkHybrid.so.5.8.0
mytest: /usr/lib/libvtkParallel.so.5.8.0
mytest: /usr/lib/libvtkRendering.so.5.8.0
mytest: /usr/lib/libvtkImaging.so.5.8.0
mytest: /usr/lib/libvtkGraphics.so.5.8.0
mytest: /usr/lib/libvtkIO.so.5.8.0
mytest: /usr/lib/libvtkFiltering.so.5.8.0
mytest: /usr/lib/libvtkCommon.so.5.8.0
mytest: /usr/lib/libvtksys.so.5.8.0
mytest: CMakeFiles/mytest.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/daniel/CLionProjects/pcl_icp/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable mytest"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/mytest.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/mytest.dir/build: mytest

.PHONY : CMakeFiles/mytest.dir/build

CMakeFiles/mytest.dir/requires: CMakeFiles/mytest.dir/test.cpp.o.requires

.PHONY : CMakeFiles/mytest.dir/requires

CMakeFiles/mytest.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/mytest.dir/cmake_clean.cmake
.PHONY : CMakeFiles/mytest.dir/clean

CMakeFiles/mytest.dir/depend:
	cd /home/daniel/CLionProjects/pcl_icp/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/daniel/CLionProjects/pcl_icp /home/daniel/CLionProjects/pcl_icp /home/daniel/CLionProjects/pcl_icp/cmake-build-debug /home/daniel/CLionProjects/pcl_icp/cmake-build-debug /home/daniel/CLionProjects/pcl_icp/cmake-build-debug/CMakeFiles/mytest.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/mytest.dir/depend
