# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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
CMAKE_SOURCE_DIR = /var/www/pclBox

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /var/www/pclBox

# Include any dependencies generated for this target.
include CMakeFiles/pclBox.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/pclBox.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/pclBox.dir/flags.make

CMakeFiles/pclBox.dir/pclBox.cpp.o: CMakeFiles/pclBox.dir/flags.make
CMakeFiles/pclBox.dir/pclBox.cpp.o: pclBox.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /var/www/pclBox/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/pclBox.dir/pclBox.cpp.o"
	/usr/bin/g++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/pclBox.dir/pclBox.cpp.o -c /var/www/pclBox/pclBox.cpp

CMakeFiles/pclBox.dir/pclBox.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pclBox.dir/pclBox.cpp.i"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_FLAGS) -E /var/www/pclBox/pclBox.cpp > CMakeFiles/pclBox.dir/pclBox.cpp.i

CMakeFiles/pclBox.dir/pclBox.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pclBox.dir/pclBox.cpp.s"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_FLAGS) -S /var/www/pclBox/pclBox.cpp -o CMakeFiles/pclBox.dir/pclBox.cpp.s

CMakeFiles/pclBox.dir/pclBox.cpp.o.requires:
.PHONY : CMakeFiles/pclBox.dir/pclBox.cpp.o.requires

CMakeFiles/pclBox.dir/pclBox.cpp.o.provides: CMakeFiles/pclBox.dir/pclBox.cpp.o.requires
	$(MAKE) -f CMakeFiles/pclBox.dir/build.make CMakeFiles/pclBox.dir/pclBox.cpp.o.provides.build
.PHONY : CMakeFiles/pclBox.dir/pclBox.cpp.o.provides

CMakeFiles/pclBox.dir/pclBox.cpp.o.provides.build: CMakeFiles/pclBox.dir/pclBox.cpp.o

# Object files for target pclBox
pclBox_OBJECTS = \
"CMakeFiles/pclBox.dir/pclBox.cpp.o"

# External object files for target pclBox
pclBox_EXTERNAL_OBJECTS =

pclBox: CMakeFiles/pclBox.dir/pclBox.cpp.o
pclBox: CMakeFiles/pclBox.dir/build.make
pclBox: /usr/lib/libboost_system-mt.so
pclBox: /usr/lib/libboost_filesystem-mt.so
pclBox: /usr/lib/libboost_thread-mt.so
pclBox: /usr/lib/libboost_date_time-mt.so
pclBox: /usr/lib/libboost_iostreams-mt.so
pclBox: /usr/lib/libpcl_common.so
pclBox: /usr/lib/libpcl_octree.so
pclBox: /usr/lib/libOpenNI.so
pclBox: /usr/lib/libvtkCommon.so.5.8.0
pclBox: /usr/lib/libvtkRendering.so.5.8.0
pclBox: /usr/lib/libvtkHybrid.so.5.8.0
pclBox: /usr/lib/libpcl_io.so
pclBox: /usr/lib/libflann_cpp_s.a
pclBox: /usr/lib/libpcl_kdtree.so
pclBox: /usr/lib/libpcl_search.so
pclBox: /usr/lib/libpcl_sample_consensus.so
pclBox: /usr/lib/libpcl_filters.so
pclBox: /usr/lib/libpcl_segmentation.so
pclBox: /usr/lib/libpcl_features.so
pclBox: /usr/lib/libqhull.so
pclBox: /usr/lib/libpcl_surface.so
pclBox: /usr/lib/libpcl_registration.so
pclBox: /usr/lib/libpcl_visualization.so
pclBox: /usr/lib/libpcl_keypoints.so
pclBox: /usr/lib/libpcl_tracking.so
pclBox: /usr/lib/libpcl_apps.so
pclBox: /usr/lib/libvtkParallel.so.5.8.0
pclBox: /usr/lib/libvtkRendering.so.5.8.0
pclBox: /usr/lib/libvtkGraphics.so.5.8.0
pclBox: /usr/lib/libvtkImaging.so.5.8.0
pclBox: /usr/lib/libvtkIO.so.5.8.0
pclBox: /usr/lib/libvtkFiltering.so.5.8.0
pclBox: /usr/lib/libvtkCommon.so.5.8.0
pclBox: /usr/lib/libvtksys.so.5.8.0
pclBox: CMakeFiles/pclBox.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable pclBox"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/pclBox.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/pclBox.dir/build: pclBox
.PHONY : CMakeFiles/pclBox.dir/build

CMakeFiles/pclBox.dir/requires: CMakeFiles/pclBox.dir/pclBox.cpp.o.requires
.PHONY : CMakeFiles/pclBox.dir/requires

CMakeFiles/pclBox.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/pclBox.dir/cmake_clean.cmake
.PHONY : CMakeFiles/pclBox.dir/clean

CMakeFiles/pclBox.dir/depend:
	cd /var/www/pclBox && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /var/www/pclBox /var/www/pclBox /var/www/pclBox /var/www/pclBox /var/www/pclBox/CMakeFiles/pclBox.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/pclBox.dir/depend
