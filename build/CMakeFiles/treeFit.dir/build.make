# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = /home/teeramoo/Desktop/Thesis_work/Point_cloud_segmentation/tree_fit

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/teeramoo/Desktop/Thesis_work/Point_cloud_segmentation/tree_fit/build

# Include any dependencies generated for this target.
include CMakeFiles/treeFit.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/treeFit.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/treeFit.dir/flags.make

CMakeFiles/treeFit.dir/src/PlaneProcessor.cpp.o: CMakeFiles/treeFit.dir/flags.make
CMakeFiles/treeFit.dir/src/PlaneProcessor.cpp.o: ../src/PlaneProcessor.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/teeramoo/Desktop/Thesis_work/Point_cloud_segmentation/tree_fit/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/treeFit.dir/src/PlaneProcessor.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/treeFit.dir/src/PlaneProcessor.cpp.o -c /home/teeramoo/Desktop/Thesis_work/Point_cloud_segmentation/tree_fit/src/PlaneProcessor.cpp

CMakeFiles/treeFit.dir/src/PlaneProcessor.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/treeFit.dir/src/PlaneProcessor.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/teeramoo/Desktop/Thesis_work/Point_cloud_segmentation/tree_fit/src/PlaneProcessor.cpp > CMakeFiles/treeFit.dir/src/PlaneProcessor.cpp.i

CMakeFiles/treeFit.dir/src/PlaneProcessor.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/treeFit.dir/src/PlaneProcessor.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/teeramoo/Desktop/Thesis_work/Point_cloud_segmentation/tree_fit/src/PlaneProcessor.cpp -o CMakeFiles/treeFit.dir/src/PlaneProcessor.cpp.s

CMakeFiles/treeFit.dir/src/PlaneProcessor.cpp.o.requires:

.PHONY : CMakeFiles/treeFit.dir/src/PlaneProcessor.cpp.o.requires

CMakeFiles/treeFit.dir/src/PlaneProcessor.cpp.o.provides: CMakeFiles/treeFit.dir/src/PlaneProcessor.cpp.o.requires
	$(MAKE) -f CMakeFiles/treeFit.dir/build.make CMakeFiles/treeFit.dir/src/PlaneProcessor.cpp.o.provides.build
.PHONY : CMakeFiles/treeFit.dir/src/PlaneProcessor.cpp.o.provides

CMakeFiles/treeFit.dir/src/PlaneProcessor.cpp.o.provides.build: CMakeFiles/treeFit.dir/src/PlaneProcessor.cpp.o


CMakeFiles/treeFit.dir/src/Viewer.cpp.o: CMakeFiles/treeFit.dir/flags.make
CMakeFiles/treeFit.dir/src/Viewer.cpp.o: ../src/Viewer.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/teeramoo/Desktop/Thesis_work/Point_cloud_segmentation/tree_fit/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/treeFit.dir/src/Viewer.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/treeFit.dir/src/Viewer.cpp.o -c /home/teeramoo/Desktop/Thesis_work/Point_cloud_segmentation/tree_fit/src/Viewer.cpp

CMakeFiles/treeFit.dir/src/Viewer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/treeFit.dir/src/Viewer.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/teeramoo/Desktop/Thesis_work/Point_cloud_segmentation/tree_fit/src/Viewer.cpp > CMakeFiles/treeFit.dir/src/Viewer.cpp.i

CMakeFiles/treeFit.dir/src/Viewer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/treeFit.dir/src/Viewer.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/teeramoo/Desktop/Thesis_work/Point_cloud_segmentation/tree_fit/src/Viewer.cpp -o CMakeFiles/treeFit.dir/src/Viewer.cpp.s

CMakeFiles/treeFit.dir/src/Viewer.cpp.o.requires:

.PHONY : CMakeFiles/treeFit.dir/src/Viewer.cpp.o.requires

CMakeFiles/treeFit.dir/src/Viewer.cpp.o.provides: CMakeFiles/treeFit.dir/src/Viewer.cpp.o.requires
	$(MAKE) -f CMakeFiles/treeFit.dir/build.make CMakeFiles/treeFit.dir/src/Viewer.cpp.o.provides.build
.PHONY : CMakeFiles/treeFit.dir/src/Viewer.cpp.o.provides

CMakeFiles/treeFit.dir/src/Viewer.cpp.o.provides.build: CMakeFiles/treeFit.dir/src/Viewer.cpp.o


CMakeFiles/treeFit.dir/src/NormalEstimator.cpp.o: CMakeFiles/treeFit.dir/flags.make
CMakeFiles/treeFit.dir/src/NormalEstimator.cpp.o: ../src/NormalEstimator.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/teeramoo/Desktop/Thesis_work/Point_cloud_segmentation/tree_fit/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/treeFit.dir/src/NormalEstimator.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/treeFit.dir/src/NormalEstimator.cpp.o -c /home/teeramoo/Desktop/Thesis_work/Point_cloud_segmentation/tree_fit/src/NormalEstimator.cpp

CMakeFiles/treeFit.dir/src/NormalEstimator.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/treeFit.dir/src/NormalEstimator.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/teeramoo/Desktop/Thesis_work/Point_cloud_segmentation/tree_fit/src/NormalEstimator.cpp > CMakeFiles/treeFit.dir/src/NormalEstimator.cpp.i

CMakeFiles/treeFit.dir/src/NormalEstimator.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/treeFit.dir/src/NormalEstimator.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/teeramoo/Desktop/Thesis_work/Point_cloud_segmentation/tree_fit/src/NormalEstimator.cpp -o CMakeFiles/treeFit.dir/src/NormalEstimator.cpp.s

CMakeFiles/treeFit.dir/src/NormalEstimator.cpp.o.requires:

.PHONY : CMakeFiles/treeFit.dir/src/NormalEstimator.cpp.o.requires

CMakeFiles/treeFit.dir/src/NormalEstimator.cpp.o.provides: CMakeFiles/treeFit.dir/src/NormalEstimator.cpp.o.requires
	$(MAKE) -f CMakeFiles/treeFit.dir/build.make CMakeFiles/treeFit.dir/src/NormalEstimator.cpp.o.provides.build
.PHONY : CMakeFiles/treeFit.dir/src/NormalEstimator.cpp.o.provides

CMakeFiles/treeFit.dir/src/NormalEstimator.cpp.o.provides.build: CMakeFiles/treeFit.dir/src/NormalEstimator.cpp.o


CMakeFiles/treeFit.dir/src/CylinderProcessor.cpp.o: CMakeFiles/treeFit.dir/flags.make
CMakeFiles/treeFit.dir/src/CylinderProcessor.cpp.o: ../src/CylinderProcessor.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/teeramoo/Desktop/Thesis_work/Point_cloud_segmentation/tree_fit/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/treeFit.dir/src/CylinderProcessor.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/treeFit.dir/src/CylinderProcessor.cpp.o -c /home/teeramoo/Desktop/Thesis_work/Point_cloud_segmentation/tree_fit/src/CylinderProcessor.cpp

CMakeFiles/treeFit.dir/src/CylinderProcessor.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/treeFit.dir/src/CylinderProcessor.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/teeramoo/Desktop/Thesis_work/Point_cloud_segmentation/tree_fit/src/CylinderProcessor.cpp > CMakeFiles/treeFit.dir/src/CylinderProcessor.cpp.i

CMakeFiles/treeFit.dir/src/CylinderProcessor.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/treeFit.dir/src/CylinderProcessor.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/teeramoo/Desktop/Thesis_work/Point_cloud_segmentation/tree_fit/src/CylinderProcessor.cpp -o CMakeFiles/treeFit.dir/src/CylinderProcessor.cpp.s

CMakeFiles/treeFit.dir/src/CylinderProcessor.cpp.o.requires:

.PHONY : CMakeFiles/treeFit.dir/src/CylinderProcessor.cpp.o.requires

CMakeFiles/treeFit.dir/src/CylinderProcessor.cpp.o.provides: CMakeFiles/treeFit.dir/src/CylinderProcessor.cpp.o.requires
	$(MAKE) -f CMakeFiles/treeFit.dir/build.make CMakeFiles/treeFit.dir/src/CylinderProcessor.cpp.o.provides.build
.PHONY : CMakeFiles/treeFit.dir/src/CylinderProcessor.cpp.o.provides

CMakeFiles/treeFit.dir/src/CylinderProcessor.cpp.o.provides.build: CMakeFiles/treeFit.dir/src/CylinderProcessor.cpp.o


# Object files for target treeFit
treeFit_OBJECTS = \
"CMakeFiles/treeFit.dir/src/PlaneProcessor.cpp.o" \
"CMakeFiles/treeFit.dir/src/Viewer.cpp.o" \
"CMakeFiles/treeFit.dir/src/NormalEstimator.cpp.o" \
"CMakeFiles/treeFit.dir/src/CylinderProcessor.cpp.o"

# External object files for target treeFit
treeFit_EXTERNAL_OBJECTS =

libtreeFit.so: CMakeFiles/treeFit.dir/src/PlaneProcessor.cpp.o
libtreeFit.so: CMakeFiles/treeFit.dir/src/Viewer.cpp.o
libtreeFit.so: CMakeFiles/treeFit.dir/src/NormalEstimator.cpp.o
libtreeFit.so: CMakeFiles/treeFit.dir/src/CylinderProcessor.cpp.o
libtreeFit.so: CMakeFiles/treeFit.dir/build.make
libtreeFit.so: CMakeFiles/treeFit.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/teeramoo/Desktop/Thesis_work/Point_cloud_segmentation/tree_fit/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Linking CXX shared library libtreeFit.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/treeFit.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/treeFit.dir/build: libtreeFit.so

.PHONY : CMakeFiles/treeFit.dir/build

CMakeFiles/treeFit.dir/requires: CMakeFiles/treeFit.dir/src/PlaneProcessor.cpp.o.requires
CMakeFiles/treeFit.dir/requires: CMakeFiles/treeFit.dir/src/Viewer.cpp.o.requires
CMakeFiles/treeFit.dir/requires: CMakeFiles/treeFit.dir/src/NormalEstimator.cpp.o.requires
CMakeFiles/treeFit.dir/requires: CMakeFiles/treeFit.dir/src/CylinderProcessor.cpp.o.requires

.PHONY : CMakeFiles/treeFit.dir/requires

CMakeFiles/treeFit.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/treeFit.dir/cmake_clean.cmake
.PHONY : CMakeFiles/treeFit.dir/clean

CMakeFiles/treeFit.dir/depend:
	cd /home/teeramoo/Desktop/Thesis_work/Point_cloud_segmentation/tree_fit/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/teeramoo/Desktop/Thesis_work/Point_cloud_segmentation/tree_fit /home/teeramoo/Desktop/Thesis_work/Point_cloud_segmentation/tree_fit /home/teeramoo/Desktop/Thesis_work/Point_cloud_segmentation/tree_fit/build /home/teeramoo/Desktop/Thesis_work/Point_cloud_segmentation/tree_fit/build /home/teeramoo/Desktop/Thesis_work/Point_cloud_segmentation/tree_fit/build/CMakeFiles/treeFit.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/treeFit.dir/depend

