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
CMAKE_SOURCE_DIR = /home/hzx/catkin_ws/src/VINS-Fusion/dense

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/hzx/catkin_ws/src/VINS-Fusion/dense/cmake-build-debug

# Include any dependencies generated for this target.
include main/CMakeFiles/cmvs.dir/depend.make

# Include the progress variables for this target.
include main/CMakeFiles/cmvs.dir/progress.make

# Include the compile flags for this target's objects.
include main/CMakeFiles/cmvs.dir/flags.make

main/CMakeFiles/cmvs.dir/cmvs.cc.o: main/CMakeFiles/cmvs.dir/flags.make
main/CMakeFiles/cmvs.dir/cmvs.cc.o: ../main/cmvs.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/hzx/catkin_ws/src/VINS-Fusion/dense/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object main/CMakeFiles/cmvs.dir/cmvs.cc.o"
	cd /home/hzx/catkin_ws/src/VINS-Fusion/dense/cmake-build-debug/main && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/cmvs.dir/cmvs.cc.o -c /home/hzx/catkin_ws/src/VINS-Fusion/dense/main/cmvs.cc

main/CMakeFiles/cmvs.dir/cmvs.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cmvs.dir/cmvs.cc.i"
	cd /home/hzx/catkin_ws/src/VINS-Fusion/dense/cmake-build-debug/main && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/hzx/catkin_ws/src/VINS-Fusion/dense/main/cmvs.cc > CMakeFiles/cmvs.dir/cmvs.cc.i

main/CMakeFiles/cmvs.dir/cmvs.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cmvs.dir/cmvs.cc.s"
	cd /home/hzx/catkin_ws/src/VINS-Fusion/dense/cmake-build-debug/main && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/hzx/catkin_ws/src/VINS-Fusion/dense/main/cmvs.cc -o CMakeFiles/cmvs.dir/cmvs.cc.s

# Object files for target cmvs
cmvs_OBJECTS = \
"CMakeFiles/cmvs.dir/cmvs.cc.o"

# External object files for target cmvs
cmvs_EXTERNAL_OBJECTS =

/home/hzx/catkin_ws/devel/lib/dense/cmvs: main/CMakeFiles/cmvs.dir/cmvs.cc.o
/home/hzx/catkin_ws/devel/lib/dense/cmvs: main/CMakeFiles/cmvs.dir/build.make
/home/hzx/catkin_ws/devel/lib/dense/cmvs: /home/hzx/catkin_ws/devel/lib/libimage_lib.so
/home/hzx/catkin_ws/devel/lib/dense/cmvs: /home/hzx/catkin_ws/devel/lib/libgraclus_lib.so
/home/hzx/catkin_ws/devel/lib/dense/cmvs: /usr/lib/x86_64-linux-gnu/libjpeg.so
/home/hzx/catkin_ws/devel/lib/dense/cmvs: /home/hzx/catkin_ws/devel/lib/libtinycthread.a
/home/hzx/catkin_ws/devel/lib/dense/cmvs: main/CMakeFiles/cmvs.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/hzx/catkin_ws/src/VINS-Fusion/dense/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/hzx/catkin_ws/devel/lib/dense/cmvs"
	cd /home/hzx/catkin_ws/src/VINS-Fusion/dense/cmake-build-debug/main && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/cmvs.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
main/CMakeFiles/cmvs.dir/build: /home/hzx/catkin_ws/devel/lib/dense/cmvs

.PHONY : main/CMakeFiles/cmvs.dir/build

main/CMakeFiles/cmvs.dir/clean:
	cd /home/hzx/catkin_ws/src/VINS-Fusion/dense/cmake-build-debug/main && $(CMAKE_COMMAND) -P CMakeFiles/cmvs.dir/cmake_clean.cmake
.PHONY : main/CMakeFiles/cmvs.dir/clean

main/CMakeFiles/cmvs.dir/depend:
	cd /home/hzx/catkin_ws/src/VINS-Fusion/dense/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/hzx/catkin_ws/src/VINS-Fusion/dense /home/hzx/catkin_ws/src/VINS-Fusion/dense/main /home/hzx/catkin_ws/src/VINS-Fusion/dense/cmake-build-debug /home/hzx/catkin_ws/src/VINS-Fusion/dense/cmake-build-debug/main /home/hzx/catkin_ws/src/VINS-Fusion/dense/cmake-build-debug/main/CMakeFiles/cmvs.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : main/CMakeFiles/cmvs.dir/depend

