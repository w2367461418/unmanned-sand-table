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
CMAKE_SOURCE_DIR = /home/sz/unpeople/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/sz/unpeople/build

# Utility rule file for _run_tests_cv_camera_rostest_test_cv_camera.test.

# Include the progress variables for this target.
include cv_camera-master/CMakeFiles/_run_tests_cv_camera_rostest_test_cv_camera.test.dir/progress.make

cv_camera-master/CMakeFiles/_run_tests_cv_camera_rostest_test_cv_camera.test:
	cd /home/sz/unpeople/build/cv_camera-master && ../catkin_generated/env_cached.sh /usr/bin/python /opt/ros/kinetic/share/catkin/cmake/test/run_tests.py /home/sz/unpeople/build/test_results/cv_camera/rostest-test_cv_camera.xml "/opt/ros/kinetic/share/rostest/cmake/../../../bin/rostest --pkgdir=/home/sz/unpeople/src/cv_camera-master --package=cv_camera --results-filename test_cv_camera.xml --results-base-dir \"/home/sz/unpeople/build/test_results\" /home/sz/unpeople/src/cv_camera-master/test/cv_camera.test "

_run_tests_cv_camera_rostest_test_cv_camera.test: cv_camera-master/CMakeFiles/_run_tests_cv_camera_rostest_test_cv_camera.test
_run_tests_cv_camera_rostest_test_cv_camera.test: cv_camera-master/CMakeFiles/_run_tests_cv_camera_rostest_test_cv_camera.test.dir/build.make

.PHONY : _run_tests_cv_camera_rostest_test_cv_camera.test

# Rule to build all files generated by this target.
cv_camera-master/CMakeFiles/_run_tests_cv_camera_rostest_test_cv_camera.test.dir/build: _run_tests_cv_camera_rostest_test_cv_camera.test

.PHONY : cv_camera-master/CMakeFiles/_run_tests_cv_camera_rostest_test_cv_camera.test.dir/build

cv_camera-master/CMakeFiles/_run_tests_cv_camera_rostest_test_cv_camera.test.dir/clean:
	cd /home/sz/unpeople/build/cv_camera-master && $(CMAKE_COMMAND) -P CMakeFiles/_run_tests_cv_camera_rostest_test_cv_camera.test.dir/cmake_clean.cmake
.PHONY : cv_camera-master/CMakeFiles/_run_tests_cv_camera_rostest_test_cv_camera.test.dir/clean

cv_camera-master/CMakeFiles/_run_tests_cv_camera_rostest_test_cv_camera.test.dir/depend:
	cd /home/sz/unpeople/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/sz/unpeople/src /home/sz/unpeople/src/cv_camera-master /home/sz/unpeople/build /home/sz/unpeople/build/cv_camera-master /home/sz/unpeople/build/cv_camera-master/CMakeFiles/_run_tests_cv_camera_rostest_test_cv_camera.test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : cv_camera-master/CMakeFiles/_run_tests_cv_camera_rostest_test_cv_camera.test.dir/depend
