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

# Utility rule file for run_tests_cv_camera_rostest_test_no_yaml.test.

# Include the progress variables for this target.
include cv_camera-master/CMakeFiles/run_tests_cv_camera_rostest_test_no_yaml.test.dir/progress.make

cv_camera-master/CMakeFiles/run_tests_cv_camera_rostest_test_no_yaml.test:
	cd /home/sz/unpeople/build/cv_camera-master && ../catkin_generated/env_cached.sh /usr/bin/python /opt/ros/kinetic/share/catkin/cmake/test/run_tests.py /home/sz/unpeople/build/test_results/cv_camera/rostest-test_no_yaml.xml "/opt/ros/kinetic/share/rostest/cmake/../../../bin/rostest --pkgdir=/home/sz/unpeople/src/cv_camera-master --package=cv_camera --results-filename test_no_yaml.xml --results-base-dir \"/home/sz/unpeople/build/test_results\" /home/sz/unpeople/src/cv_camera-master/test/no_yaml.test "

run_tests_cv_camera_rostest_test_no_yaml.test: cv_camera-master/CMakeFiles/run_tests_cv_camera_rostest_test_no_yaml.test
run_tests_cv_camera_rostest_test_no_yaml.test: cv_camera-master/CMakeFiles/run_tests_cv_camera_rostest_test_no_yaml.test.dir/build.make

.PHONY : run_tests_cv_camera_rostest_test_no_yaml.test

# Rule to build all files generated by this target.
cv_camera-master/CMakeFiles/run_tests_cv_camera_rostest_test_no_yaml.test.dir/build: run_tests_cv_camera_rostest_test_no_yaml.test

.PHONY : cv_camera-master/CMakeFiles/run_tests_cv_camera_rostest_test_no_yaml.test.dir/build

cv_camera-master/CMakeFiles/run_tests_cv_camera_rostest_test_no_yaml.test.dir/clean:
	cd /home/sz/unpeople/build/cv_camera-master && $(CMAKE_COMMAND) -P CMakeFiles/run_tests_cv_camera_rostest_test_no_yaml.test.dir/cmake_clean.cmake
.PHONY : cv_camera-master/CMakeFiles/run_tests_cv_camera_rostest_test_no_yaml.test.dir/clean

cv_camera-master/CMakeFiles/run_tests_cv_camera_rostest_test_no_yaml.test.dir/depend:
	cd /home/sz/unpeople/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/sz/unpeople/src /home/sz/unpeople/src/cv_camera-master /home/sz/unpeople/build /home/sz/unpeople/build/cv_camera-master /home/sz/unpeople/build/cv_camera-master/CMakeFiles/run_tests_cv_camera_rostest_test_no_yaml.test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : cv_camera-master/CMakeFiles/run_tests_cv_camera_rostest_test_no_yaml.test.dir/depend
