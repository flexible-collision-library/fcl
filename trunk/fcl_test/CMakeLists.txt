cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)
set(ROS_BUILD_TYPE Release)

rosbuild_init()

add_definitions(-DUSE_PQP=1)
add_definitions(-DUSE_SVMLIGHT=0)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_gtest(test_core_collision test/test_core_collision.cpp)

rosbuild_add_gtest(test_core_distance test/test_core_distance.cpp)

rosbuild_add_executable(test_core_collision_point test/test_core_collision_point.cpp)

rosbuild_add_gtest(test_core_geometric_shapes test/test_core_geometric_shapes.cpp)

rosbuild_add_gtest(test_core_collision_shape_mesh_consistency test/test_core_collision_shape_mesh_consistency.cpp)

rosbuild_add_gtest(test_core_broad_phase test/test_core_broad_phase.cpp)

rosbuild_add_gtest(test_core_front_list test/test_core_front_list.cpp)

rosbuild_add_gtest(test_core_continuous_collision test/test_core_continuous_collision.cpp)

rosbuild_add_executable(test_core_deformable_object test/test_core_deformable_object.cpp)

rosbuild_add_executable(test_core_conservative_advancement test/test_core_conservative_advancement.cpp)

rosbuild_add_executable(timing_test test/timing_test.cpp test/timer.cpp)
