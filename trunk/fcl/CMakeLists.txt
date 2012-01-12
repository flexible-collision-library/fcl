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

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

add_definitions(-DUSE_PQP=1)
add_definitions(-DUSE_SVMLIGHT=0)

rosbuild_add_library(${PROJECT_NAME} src/AABB.cpp src/OBB.cpp src/RSS.cpp src/vec_3f.cpp src/traversal_node_base.cpp src/traversal_node_bvhs.cpp src/intersect.cpp src/motion.cpp src/BV_fitter.cpp src/BV_splitter.cpp src/BVH_model.cpp src/BVH_utility.cpp src/transform.cpp src/simple_setup.cpp src/geometric_shapes.cpp src/geometric_shapes_utility.cpp src/geometric_shapes_intersect.cpp src/collision_node.cpp src/traversal_recurse.cpp src/broad_phase_collision.cpp src/collision.cpp src/collision_func_matrix.cpp src/interval_tree.cpp src/conservative_advancement.cpp src/matrix_3f.cpp src/interval.cpp src/interval_vector.cpp src/interval_matrix.cpp src/taylor_model.cpp src/taylor_vector.cpp src/taylor_matrix.cpp)
target_link_libraries(fcl)

rosbuild_add_gtest(test_core_collision test/test_core_collision.cpp)
target_link_libraries(test_core_collision fcl)

rosbuild_add_gtest(test_core_distance test/test_core_distance.cpp)
target_link_libraries(test_core_distance fcl)

rosbuild_add_executable(test_core_collision_point test/test_core_collision_point.cpp)
target_link_libraries(test_core_collision_point fcl)

rosbuild_add_gtest(test_core_geometric_shapes test/test_core_geometric_shapes.cpp)
target_link_libraries(test_core_geometric_shapes fcl)

rosbuild_add_gtest(test_core_collision_shape_mesh_consistency test/test_core_collision_shape_mesh_consistency.cpp)
target_link_libraries(test_core_collision_shape_mesh_consistency fcl)

rosbuild_add_gtest(test_core_broad_phase test/test_core_broad_phase.cpp)
target_link_libraries(test_core_broad_phase fcl)

rosbuild_add_gtest(test_core_front_list test/test_core_front_list.cpp)
target_link_libraries(test_core_front_list fcl)

rosbuild_add_gtest(test_core_continuous_collision test/test_core_continuous_collision.cpp)
target_link_libraries(test_core_continuous_collision fcl)

rosbuild_add_executable(test_core_deformable_object test/test_core_deformable_object.cpp)
target_link_libraries(test_core_deformable_object fcl assimp)

rosbuild_add_executable(test_core_conservative_advancement test/test_core_conservative_advancement.cpp)
target_link_libraries(test_core_conservative_advancement fcl)

rosbuild_add_executable(timing_test test/timing_test.cpp test/timer.cpp)
target_link_libraries(timing_test fcl)