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

add_definitions(-DUSE_SVMLIGHT=0)

rosbuild_add_library(${PROJECT_NAME} src/AABB.cpp src/OBB.cpp src/RSS.cpp src/vec_3f.cpp src/traversal_node_base.cpp src/traversal_node_bvhs.cpp src/intersect.cpp src/motion.cpp src/BV_fitter.cpp src/BV_splitter.cpp src/BVH_model.cpp src/BVH_utility.cpp src/transform.cpp src/simple_setup.cpp src/geometric_shapes.cpp src/geometric_shapes_utility.cpp src/geometric_shapes_intersect.cpp src/collision_node.cpp src/traversal_recurse.cpp src/broad_phase_collision.cpp src/collision.cpp src/collision_func_matrix.cpp src/interval_tree.cpp src/conservative_advancement.cpp src/matrix_3f.cpp src/interval.cpp src/interval_vector.cpp src/interval_matrix.cpp src/taylor_model.cpp src/taylor_vector.cpp src/taylor_matrix.cpp)
