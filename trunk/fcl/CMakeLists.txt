cmake_minimum_required(VERSION 2.4.6)
project(fcl CXX C)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)
set(CMAKE_BUILD_TYPE Release)

set(CMAKE_SKIP_BUILD_RPATH  FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories("include")

find_package(PkgConfig)
pkg_check_modules(CCD REQUIRED ccd)
include_directories(${CCD_INCLUDE_DIRS})
link_directories(${CCD_LIBRARY_DIRS})

pkg_check_modules(OCTOMAP QUIET octomap)
if(OCTOMAP_FOUND)
    include_directories(${OCTOMAP_INCLUDE_DIRS})
    link_directories(${OCTOMAP_LIBRARY_DIRS})
endif()
add_definitions(-DUSE_SVMLIGHT=0)

add_library(${PROJECT_NAME} SHARED src/BV/AABB.cpp src/BV/OBB.cpp src/BV/RSS.cpp src/BV/kIOS.cpp src/BV/OBBRSS.cpp src/BV/kDOP.cpp src/traversal/traversal_node_base.cpp src/traversal/traversal_node_bvhs.cpp src/intersect.cpp src/ccd/motion.cpp src/BVH/BV_fitter.cpp src/BVH/BV_splitter.cpp src/BVH/BVH_model.cpp src/BVH/BVH_utility.cpp src/math/transform.cpp src/traversal/traversal_node_setup.cpp src/shape/geometric_shapes.cpp src/shape/geometric_shapes_utility.cpp src/collision_node.cpp src/traversal/traversal_recurse.cpp src/broadphase/broadphase_bruteforce.cpp src/broadphase/broadphase_spatialhash.cpp src/broadphase/broadphase_SaP.cpp src/broadphase/broadphase_SSaP.cpp src/broadphase/broadphase_interval_tree.cpp src/broadphase/broadphase_dynamic_AABB_tree.cpp src/broadphase/broadphase_dynamic_AABB_tree_array.cpp src/collision.cpp src/collision_func_matrix.cpp src/broadphase/interval_tree.cpp src/ccd/conservative_advancement.cpp src/ccd/interval.cpp src/ccd/interval_vector.cpp src/ccd/interval_matrix.cpp src/ccd/taylor_model.cpp src/ccd/taylor_vector.cpp src/ccd/taylor_matrix.cpp src/distance_func_matrix.cpp src/distance.cpp src/narrowphase/gjk.cpp src/narrowphase/gjk_libccd.cpp src/narrowphase/narrowphase.cpp src/broadphase/hierarchy_tree.cpp src/profile.cpp src/collision_data.cpp)


target_link_libraries(${PROJECT_NAME} ${CCD_LIBRARIES} ${OCTOMAP_LIBRARIES})

set(pkg_conf_file "${CMAKE_CURRENT_SOURCE_DIR}/fcl.pc")
configure_file("${pkg_conf_file}.in" "${pkg_conf_file}" @ONLY)

install(TARGETS ${PROJECT_NAME} ARCHIVE DESTINATION lib/ LIBRARY DESTINATION lib/)
install(DIRECTORY include/ DESTINATION include FILES_MATCHING PATTERN "*.h")
install(FILES "${pkg_conf_file}" DESTINATION lib/pkgconfig/ COMPONENT pkgconfig)
