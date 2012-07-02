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

set(FCL_VERSION "0.1.1")
set(PKG_DESC "Fast Collision Library")
set(PKG_EXTERNAL_DEPS "ccd flann")

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
pkg_check_modules(FLANN REQUIRED flann)
include_directories(${FLANN_INCLUDE_DIRS})
link_directories(${FLANN_LIBRARY_DIRS})

pkg_check_modules(CCD REQUIRED ccd)
include_directories(${CCD_INCLUDE_DIRS})
link_directories(${CCD_LIBRARY_DIRS})

add_definitions(-DUSE_SVMLIGHT=0)

add_library(${PROJECT_NAME} SHARED src/BV/AABB.cpp src/BV/OBB.cpp src/BV/RSS.cpp src/BV/kIOS.cpp src/BV/OBBRSS.cpp src/traversal_node_base.cpp src/traversal_node_bvhs.cpp src/intersect.cpp src/motion.cpp src/BV_fitter.cpp src/BV_splitter.cpp src/BVH_model.cpp src/BVH_utility.cpp src/transform.cpp src/simple_setup.cpp src/geometric_shapes.cpp src/geometric_shapes_utility.cpp src/gjk_libccd.cpp src/collision_node.cpp src/traversal_recurse.cpp src/broad_phase_collision.cpp src/collision.cpp src/collision_func_matrix.cpp src/interval_tree.cpp src/conservative_advancement.cpp src/matrix_3f.cpp src/interval.cpp src/interval_vector.cpp src/interval_matrix.cpp src/taylor_model.cpp src/taylor_vector.cpp src/taylor_matrix.cpp src/distance_func_matrix.cpp src/distance.cpp src/gjk.cpp src/narrowphase.cpp src/hierarchy_tree.cpp)

target_link_libraries(${PROJECT_NAME} ${FLANN_LIBRARIES} ${CCD_LIBRARIES})

set(pkg_conf_file "${CMAKE_CURRENT_SOURCE_DIR}/fcl.pc")
configure_file("${pkg_conf_file}.in" "${pkg_conf_file}" @ONLY)

install(TARGETS ${PROJECT_NAME} ARCHIVE DESTINATION lib/ LIBRARY DESTINATION lib/)
install(DIRECTORY include/ DESTINATION include FILES_MATCHING PATTERN "*.h")
install(FILES "${pkg_conf_file}" DESTINATION lib/pkgconfig/ COMPONENT pkgconfig)
