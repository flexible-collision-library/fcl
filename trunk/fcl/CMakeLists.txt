cmake_minimum_required(VERSION 2.4.6)
project(fcl CXX C)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries

set(CMAKE_BUILD_TYPE Release)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules")


# Check for SSE flags
set(FCL_HAVE_SSE 0)
include(FCLCheckSSE)
fcl_check_for_sse()
if (SSE_FLAGS)
  message(STATUS "FCL uses SSE")
  add_definitions(${SSE_FLAGS})
  set(FCL_HAVE_SSE 0)
  # waiting for Jia to fix this; not using sse anyway
else()
  message(STATUS "FCL does not use SSE")
endif()

# make sure we know what flag we used for SSE 
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


add_subdirectory(include/fcl)
add_subdirectory(src)

set(pkg_conf_file "${CMAKE_CURRENT_SOURCE_DIR}/fcl.pc")
configure_file("${pkg_conf_file}.in" "${pkg_conf_file}" @ONLY)

install(DIRECTORY include/ DESTINATION include FILES_MATCHING PATTERN "*.h")
install(FILES "${pkg_conf_file}" DESTINATION lib/pkgconfig/ COMPONENT pkgconfig)
