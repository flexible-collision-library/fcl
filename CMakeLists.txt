cmake_minimum_required(VERSION 2.8)
project(fcl CXX C)

# set the default build type
if (NOT CMAKE_BUILD_TYPE)
   set(CMAKE_BUILD_TYPE Release)
endif()

# This shouldn't be necessary, but there has been trouble
# with MSVC being set off, but MSVCXX ON.
if(MSVC OR MSVC90 OR MSVC10)
    set(MSVC ON)
endif (MSVC OR MSVC90 OR MSVC10)

set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules")
include(CompilerSettings)
include(FCLVersion)

if(MSVC OR IS_ICPC)
  option(FCL_STATIC_LIBRARY "Whether the FCL library should be static rather than shared" ON)
else()
  option(FCL_STATIC_LIBRARY "Whether the FCL library should be static rather than shared" OFF)
endif()

# Whether to enable SSE
option(FCL_USE_SSE "Whether FCL should SSE instructions" OFF)
set(FCL_HAVE_SSE 0)
if(FCL_USE_SSE)
  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    set(FCL_HAVE_SSE 0) #always disable, for now
    add_definitions(-march=native)
  endif()
  # TODO: do something similar for other compilers
endif()

# Find Octomap (optional)
find_package(PkgConfig QUIET)
set(FCL_HAVE_OCTOMAP 0)
if(PKG_CONFIG_FOUND)
    pkg_check_modules(OCTOMAP QUIET octomap)
endif()
if(NOT OCTOMAP_FOUND)
    # if pkfconfig is not installed, then fall back on more fragile detection
    # of octomap
    find_path(OCTOMAP_INCLUDE_DIRS octomap.h
        PATH_SUFFIXES octomap)
    find_path(OCTOMAP_LIBRARY_DIRS
        ${CMAKE_SHARED_LIBRARY_PREFIX}octomap${CMAKE_SHARED_LIBRARY_SUFFIX})
    if(OCTOMAP_INCLUDE_DIRS AND OCTOMAP_LIBRARY_DIRS)
        set(OCTOMAP_LIBRARIES "octomap;octomath")
    endif()
endif()
if (OCTOMAP_INCLUDE_DIRS AND OCTOMAP_LIBRARY_DIRS)
  include_directories(${OCTOMAP_INCLUDE_DIRS})
  link_directories(${OCTOMAP_LIBRARY_DIRS})
  set(FCL_HAVE_OCTOMAP 1)
  message(STATUS "FCL uses Octomap")
else()
  message(STATUS "FCL does not use Octomap")
endif()

# Find FLANN (optional)
find_package(flann QUIET)
if (FLANN_FOUND)
  set(FCL_HAVE_FLANN 1)
  include_directories(${FLANN_INCLUDE_DIRS})
  link_directories(${FLANN_LIBRARY_DIRS})
  message(STATUS "FCL uses Flann")
else()
  message(STATUS "FCL does not use Flann")
endif()


find_package(tinyxml QUIET)
if (TINYXML_FOUND)
  set(FCL_HAVE_TINYXML 1)
  include_directories(${TINYXML_INCLUDE_DIRS})
  link_directories(${TINYXML_LIBRARY_DIRS})
  message(STATUS "FCL uses tinyxml")
else()
  message(STATUS "FCL does not use tinyxml")
endif()


find_package(Boost COMPONENTS thread date_time filesystem system unit_test_framework REQUIRED)
include_directories(${Boost_INCLUDE_DIR})

if(MSVC)
    add_definitions(-DBOOST_ALL_NO_LIB)
endif()
add_definitions(-DBOOST_TEST_DYN_LINK)

# FCL's own include dir should be at the front of the include path
include_directories(BEFORE "include")

if(PKG_CONFIG_FOUND)
  pkg_check_modules(CCD ccd)
  # check to see if the pkg is installed under the libccd name
  if(NOT CCD_FOUND)
    pkg_check_modules(CCD libccd)
  endif()
endif()

if(NOT CCD_FOUND)
    # if pkfconfig is not installed, then fall back on more fragile detection
    # of ccd
    find_path(CCD_INCLUDE_DIRS ccd.h
        PATH_SUFFIXES ccd)
    find_path(CCD_LIBRARY_DIRS
        ${CMAKE_SHARED_LIBRARY_PREFIX}ccd${CMAKE_SHARED_LIBRARY_SUFFIX})
    if(CCD_INCLUDE_DIRS AND CCD_LIBRARY_DIRS)
        set(CCD_LIBRARIES "ccd")
    else()
        message(FATAL_ERROR "Libccd is required by FCL")
    endif()
endif()
include_directories(${CCD_INCLUDE_DIRS})
link_directories(${CCD_LIBRARY_DIRS})

add_subdirectory(include/fcl)
add_subdirectory(src)

set(pkg_conf_file "${CMAKE_CURRENT_SOURCE_DIR}/fcl.pc")
configure_file("${pkg_conf_file}.in" "${pkg_conf_file}" @ONLY)

install(DIRECTORY include/ DESTINATION include
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hxx"
  PATTERN ".DS_Store" EXCLUDE
)

install(FILES "${pkg_conf_file}" DESTINATION lib/pkgconfig/ COMPONENT pkgconfig)


enable_testing()
add_subdirectory(test)
