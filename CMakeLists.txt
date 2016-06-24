cmake_minimum_required(VERSION 2.8.12)

# Use MACOSX_RPATH by default on OS X. This was added in CMake 2.8.12 and
# became default in CMake 3.0. Explicitly setting this policy is necessary to
# suppress a warning in CMake 3.0 and above.
if(POLICY CMP0042)
  cmake_policy(SET CMP0042 NEW)
endif()

project(fcl CXX C)

# set the default build type
if (NOT CMAKE_BUILD_TYPE)
   set(CMAKE_BUILD_TYPE Release)
endif()

# Set build type variable
set(FCL_BUILD_TYPE_RELEASE FALSE)
set(FCL_BUILD_TYPE_DEBUG FALSE)

string(TOUPPER ${CMAKE_BUILD_TYPE} CMAKE_BUILD_TYPE_UPPERCASE)
if("${CMAKE_BUILD_TYPE_UPPERCASE}" STREQUAL "RELEASE")
  set(FCL_BUILD_TYPE_RELEASE TRUE)
elseif("${CMAKE_BUILD_TYPE_UPPERCASE}" STREQUAL "DEBUG")
  set(FCL_BUILD_TYPE_DEBUG TRUE)
else()
  message(STATUS "CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} unknown. Valid options are: Debug Release")
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
include(GNUInstallDirs)

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

# Coveralls support
option(FCL_COVERALLS "Turn on coveralls support" OFF)
option(FCL_COVERALLS_UPLOAD "Upload the generated coveralls json" ON)
if(FCL_COVERALLS)
  include(Coveralls)
  coveralls_turn_on_coverage()
endif()

# Find Octomap (optional)
find_package(PkgConfig QUIET)

option(FCL_WITH_OCTOMAP "octomap library support" ON)
set(FCL_HAVE_OCTOMAP 0)
if(FCL_WITH_OCTOMAP)
  if(PKG_CONFIG_FOUND)
      pkg_check_modules(OCTOMAP QUIET octomap)
  endif()
  if(NOT OCTOMAP_FOUND)
      # if pkgconfig is not installed, then fall back on more fragile detection
      # of octomap
      find_path(OCTOMAP_INCLUDE_DIRS octomap.h
          PATH_SUFFIXES octomap)
      find_library(OCTOMAP_LIBRARY_DIRS
          ${CMAKE_SHARED_LIBRARY_PREFIX}octomap${CMAKE_SHARED_LIBRARY_SUFFIX})
      if(OCTOMAP_INCLUDE_DIRS AND OCTOMAP_LIBRARY_DIRS)
          set(OCTOMAP_LIBRARIES "octomap;octomath")
      endif()
  endif()
  if (OCTOMAP_FOUND OR (OCTOMAP_INCLUDE_DIRS AND OCTOMAP_LIBRARY_DIRS))
    string(REPLACE "." ";" VERSION_LIST ${OCTOMAP_VERSION})
    list(GET VERSION_LIST 0 OCTOMAP_MAJOR_VERSION)
    list(GET VERSION_LIST 1 OCTOMAP_MINOR_VERSION)
    list(GET VERSION_LIST 2 OCTOMAP_PATCH_VERSION)
    include_directories(${OCTOMAP_INCLUDE_DIRS})
    link_directories(${OCTOMAP_LIBRARY_DIRS})
    set(FCL_HAVE_OCTOMAP 1)
    message(STATUS "FCL uses Octomap")
  else()
    message(STATUS "FCL does not use Octomap")
  endif()
else()
  message(STATUS "FCL does not use Octomap (as requested)")
endif()


# find_package(tinyxml QUIET)
# if (TINYXML_FOUND)
#   set(FCL_HAVE_TINYXML 1)
#   include_directories(${TINYXML_INCLUDE_DIRS})
#   link_directories(${TINYXML_LIBRARY_DIRS})
#   message(STATUS "FCL uses tinyxml")
# else()
#   message(STATUS "FCL does not use tinyxml")
# endif()


# FCL's own include dir should be at the front of the include path
include_directories(BEFORE "include")
include_directories("${CMAKE_CURRENT_BINARY_DIR}/include")

if(PKG_CONFIG_FOUND)
  pkg_check_modules(CCD ccd)
  # check to see if the pkg is installed under the libccd name
  if(NOT CCD_FOUND)
    pkg_check_modules(CCD libccd)
  endif()
endif()

if(NOT CCD_FOUND)
    # if pkgconfig is not installed, then fall back on more fragile detection
    # of ccd
    find_path(CCD_INCLUDE_DIRS ccd/ccd.h)
    find_library(CCD_LIBRARY
        ${CMAKE_SHARED_LIBRARY_PREFIX}ccd${CMAKE_SHARED_LIBRARY_SUFFIX})
    if(CCD_INCLUDE_DIRS AND CCD_LIBRARY)
        set(CCD_LIBRARIES "${CCD_LIBRARY}")
    else()
        message(FATAL_ERROR "Libccd is required by FCL")
    endif()
endif()
include_directories(${CCD_INCLUDE_DIRS})
link_directories(${CCD_LIBRARY_DIRS})

add_subdirectory(include/fcl)
add_subdirectory(src)

set(pkg_conf_file_in "${CMAKE_CURRENT_SOURCE_DIR}/fcl.pc.in")
set(pkg_conf_file_out "${CMAKE_CURRENT_BINARY_DIR}/fcl.pc")
if(NOT MSVC)
    set(PKG_CFLAGS "-std=c++11")
endif()
configure_file("${pkg_conf_file_in}" "${pkg_conf_file_out}" @ONLY)

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hxx"
  PATTERN ".DS_Store" EXCLUDE
)

install(FILES "${pkg_conf_file_out}" DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig/ COMPONENT pkgconfig)

# Add uninstall target
configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules/cmake_uninstall.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/CMakeModules/cmake_uninstall.cmake"
  IMMEDIATE @ONLY)
add_custom_target(uninstall
  "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_BINARY_DIR}/CMakeModules/cmake_uninstall.cmake")

find_package(Boost COMPONENTS date_time filesystem system unit_test_framework)
option(FCL_BUILD_TESTS "Build FCL tests" ${Boost_FOUND})
if(FCL_BUILD_TESTS)
    enable_testing()
    add_subdirectory(test)
endif()
