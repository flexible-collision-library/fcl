# -*- mode: cmake -*-
# vi: set ft=cmake :

# Copyright (c) 2012, Willow Garage, Inc.
# Copyright (c) 2016, Toyota Research Institute, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# * Neither the name of the copyright holder nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

if(APPLE)
  cmake_minimum_required(VERSION 3.0.0)
elseif(MSVC)
  cmake_minimum_required(VERSION 3.1.3)
else()
  cmake_minimum_required(VERSION 2.8.12)
endif()

project(fcl CXX)

# Compiler ID for Apple Clang is now AppleClang.
if(POLICY CMP0025)
  cmake_policy(SET CMP0025 NEW)
endif()

include(CTest)

configure_file(CTestCustom.cmake.in CTestCustom.cmake @ONLY)

option(FCL_ENABLE_PROFILING "Enable profiling" OFF)
option(FCL_TREAT_WARNINGS_AS_ERRORS "Treat warnings as errors" OFF)
# Option for some bundle-like build system in order not to expose
# any FCL binary symbols in their public ABI
option(FCL_HIDE_ALL_SYMBOLS "Hide all binary symbols" OFF)

# set the default build type
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING
    "Choose the type of build; options are Debug Release RelWithDebInfo MinSizeRel."
    FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY
    STRINGS Debug Release RelWithDebInfo MinSizeRel)
endif()

# This shouldn't be necessary, but there has been trouble
# with MSVC being set off, but MSVCXX ON.
if(MSVC OR MSVC90 OR MSVC10)
    set(MSVC ON)
endif (MSVC OR MSVC90 OR MSVC10)

set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules")
include(GenerateExportHeader)
include(GNUInstallDirs)
include(FCLMacros)
include(CompilerSettings)
include(FCLVersion)

if(MSVC OR IS_ICPC)
  option(FCL_STATIC_LIBRARY "Whether the FCL library should be static rather than shared" ON)
else()
  option(FCL_STATIC_LIBRARY "Whether the FCL library should be static rather than shared" OFF)
endif()

# Whether to enable SSE
set(SSE_FLAGS "")
option(FCL_USE_X64_SSE "Whether FCL should x64 SSE instructions" ON)
if(FCL_USE_X64_SSE)
  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    set(SSE_FLAGS -mfpmath=sse -msse -msse2 -msse3 -mssse3)
  elseif(MSVC)
    # Win64 will add the flag automatically
    if(CMAKE_VS_PLATFORM_NAME STREQUAL "Win32")
      set(SSE_FLAGS /arch:SSE2)
    endif()
  endif()
  add_compile_options(${SSE_FLAGS})
endif()

option(FCL_USE_HOST_NATIVE_ARCH "Whether FCL should use cflags from the host used to compile" OFF)
if (FCL_USE_HOST_NATIVE_ARCH)
  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")
  else()
    message(WARNING "FCL_USE_HOST_NATIVE_ARCH is only supported in Linux. No effect.")
  endif()
endif()

# DEPRECATED: old cmake option. Not strictly correct from the semantic point of view
# it was activating march=native, not only SSE
option(FCL_USE_SSE "(deprecated) Whether FCL should SSE instructions" OFF)
if(FCL_USE_SSE)
  set(FCL_HAVE_SSE TRUE)
  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    message(WARNING "FCL_USE_SSE is deprecated please use: FCL_USE_X64_SSE or FCL_USE_HOST_NATIVE_ARCH. "
                    "If you want to replicate the previous behaviour use FCL_USE_HOST_NATIVE_ARCH")
    add_definitions(-march=native)
  elseif(MSVC)
    # Win64 will add the flag automatically
    if("$CMAKE_VS_PLATFORM_NAME}" STREQUAL "Win32")
      add_definitions(/arch:SSE2)
    endif()
  endif()
endif()

# Coveralls support
option(FCL_COVERALLS "Turn on coveralls support" OFF)
option(FCL_COVERALLS_UPLOAD "Upload the generated coveralls json" ON)
if(FCL_COVERALLS)
  include(Coveralls)
  coveralls_turn_on_coverage()
endif()

find_package(PkgConfig QUIET)
set(PKG_CONFIG_USE_CMAKE_PREFIX_PATH ON)

#===============================================================================
# Find required dependency Eigen3 (>= 3.0.5)
#
# If Eigen3 is not found, manually set the cache variable EIGEN3_INCLUDE_DIR
#===============================================================================
find_package(Eigen3 3.0.5 QUIET CONFIG)

# If Eigen3Config.cmake is not found, use the FindEigen3.cmake module
if(NOT Eigen3_FOUND)
  find_package(Eigen3 3.0.5 QUIET MODULE)
  set(Eigen3_FOUND ON)
endif()

if(Eigen3_FOUND)
  set(FCL_HAVE_EIGEN TRUE)
else()
  message(SEND_ERROR "EIGEN3 (>= 3.0.5) is required by FCL")
  set(FCL_HAVE_EIGEN FALSE)
endif()

#===============================================================================
# Find required dependency libccd
#
# If libccd is not found, manually set the cache variables CCD_INCLUDE_DIR and
# CCD_LIBRARY
#===============================================================================
find_package(ccd QUIET)

# If ccd-config.cmake is not found, use pkg-config and/or find_path() and
# find_library()
if(NOT ccd_FOUND)
  if(PKG_CONFIG_FOUND)
    pkg_check_modules(PC_CCD ccd)
    pkg_check_modules(PC_LIBCCD libccd)
  endif()

  find_path(CCD_INCLUDE_DIR ccd/ccd.h
    HINTS "${PC_CCD_INCLUDE_DIRS}" "${PC_LIBCCD_INCLUDE_DIRS}")

  # Using find_library() even if pkg-config is available ensures that the full
  # path to the ccd library is available in CCD_LIBRARIES
  find_library(CCD_LIBRARY ccd
    HINTS "${PC_CCD_LIBRARY_DIRS}" "${PC_LIBCCD_LIBRARY_DIRS}")

  # libccd links to LibM on UNIX.
  if(CYGWIN OR NOT WIN32)
    find_library(M_LIBRARY m)
  endif()

  if(CCD_INCLUDE_DIR AND CCD_LIBRARY)
    set(CCD_INCLUDE_DIRS "${CCD_INCLUDE_DIR}")
    set(CCD_LIBRARIES "${CCD_LIBRARY}" "${M_LIBRARY}")
    set(ccd_FOUND ON)

    mark_as_advanced(CCD_INCLUDE_DIR CCD_LIBRARY)
  endif()
endif()

if(NOT ccd_FOUND)
  message(FATAL_ERROR "CCD is required by FCL")
endif()

set(PKG_EXTERNAL_DEPS "ccd eigen3")

#===============================================================================
# Find optional dependency OctoMap
#
# If OctoMap is not found, manually set the cache variables OCTOMAP_INCLUDE_DIR
# and OCTOMAP_LIBRARY, OCTOMATH_LIBRARY, and OCTOMAP_VERSION
#===============================================================================
option(FCL_WITH_OCTOMAP "OctoMap library support" ON)
set(FCL_HAVE_OCTOMAP 0)

if(FCL_WITH_OCTOMAP)
  find_package(octomap QUIET)

  # Older versions of octomap-config.cmake may not define OCTOMAP_VERSION so
  # fall back to pkg-config
  if(NOT octomap_FOUND OR NOT OCTOMAP_VERSION)
    if(PKG_CONFIG_FOUND)
      pkg_check_modules(PC_OCTOMAP octomap)
    endif()

    find_path(OCTOMAP_INCLUDE_DIR octomap/octomap.h
      HINTS "${PC_OCTOMAP_INCLUDE_DIRS}")

    # Using find_library() even if pkg-config is available ensures that the full
    # paths to the octomap and octomath libraries are set in OCTOMAP_LIBRARIES
    find_library(OCTOMAP_LIBRARY octomap
      HINTS "${PC_OCTOMAP_LIBRARY_DIRS}")

    find_library(OCTOMATH_LIBRARY octomath
      HINTS "${PC_OCTOMAP_LIBRARY_DIRS}")

    # Use a cache variable so that the version can be manually set if pkg-config
    # is not available
    set(OCTOMAP_VERSION "${PC_OCTOMAP_VERSION}"
      CACHE STRING "octomap version (major.minor.patch)")

    if(OCTOMAP_INCLUDE_DIR AND OCTOMAP_LIBRARY AND OCTOMATH_LIBRARY AND OCTOMAP_VERSION)
      set(OCTOMAP_INCLUDE_DIRS "${OCTOMAP_INCLUDE_DIR}")
      set(OCTOMAP_LIBRARIES "${OCTOMAP_LIBRARY}" "${OCTOMATH_LIBRARY}")
      set(octomap_FOUND ON)

      mark_as_advanced(OCTOMAP_INCLUDE_DIR OCTOMAP_LIBRARY OCTOMATH_LIBRARY OCTOMAP_VERSION)
    else()
      set(octomap_FOUND OFF)
    endif()
  endif()

  if(octomap_FOUND)
    if(NOT OCTOMAP_MAJOR_VERSION AND NOT OCTOMAP_MINOR_VERSION AND NOT OCTOMAP_PATCH_VERSION)
      string(REPLACE "." ";" VERSION_LIST "${OCTOMAP_VERSION}")
      list(GET VERSION_LIST 0 OCTOMAP_MAJOR_VERSION)
      list(GET VERSION_LIST 1 OCTOMAP_MINOR_VERSION)
      list(GET VERSION_LIST 2 OCTOMAP_PATCH_VERSION)
    endif()

    set(FCL_HAVE_OCTOMAP 1)
    message(STATUS "FCL uses OctoMap")
    set(PKG_EXTERNAL_DEPS "${PKG_EXTERNAL_DEPS} octomap")
  else()
    message(STATUS "FCL does not use OctoMap")
  endif()
else()
  message(STATUS "FCL does not use OctoMap (as requested)")
endif()

# FCL's own include dir should be at the front of the include path
include_directories(BEFORE "include")
include_directories(BEFORE "${CMAKE_CURRENT_BINARY_DIR}/include")

add_subdirectory(src)
add_subdirectory(include/fcl)

set(pkg_conf_file_in "${CMAKE_CURRENT_SOURCE_DIR}/fcl.pc.in")
set(pkg_conf_file_out "${CMAKE_CURRENT_BINARY_DIR}/fcl.pc")
set(PKG_DESC "Flexible Collision Library")
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

set(FCL_BUILD_TESTS "DEFAULT" CACHE INTERNAL "Deprecated; use BUILD_TESTING instead.")

if(NOT FCL_BUILD_TESTS STREQUAL "DEFAULT")
  message(DEPRECATION "FCL_BUILD_TESTS is deprecated; use BUILD_TESTING instead.")
  if(FCL_BUILD_TESTS)
    set(_BUILD_TESTING ON)
  else()
    set(_BUILD_TESTING OFF)
  endif()
  set(BUILD_TESTING ${_BUILD_TESTING} CACHE BOOL "Build the testing tree." FORCE)
  unset(_BUILD_TESTING)
endif()

if(BUILD_TESTING AND NOT FCL_HIDE_ALL_SYMBOLS)
    add_subdirectory(test)
endif()

#===============================================================================
# API documentation using Doxygen
# References:
#   http://mementocodex.wordpress.com/2013/01/19/how-to-generate-code-documentation-with-doxygen-and-cmake-a-slightly-improved-approach/
#   http://www.cmake.org/pipermail/cmake/2007-February/012796.html
#===============================================================================
# Doxygen
find_package(Doxygen QUIET)

if(DOXYGEN_FOUND)

  set(DOXYGEN_DOXYFILE_IN      ${PROJECT_SOURCE_DIR}/doc/Doxyfile.in     )
  set(DOXYGEN_DOXYFILE         ${PROJECT_BINARY_DIR}/doc/Doxyfile        )
  set(DOXYGEN_HTML_INDEX       ${PROJECT_SOURCE_DIR}/doc/doxygen/index.html )
  set(DOXYGEN_OUTPUT_ROOT      ${PROJECT_SOURCE_DIR}/doc/doxygen            )
  set(DOXYGEN_GENERATE_TAGFILE ${DOXYGEN_OUTPUT_ROOT}/${PROJECT_NAME}.tag)
  set(DOXYGEN_INCLUDE_PATH     ${PROJECT_SOURCE_DIR}                     )
  set(DOXYGEN_INPUT_ROOT       "${PROJECT_SOURCE_DIR}/include ${PROJECT_SOURCE_DIR}/src")

  configure_file(${DOXYGEN_DOXYFILE_IN} ${DOXYGEN_DOXYFILE} @ONLY)
  add_custom_command(OUTPUT ${DOXYGEN_HTML_INDEX}
                     COMMAND ${CMAKE_COMMAND} -E echo_append "Building API Documentation..."
                     COMMAND ${DOXYGEN_EXECUTABLE} -u ${DOXYGEN_DOXYFILE}
                     COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_DOXYFILE}
                     COMMAND ${CMAKE_COMMAND} -E echo "Done."
                     WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/doc
                     DEPENDS ${DOXYGEN_DOXYFILE}
  )
  add_custom_target(docs DEPENDS ${DOXYGEN_HTML_INDEX})
  add_custom_target(docs_forced
                    COMMAND ${CMAKE_COMMAND} -E echo_append "Building API Documentation..."
                    COMMAND ${DOXYGEN_EXECUTABLE} -u ${DOXYGEN_DOXYFILE}
                    COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_DOXYFILE}
                    COMMAND ${CMAKE_COMMAND} -E echo "Done."
                    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/doc
  )

endif()
