if(APPLE)
  cmake_minimum_required(VERSION 3.0.00)
elseif(MSVC)
  cmake_minimum_required(VERSION 3.1.3)
else()
  cmake_minimum_required(VERSION 2.8.12)
endif()

# Set the CMAKE_CXX_COMPILER_ID variable to AppleClang instead of Clang.
# AppleClang and Clang have different version number. This was introduced in
# CMake 3.0.
if(POLICY CMP0025)
  cmake_policy(SET CMP0025 NEW)
endif()

# Use MACOSX_RPATH by default on OS X. This was added in CMake 2.8.12 and
# became default in CMake 3.0. Explicitly setting this policy is necessary to
# suppress a warning in CMake 3.0 and above.
if(POLICY CMP0042)
  cmake_policy(SET CMP0042 NEW)
endif()

project(fcl CXX C)

option(FCL_ENABLE_PROFILING "Enable profiling" OFF)
option(FCL_TREAT_WARNINGS_AS_ERRORS "Treat warnings as errors" OFF)

# set the default build type
if (NOT MSVC AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# This shouldn't be necessary, but there has been trouble
# with MSVC being set off, but MSVCXX ON.
if(MSVC OR MSVC90 OR MSVC10)
    set(MSVC ON)
endif (MSVC OR MSVC90 OR MSVC10)

set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules")
include(FCLMacros)
include(CompilerSettings)
include(FCLVersion)
include(GNUInstallDirs)

if(MSVC OR IS_ICPC)
  option(FCL_STATIC_LIBRARY "Whether the FCL library should be static rather than shared" ON)
else()
  option(FCL_STATIC_LIBRARY "Whether the FCL library should be static rather than shared" OFF)
endif()

# Whether to enable SSE
set (SSE_FLAGS "")
option(FCL_USE_X64_SSE "Whether FCL should x64 SSE instructions" ON)
if (FCL_USE_X64_SSE)
  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    set(SSE_FLAGS -mfpmath=sse -msse -msse2 -msse3 -mssse3)
  elseif(MSVC)
    # Win64 will add the flag automatically
    if("$CMAKE_VS_PLATFORM_NAME}" STREQUAL "Win32")
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

# Find Eigen3
find_package(Eigen3 3.0.5 QUIET)
if(EIGEN3_FOUND)
  set(FCL_HAVE_EIGEN TRUE)
  include_directories(${EIGEN3_INCLUDE_DIR})
else()
  message(SEND_ERROR "EIGEN3 (>= 3.0.5) is required by FCL")
  set(FCL_HAVE_EIGEN FALSE)
endif()

# Find libccd
find_package(ccd QUIET)

if(NOT CCD_FOUND AND PKG_CONFIG_FOUND)
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

# Find Octomap (optional)
option(FCL_WITH_OCTOMAP "octomap library support" ON)
set(FCL_HAVE_OCTOMAP 0)
if(FCL_WITH_OCTOMAP)
  find_package(octomap QUIET)
  # octomap-config.cmake may not define OCTOMAP_VERSION so fall back to
  # pkgconfig
  if(NOT OCTOMAP_VERSION AND PKG_CONFIG_FOUND)
      pkg_check_modules(OCTOMAP QUIET octomap)
  endif()
  # whether octomap_FOUND and/or OCTOMAP_FOUND are set is inconsistent, but
  # OCTOMAP_INCLUDE_DIRS and OCTOMAP_LIBRARY_DIRS should always be set when
  # octomap is found
  if(NOT OCTOMAP_INCLUDE_DIRS AND NOT OCTOMAP_LIBRARY_DIRS)
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
  if(OCTOMAP_INCLUDE_DIRS AND OCTOMAP_LIBRARY_DIRS AND OCTOMAP_VERSION)
    if(NOT OCTOMAP_MAJOR_VERSION AND NOT OCTOMAP_MINOR_VERSION AND NOT OCTOMAP_PATCH_VERSION)
      string(REPLACE "." ";" VERSION_LIST "${OCTOMAP_VERSION}")
      list(GET VERSION_LIST 0 OCTOMAP_MAJOR_VERSION)
      list(GET VERSION_LIST 1 OCTOMAP_MINOR_VERSION)
      list(GET VERSION_LIST 2 OCTOMAP_PATCH_VERSION)
    endif()
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


# FCL's own include dir should be at the front of the include path
include_directories(BEFORE "include")
include_directories(BEFORE "${CMAKE_CURRENT_BINARY_DIR}/include")

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

option(FCL_BUILD_TESTS "Build FCL tests" ON)
if(FCL_BUILD_TESTS)
    enable_testing()
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
