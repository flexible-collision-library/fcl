# -*- mode: cmake -*-
# vi: set ft=cmake :

# Copyright (c) 2013, Willow Garage, Inc.
# Copyright (c) 2017, Toyota Research Institute, Inc.
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

# GCC
if(CMAKE_COMPILER_IS_GNUCXX)
    add_definitions(-std=c++11 -W -Wall -Wextra -Wpedantic)
    if(FCL_TREAT_WARNINGS_AS_ERRORS)
        add_definitions(-Werror)
    endif()
endif()

# Clang
if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    add_definitions(-std=c++11 -W -Wall -Wextra)
    if(FCL_TREAT_WARNINGS_AS_ERRORS)
        add_definitions(-Werror)
    endif()
endif()

# AppleClang
if(CMAKE_CXX_COMPILER_ID STREQUAL "AppleClang")
    # Require at least Apple LLVM version 6.1
    if (CMAKE_CXX_COMPILER_VERSION VERSION_LESS 6.1)
        message(FATAL_ERROR "AppleClang version must be at least 6.1!")
    endif()
    add_definitions(-std=c++11 -W -Wall -Wextra)
    if(FCL_TREAT_WARNINGS_AS_ERRORS)
        add_definitions(-Werror)
    endif()
endif()

# Visual Studio
if(MSVC)
    if(MSVC_VERSION VERSION_LESS 1900)
        message(FATAL_ERROR "${PROJECT_NAME} requires Visual Studio 2015 or greater.")
    endif()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /EHsc /MP /W1 /bigobj")
    if(FCL_TREAT_WARNINGS_AS_ERRORS)
        add_definitions(/WX)
    endif()
endif()

# Intel
if(CMAKE_CXX_COMPILER_ID STREQUAL "Intel")
    set(IS_ICPC 1)
else()
    set(IS_ICPC 0)
endif()
if(IS_ICPC)
    add_definitions(-std=c++11 -wd191 -wd411 -wd654 -wd1125 -wd1292 -wd1565 -wd1628 -wd2196)
    set(CMAKE_AR "xiar" CACHE STRING "Intel archiver" FORCE)
    set(CMAKE_CXX_FLAGS "-pthread" CACHE STRING "Default compile flags" FORCE)
    set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG"
    CACHE STRING "Flags used by the C++ compiler during release builds." FORCE)
    set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g" CACHE STRING
    "Flags used by the C++ compiler during debug builds." FORCE)
    set(CMAKE_LINKER "xild" CACHE STRING "Intel linker" FORCE)
    if(FCL_TREAT_WARNINGS_AS_ERRORS)
        add_definitions(-Werror)
    endif()
endif()

# XL
if(CMAKE_CXX_COMPILER_ID STREQUAL "XL")
    set(IS_XLC 1)
else()
    set(IS_XLC 0)
endif()
if(IS_XLC)
    add_definitions(-std=c++11 -qpic -q64 -qmaxmem=-1)
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -q64")
    set(CMAKE_MODULE_LINKER_FLAGS "${CMAKE_MODULE_LINKER_FLAGS} -q64")
endif()

# MinGW
if((CMAKE_COMPILER_IS_GNUCXX OR IS_ICPC) AND NOT MINGW)
    add_definitions(-fPIC)
    if(FCL_TREAT_WARNINGS_AS_ERRORS)
        add_definitions(-Werror)
    endif()
endif()

# By default CMake sets RPATH for binaries in the build tree, but clears
# it when installing. Switch this option off if the default behaviour is
# desired.
option(FCL_NO_DEFAULT_RPATH "Set RPATH for installed binaries" ON)
mark_as_advanced(FCL_NO_DEFAULT_RPATH)

# Set rpath http://www.paraview.org/Wiki/CMake_RPATH_handling
if(FCL_NO_DEFAULT_RPATH)
    message(STATUS "Set RPATH explicitly to '${CMAKE_INSTALL_FULL_LIBDIR}'")
    set(CMAKE_SKIP_BUILD_RPATH OFF)
    set(CMAKE_BUILD_WITH_INSTALL_RPATH OFF)
    set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_FULL_LIBDIR}")
    set(CMAKE_INSTALL_RPATH_USE_LINK_PATH ON)
endif()
