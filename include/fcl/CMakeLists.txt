# -*- mode: cmake -*-
# vi: set ft=cmake :

# Copyright (c) 2012, Willow Garage, Inc.
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

set(GENERATED_FILE_MARKER "GENERATED FILE DO NOT EDIT")

configure_file(config.h.in config.h @ONLY)

generate_export_header(${PROJECT_NAME}
  EXPORT_FILE_NAME export.h
)

get_filename_component(PARENT_BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}"
  DIRECTORY
)

file(RELATIVE_PATH FCL_CONFIG_H_RELATIVE_PATH
  "${PARENT_BINARY_DIR}" "${CMAKE_CURRENT_BINARY_DIR}/config.h"
)

file(RELATIVE_PATH FCL_EXPORT_H_RELATIVE_PATH
  "${PARENT_BINARY_DIR}" "${CMAKE_CURRENT_BINARY_DIR}/export.h"
)

set(BINARY_DIR_HEADERS
  ${FCL_CONFIG_H_RELATIVE_PATH} ${FCL_EXPORT_H_RELATIVE_PATH}
)

if(CMAKE_VERSION VERSION_LESS 3.12)
  set(CONFIGURE_DEPENDS_OPTION)
else()
  set(CONFIGURE_DEPENDS_OPTION CONFIGURE_DEPENDS)
endif()

get_filename_component(PARENT_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}"
  DIRECTORY
)

file(GLOB_RECURSE SOURCE_DIR_HEADERS RELATIVE "${PARENT_SOURCE_DIR}"
  ${CONFIGURE_DEPENDS_OPTION} "${CMAKE_CURRENT_SOURCE_DIR}/*.h"
)

set(HEADERS ${BINARY_DIR_HEADERS} ${SOURCE_DIR_HEADERS})
set(INCLUDE_DIRECTIVES)

foreach(HEADER ${HEADERS})
  if(NOT HEADER MATCHES "(/detail/|-inl.h$)")
    set(INCLUDE_DIRECTIVES
      "${INCLUDE_DIRECTIVES}\n#include \"${HEADER}\""
    )
  endif()
endforeach()

string(STRIP "${INCLUDE_DIRECTIVES}" INCLUDE_DIRECTIVES)
configure_file(fcl.h.in fcl.h @ONLY)

install(DIRECTORY "${PARENT_BINARY_DIR}/" "${PARENT_SOURCE_DIR}/"
  DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
  COMPONENT Development
  FILES_MATCHING PATTERN "*.h"
  PATTERN "CMakeFiles" EXCLUDE
)

file(GLOB_RECURSE FCL_HEADERS ${CONFIGURE_DEPENDS_OPTION}
  "${CMAKE_CURRENT_BINARY_DIR}/*.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/*.h"
)
set(FCL_HEADERS ${FCL_HEADERS} PARENT_SCOPE)
