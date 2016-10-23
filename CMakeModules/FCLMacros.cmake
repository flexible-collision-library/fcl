#===============================================================================
# Appends items to a cached list.
# Usage:
#   fcl_append_to_cached_string(_string _cacheDesc [items...])
#===============================================================================
macro(fcl_append_to_cached_string _string _cacheDesc)
  foreach(newItem ${ARGN})
    set(${_string} "${${_string}}${newItem}" CACHE INTERNAL ${_cacheDesc} FORCE)
  endforeach()
endmacro()

#===============================================================================
# Get list of file names give list of full paths.
# Usage:
#   fcl_get_filename_components(_var _cacheDesc [items...])
#===============================================================================
macro(fcl_get_filename_components _var _cacheDesc _prefix_to_remove)
  set(${_var} "" CACHE INTERNAL ${_cacheDesc} FORCE)
  string(LENGTH ${_prefix_to_remove} prefix_length)
  foreach(header ${ARGN})
    string(LENGTH ${header} full_length)
    math(EXPR relative_path_length "${full_length} - ${prefix_length}")
    string(SUBSTRING ${header} ${prefix_length} ${relative_path_length} header)
    if(NOT ${header} MATCHES "/detail/" AND NOT ${header} MATCHES "-inl.h")
      fcl_append_to_cached_string(
        ${_var}
        ${_cacheDesc}"_HEADER_NAMES"
        "${header}\;")
    endif()
  endforeach()
endmacro()

#===============================================================================
# Generate header file list to a cached list.
# Usage:
#   fcl_generate_include_header_list(_var _target_dir _cacheDesc [headers...])
#===============================================================================
macro(fcl_generate_include_header_list _var _target_dir _cacheDesc)
  set(${_var} "" CACHE INTERNAL ${_cacheDesc} FORCE)
  foreach(header ${ARGN})
    fcl_append_to_cached_string(
      ${_var}
      ${_cacheDesc}"_HEADERS"
      "#include \"${_target_dir}${header}\"\n")
  endforeach()
endmacro()
