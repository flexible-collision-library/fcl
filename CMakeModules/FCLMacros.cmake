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
macro(fcl_get_filename_components _var _cacheDesc _suffix_to_remove)
  set(${_var} "" CACHE INTERNAL ${_cacheDesc} FORCE)
  string(LENGTH ${_suffix_to_remove} suffix_length)
  foreach(header ${ARGN})
    string(LENGTH ${header} full_length)
    string(
      SUBSTRING
      ${header}
      ${suffix_length}
      ${full_length}-${suffix_length}
      header)
      if(${header} MATCHES "/detail/")
        continue()
      endif()
      if(${header} MATCHES "-impl.h")
        continue()
      endif()
    fcl_append_to_cached_string(
      ${_var}
      ${_cacheDesc}"_HEADER_NAMES"
      "${header}\;")
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
