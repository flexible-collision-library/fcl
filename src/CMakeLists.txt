file(GLOB_RECURSE FCL_SOURCE_CODE ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)
if(FCL_STATIC_LIBRARY)
  add_library(${PROJECT_NAME} STATIC ${FCL_HEADERS} ${FCL_SOURCE_CODE})
else()
  add_library(${PROJECT_NAME} SHARED ${FCL_HEADERS} ${FCL_SOURCE_CODE})
endif()

# Be sure to pass to the consumer the set of SIMD used in the compilation
target_compile_options(${PROJECT_NAME} PUBLIC ${SSE_FLAGS})

set_target_properties(${PROJECT_NAME} PROPERTIES
  VERSION ${FCL_VERSION}
  SOVERSION ${FCL_ABI_VERSION})

# Hide symbols by default
#################################################
# Macro to check for visibility capability in compiler
# Original idea from: https://gitorious.org/ferric-cmake-stuff/
macro (check_compiler_visibility)
  include (CheckCXXCompilerFlag)
  check_cxx_compiler_flag(-fvisibility=hidden COMPILER_SUPPORTS_VISIBILITY)
endmacro()

if (UNIX)
  check_compiler_visibility()
  if (COMPILER_SUPPORTS_VISIBILITY)
    set_target_properties(${PROJECT_NAME}
      PROPERTIES COMPILE_FLAGS "-fvisibility=hidden")
  endif()
endif()

if (FCL_HIDE_ALL_SYMBOLS)
 add_definitions("-DFCL_STATIC_DEFINE")
endif()

# Use the IMPORTED target from newer versions of ccd-config.cmake if available,
# otherwise fall back to CCD_INCLUDE_DIRS and CCD_LIBRARIES
if(TARGET ccd)
  target_link_libraries(${PROJECT_NAME} PUBLIC ccd)
else()
  target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC "${CCD_INCLUDE_DIRS}")
  target_link_libraries(${PROJECT_NAME} PUBLIC "${CCD_LIBRARIES}")
endif()

# Use the IMPORTED target from newer versions of Eigen3Config.cmake if
# available, otherwise fall back to EIGEN3_INCLUDE_DIRS from older versions of
# Eigen3Config.cmake or EIGEN3_INCLUDE_DIR from FindEigen3.cmake
if(TARGET Eigen3::Eigen)
  # Note that Eigen3::Eigen is an INTERFACE library, so the INCLUDE_DIRECTORIES
  # and INTERFACE_INCLUDE_DIRECTORIES are populated, but nothing is actually
  # linked
  target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen)
elseif(EIGEN3_INCLUDE_DIRS)
  target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC "${EIGEN3_INCLUDE_DIRS}")
else()
  target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC "${EIGEN3_INCLUDE_DIR}")
endif()

if(FCL_HAVE_OCTOMAP)
  # Use the IMPORTED target from newer versions of octomap-config.cmake if
  # available, otherwise fall back to OCTOMAP_INCLUDE_DIRS and OCTOMAP_LIBRARIES
  if(TARGET octomap)
    target_link_libraries(${PROJECT_NAME} PUBLIC octomap)
  elseif(OCTOMAP_INCLUDE_DIRS AND OCTOMAP_LIBRARIES)
    target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC "${OCTOMAP_INCLUDE_DIRS}")
    target_link_libraries(${PROJECT_NAME} PUBLIC "${OCTOMAP_LIBRARIES}")
  endif()
endif()

target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}/include>
  $<INSTALL_INTERFACE:include>)

export(TARGETS ${PROJECT_NAME}
  FILE "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake")

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Config
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

if(WIN32 AND NOT CYGWIN)
  install(EXPORT ${PROJECT_NAME}Config DESTINATION CMake)
else()
  install(EXPORT ${PROJECT_NAME}Config
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME})
endif()

# Setup the coveralls target and tell it to gather coverage data for all the lib sources.
if(FCL_COVERALLS)
  coveralls_setup("${FCL_HEADERS};${FCL_SOURCE_CODE};" ${FCL_COVERALLS_UPLOAD})
endif()
