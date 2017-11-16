file(GLOB_RECURSE HEADERS            ${CMAKE_CURRENT_SOURCE_DIR}/*.h)
file(GLOB_RECURSE CONFIGURED_HEADERS ${CMAKE_CURRENT_BINARY_DIR}/*.h)
set(FCL_HEADERS ${HEADERS} ${CONFIGURED_HEADERS} PARENT_SCOPE)

# Generate export header. There is no way of generating a file name
# called just export.h. Workaround using configure and remove
generate_export_header(${PROJECT_NAME})
configure_file(${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_export.h
  ${CMAKE_CURRENT_BINARY_DIR}/export.h
  COPYONLY)
file(REMOVE ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_export.h)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/export.h
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/fcl)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config.h.in ${CMAKE_CURRENT_BINARY_DIR}/config.h)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/config.h DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/fcl)

fcl_get_filename_components(
  fcl_header_rel_paths
  "fcl headers"
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${HEADERS})
fcl_get_filename_components(
  fcl_config_header_rel_paths
  "fcl configured headers"
  ${CMAKE_CURRENT_BINARY_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}/config.h)
list(INSERT fcl_header_rel_paths 0 ${fcl_config_header_rel_paths})

fcl_generate_include_header_list(
  fcl_headers
  "fcl"
  "fcl header relative paths"
  ${fcl_header_rel_paths})
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/fcl.h.in
  ${CMAKE_CURRENT_BINARY_DIR}/fcl.h)

install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/fcl.h
  DESTINATION include/fcl
  COMPONENT headers)

list(APPEND FCL_HEADERS ${CMAKE_CURRENT_BINARY_DIR}/fcl.h)
