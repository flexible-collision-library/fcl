include_directories(${Boost_INCLUDE_DIR})

if(MSVC)
    add_definitions(-DBOOST_ALL_NO_LIB)
endif()
add_definitions(-DBOOST_TEST_DYN_LINK)


macro(add_fcl_test test_name)
  add_executable(${ARGV})
  target_link_libraries(${test_name}
    fcl
    ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_SYSTEM_LIBRARY}
    ${Boost_THREAD_LIBRARY}
    ${Boost_DATE_TIME_LIBRARY}
    ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
  add_test(${test_name} ${EXECUTABLE_OUTPUT_PATH}/${test_name})
endmacro(add_fcl_test)

# configure location of resources
file(TO_NATIVE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/fcl_resources" TEST_RESOURCES_SRC_DIR)
file(TO_NATIVE_PATH "${CMAKE_CURRENT_BINARY_DIR}/fcl_resources" TEST_RESOURCES_BIN_DIR)
if(WIN32)
    # Correct directory separator for Windows
    string(REPLACE "\\" "\\\\" TEST_RESOURCES_SRC_DIR ${TEST_RESOURCES_SRC_DIR})
    string(REPLACE "\\" "\\\\" TEST_RESOURCES_BIN_DIR ${TEST_RESOURCES_BIN_DIR})
endif(WIN32)
configure_file("${TEST_RESOURCES_SRC_DIR}/config.h.in" "${TEST_RESOURCES_BIN_DIR}/config.h")

include_directories(.)
include_directories("${CMAKE_CURRENT_BINARY_DIR}")

add_fcl_test(test_fcl_collision test_fcl_collision.cpp test_fcl_utility.cpp)
add_fcl_test(test_fcl_distance test_fcl_distance.cpp test_fcl_utility.cpp)
add_fcl_test(test_fcl_geometric_shapes test_fcl_geometric_shapes.cpp test_fcl_utility.cpp)
add_fcl_test(test_fcl_broadphase test_fcl_broadphase.cpp test_fcl_utility.cpp)
add_fcl_test(test_fcl_shape_mesh_consistency test_fcl_shape_mesh_consistency.cpp test_fcl_utility.cpp)
add_fcl_test(test_fcl_frontlist test_fcl_frontlist.cpp test_fcl_utility.cpp)
add_fcl_test(test_fcl_math test_fcl_math.cpp test_fcl_utility.cpp)

add_fcl_test(test_fcl_sphere_capsule test_fcl_sphere_capsule.cpp)
add_fcl_test(test_fcl_capsule_capsule test_fcl_capsule_capsule.cpp)
add_fcl_test(test_fcl_simple test_fcl_simple.cpp)
add_fcl_test(test_fcl_capsule_box_1 test_fcl_capsule_box_1.cpp)
add_fcl_test(test_fcl_capsule_box_2 test_fcl_capsule_box_2.cpp)
#add_fcl_test(test_fcl_global_penetration test_fcl_global_penetration.cpp libsvm/svm.cpp test_fcl_utility.cpp)
add_fcl_test(test_fcl_bvh_models test_fcl_bvh_models.cpp test_fcl_utility.cpp)

if (FCL_HAVE_OCTOMAP)
  add_fcl_test(test_fcl_octomap test_fcl_octomap.cpp test_fcl_utility.cpp)
endif()

#if (FCL_HAVE_TINYXML)
#  add_executable(test_fcl_xmldata test_fcl_xmldata.cpp test_fcl_utility.cpp libsvm/svm.cpp)
#  target_link_libraries(test_fcl_xmldata
#    fcl
#    ${TINYXML_LIBRARY_DIRS}
#    ${Boost_SYSTEM_LIBRARY}
#    ${Boost_THREAD_LIBRARY}
#    ${Boost_DATE_TIME_LIBRARY}
#    ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
#  add_test(test_fcl_xmldata ${EXECUTABLE_OUTPUT_PATH}/test_fcl_xmldata)
#endif()
