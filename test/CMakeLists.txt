#===============================================================================
# Google Test settings
#===============================================================================

include_directories (BEFORE
  ${CMAKE_CURRENT_SOURCE_DIR}/gtest/include
  ${CMAKE_CURRENT_SOURCE_DIR}/gtest
  ${CMAKE_CURRENT_SOURCE_DIR}
)

link_directories(${PROJECT_BINARY_DIR}/test)

# Build gtest
add_library(gtest STATIC gtest/src/gtest-all.cc)
add_library(gtest_main STATIC gtest/src/gtest_main.cc)
target_link_libraries(gtest_main gtest)
if(NOT WIN32)
  target_link_libraries(gtest pthread)
endif()

add_definitions(-DGTEST_DONT_DEFINE_TEST=1)
add_definitions(-DGTEST_DONT_DEFINE_FAIL=1)
add_definitions(-DGTEST_DONT_DEFINE_SUCCEED=1)
add_definitions(-DGTEST_DONT_DEFINE_ASSERT_EQ=1)
add_definitions(-DGTEST_DONT_DEFINE_ASSERT_NE=1)
add_definitions(-DGTEST_DONT_DEFINE_ASSERT_LE=1)
add_definitions(-DGTEST_DONT_DEFINE_ASSERT_LT=1)
add_definitions(-DGTEST_DONT_DEFINE_ASSERT_GE=1)
add_definitions(-DGTEST_DONT_DEFINE_ASSERT_GT=1)

execute_process(COMMAND cmake -E remove_directory ${CMAKE_BINARY_DIR}/test_results)
execute_process(COMMAND cmake -E make_directory ${CMAKE_BINARY_DIR}/test_results)
include_directories(${GTEST_INCLUDE_DIRS})

add_library(test_fcl_utility test_fcl_utility.cpp)
target_link_libraries(test_fcl_utility fcl)

# test file list
set(tests
    test_fcl_auto_diff.cpp
    test_fcl_box_box.cpp
    test_fcl_broadphase_collision_1.cpp
    test_fcl_broadphase_collision_2.cpp
    test_fcl_broadphase_distance.cpp
    test_fcl_bvh_models.cpp
    test_fcl_capsule_box_1.cpp
    test_fcl_capsule_box_2.cpp
    test_fcl_capsule_capsule.cpp
    test_fcl_cylinder_half_space.cpp
    test_fcl_collision.cpp
    test_fcl_constant_eps.cpp
    test_fcl_distance.cpp
    test_fcl_frontlist.cpp
    test_fcl_general.cpp
    test_fcl_generate_bvh_model_deferred_finalize.cpp
    test_fcl_geometric_shapes.cpp
    test_fcl_math.cpp
    test_fcl_profiler.cpp
    test_fcl_shape_mesh_consistency.cpp
    test_fcl_signed_distance.cpp
    test_fcl_simple.cpp
    test_fcl_sphere_box.cpp
    test_fcl_sphere_capsule.cpp
    test_fcl_sphere_cylinder.cpp
    test_fcl_sphere_sphere.cpp
)

if (FCL_HAVE_OCTOMAP)
  list(APPEND tests test_fcl_octomap_cost.cpp)
  list(APPEND tests test_fcl_octomap_collision.cpp)
  list(APPEND tests test_fcl_octomap_distance.cpp)
endif()

macro(add_fcl_test test_file_name)
  # Get the name (i.e. bla.cpp => bla)
  get_filename_component(test_name ${ARGV} NAME_WE)
  add_executable(${test_name} ${ARGV})
  target_link_libraries(${test_name} fcl test_fcl_utility gtest)
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

# Build all the tests
foreach(test ${tests})
  add_fcl_test(${test})
endforeach(test)

add_subdirectory(geometry)
add_subdirectory(narrowphase)
add_subdirectory(broadphase)
