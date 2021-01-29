set(tests
    test_collision_func_matrix.cpp
    )

# Build all the tests
foreach(test ${tests})
    add_fcl_test(${test})
endforeach(test)

add_subdirectory(convexity_based_algorithm)
add_subdirectory(primitive_shape_algorithm)
