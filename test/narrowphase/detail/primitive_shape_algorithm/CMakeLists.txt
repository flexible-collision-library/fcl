set(tests
    test_sphere_box.cpp
    test_sphere_cylinder.cpp
    test_half_space_convex.cpp
)

# Build all the tests
foreach(test ${tests})
    add_fcl_test(${test})
endforeach(test)
