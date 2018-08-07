set(tests
    test_sphere_box.cpp
    test_sphere_cylinder.cpp
)

# Build all the tests
foreach(test ${tests})
    add_fcl_test(${test})
endforeach(test)
