set(tests
    test_sphere_box.cpp
)

# Build all the tests
foreach(test ${tests})
    add_fcl_test(${test})
endforeach(test)
