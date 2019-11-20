set(tests
        test_convex.cpp
        test_capsule.cpp
        )

# Build all the tests
foreach(test ${tests})
    add_fcl_test(${test})
endforeach(test)
