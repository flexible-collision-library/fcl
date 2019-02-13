set(tests
        test_broadphase_dynamic_AABB_tree.cpp
        )

# Build all the tests
foreach(test ${tests})
    add_fcl_test(${test})
endforeach(test)
