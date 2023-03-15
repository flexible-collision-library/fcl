set(tests
        test_interp_motion.cpp
        )

# Build all the tests
foreach(test ${tests})
    add_fcl_test(${test})
endforeach(test)
