set(tests
        test_box_box-inl.cpp
        )

# Build all the tests
foreach(test ${tests})
    add_fcl_test(${test})
endforeach(test)
