set(tests
    test_gjk_libccd-inl_epa.cpp
    test_gjk_libccd-inl_extractClosestPoints.cpp
    test_gjk_libccd-inl_gjk_doSimplex2.cpp
)

# Build all the tests
foreach(test ${tests})
  add_fcl_test(${test})
endforeach(test)

if(CCD_VERSION VERSION_EQUAL 2.0 OR CCD_VERSION VERSION_GREATER 2.0)
    add_fcl_test(test_gjk_libccd-inl_signed_distance.cpp)
endif()
