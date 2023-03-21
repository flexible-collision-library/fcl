set(tests
        test_box.cpp
        test_capsule.cpp
        test_cone.cpp
        test_convex.cpp
        test_cylinder.cpp
        test_ellipsoid.cpp
        test_halfspace.cpp
        test_plane.cpp
        test_sphere.cpp
        )

# Build all the tests
foreach(test ${tests})
    add_fcl_test(${test})
endforeach(test)
