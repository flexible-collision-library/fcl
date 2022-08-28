option(FCL_BUILD_EXAMPLES "Build FCL examples" ON)

if (FCL_BUILD_EXAMPLES)

    macro(add_fcl_demo demo_name)
        add_executable(${ARGV})
        target_link_libraries(${demo_name}
            fcl)
    endmacro(add_fcl_demo)

    add_fcl_demo(demo_libccdindp demo_libccdindp.cpp test_fcl_utility.h test_fcl_utility.cpp)

    #add_fcl_demo(demo_boxspheredistance boxspheredistance.cpp test_fcl_utility.h test_fcl_utility.cpp)
    #add_fcl_demo(demo_boxellipsoiddistance boxellipsoiddistance.cpp test_fcl_utility.h test_fcl_utility.cpp)
    #add_fcl_demo(demo_ellipsoidspheredistance ellipsoidspheredistance.cpp test_fcl_utility.h test_fcl_utility.cpp)
    #add_fcl_demo(demo_sphereellipsoiddistance sphereellipsoiddistance.cpp test_fcl_utility.h test_fcl_utility.cpp)
    #add_fcl_demo(demo_spherespheredistance spherespheredistance.cpp test_fcl_utility.h test_fcl_utility.cpp)
    #add_fcl_demo(demo_boxbox boxboxdistance.cpp test_fcl_utility.h test_fcl_utility.cpp)

endif(FCL_BUILD_EXAMPLES)
