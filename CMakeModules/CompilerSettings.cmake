# GCC
if(CMAKE_COMPILER_IS_GNUCXX)
    add_definitions(-std=c++11 -W -Wall -Wextra -Wpedantic)
    if(FCL_TREAT_WARNINGS_AS_ERRORS)
        add_definitions(-Werror)
    endif()
endif()

# Clang
if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    add_definitions(-std=c++11 -W -Wall -Wextra)
    if(FCL_TREAT_WARNINGS_AS_ERRORS)
        add_definitions(-Werror)
    endif()
endif()

# AppleClang
if(CMAKE_CXX_COMPILER_ID STREQUAL "AppleClang")
    # Require at least Apple LLVM version 6.1
    if (CMAKE_CXX_COMPILER_VERSION VERSION_LESS 6.1)
        message(FATAL_ERROR "AppleClang version must be at least 6.1!")
    endif()
    add_definitions(-std=c++11 -W -Wall -Wextra)
    if(FCL_TREAT_WARNINGS_AS_ERRORS)
        add_definitions(-Werror)
    endif()
endif()

# Visual Studio
if(MSVC)
    if(MSVC_VERSION VERSION_LESS 1900)
        message(FATAL_ERROR "${PROJECT_NAME} requires Visual Studio 2015 or greater.")
    endif()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /EHsc /MP /W1 /bigobj")
    if(FCL_TREAT_WARNINGS_AS_ERRORS)
        add_definitions(/WX)
    endif()
endif()

# Intel
if(CMAKE_CXX_COMPILER_ID STREQUAL "Intel")
    set(IS_ICPC 1)
else()
    set(IS_ICPC 0)
endif()
if(IS_ICPC)
    add_definitions(-std=c++11 -wd191 -wd411 -wd654 -wd1125 -wd1292 -wd1565 -wd1628 -wd2196)
    set(CMAKE_AR "xiar" CACHE STRING "Intel archiver" FORCE)
    set(CMAKE_CXX_FLAGS "-pthread" CACHE STRING "Default compile flags" FORCE)
    set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG"
    CACHE STRING "Flags used by the C++ compiler during release builds." FORCE)
    set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g" CACHE STRING
    "Flags used by the C++ compiler during debug builds." FORCE)
    set(CMAKE_LINKER "xild" CACHE STRING "Intel linker" FORCE)
    if(FCL_TREAT_WARNINGS_AS_ERRORS)
        add_definitions(-Werror)
    endif()
endif()

# XL
if(CMAKE_CXX_COMPILER_ID STREQUAL "XL")
    set(IS_XLC 1)
else()
    set(IS_XLC 0)
endif()
if(IS_XLC)
    add_definitions(-std=c++11 -qpic -q64 -qmaxmem=-1)
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -q64")
    set(CMAKE_MODULE_LINKER_FLAGS "${CMAKE_MODULE_LINKER_FLAGS} -q64")
endif()

# MinGW
if((CMAKE_COMPILER_IS_GNUCXX OR IS_ICPC) AND NOT MINGW)
    add_definitions(-fPIC)
    if(FCL_TREAT_WARNINGS_AS_ERRORS)
        add_definitions(-Werror)
    endif()
endif()

# By default CMake sets RPATH for binaries in the build tree, but clears
# it when installing. Switch this option off if the default behaviour is
# desired.
option(FCL_NO_DEFAULT_RPATH "Set RPATH for installed binaries" ON)
mark_as_advanced(FCL_NO_DEFAULT_RPATH)

# Set rpath http://www.paraview.org/Wiki/CMake_RPATH_handling
if(FCL_NO_DEFAULT_RPATH)
    message(STATUS "Set RPATH explicitly to '${CMAKE_INSTALL_LIBDIR_FULL}'")
    set(CMAKE_SKIP_BUILD_RPATH FALSE)
    set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
    set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_LIBDIR_FULL}")
    set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
endif()

# no prefix needed for python modules
set(CMAKE_SHARED_MODULE_PREFIX "")
