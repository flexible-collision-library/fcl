# set the version in a way CMake can use
file(READ package.xml PACKAGE_XML)
string(REGEX MATCH "<version>[0-9]+\\.[0-9]+\\.[0-9]+</version>" DIRTY_VERSION_STRING ${PACKAGE_XML})

string(REGEX REPLACE "^<version>([0-9]+)\\.[0-9]+\\.[0-9]+</version>" "\\1" FCL_MAJOR_VERSION "${DIRTY_VERSION_STRING}")
string(REGEX REPLACE "^<version>[0-9]+\\.([0-9])+\\.[0-9]+</version>" "\\1" FCL_MINOR_VERSION "${DIRTY_VERSION_STRING}")
string(REGEX REPLACE "^<version>[0-9]+\\.[0-9]+\\.([0-9]+)</version>" "\\1" FCL_PATCH_VERSION "${DIRTY_VERSION_STRING}")

set(FCL_VERSION "${FCL_MAJOR_VERSION}.${FCL_MINOR_VERSION}.${FCL_PATCH_VERSION}")

# increment this when we have ABI changes
set(FCL_ABI_VERSION ${FCL_MAJOR_VERSION}.${FCL_MINOR_VERSION})
