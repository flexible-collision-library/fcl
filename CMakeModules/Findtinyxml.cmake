# this module was taken from http://trac.evemu.org/browser/trunk/cmake/FindTinyXML.cmake
# - Find TinyXML
# Find the native TinyXML includes and library
#
# TINYXML_FOUND - True if TinyXML found.
# TINYXML_INCLUDE_DIR - where to find tinyxml.h, etc.
# TINYXML_LIBRARIES - List of libraries when using TinyXML.
#
INCLUDE( "FindPackageHandleStandardArgs" )


FIND_PATH( TINYXML_INCLUDE_DIRS "tinyxml.h"
PATH_SUFFIXES "tinyxml" )

FIND_LIBRARY( TINYXML_LIBRARY_DIRS
NAMES "tinyxml"
PATH_SUFFIXES "tinyxml" )

# handle the QUIETLY and REQUIRED arguments and set TINYXML_FOUND to TRUE if
# all listed variables are TRUE
FIND_PACKAGE_HANDLE_STANDARD_ARGS( "TinyXML" DEFAULT_MSG TINYXML_INCLUDE_DIRS TINYXML_LIBRARY_DIRS )
