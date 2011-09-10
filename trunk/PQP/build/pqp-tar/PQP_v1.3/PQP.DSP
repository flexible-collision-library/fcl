# Microsoft Developer Studio Project File - Name="PQP" - Package Owner=<4>
# Microsoft Developer Studio Generated Build File, Format Version 5.00
# ** DO NOT EDIT **

# TARGTYPE "Win32 (x86) Static Library" 0x0104

CFG=PQP - Win32 Debug
!MESSAGE This is not a valid makefile. To build this project using NMAKE,
!MESSAGE use the Export Makefile command and run
!MESSAGE 
!MESSAGE NMAKE /f "PQP.MAK".
!MESSAGE 
!MESSAGE You can specify a configuration when running NMAKE
!MESSAGE by defining the macro CFG on the command line. For example:
!MESSAGE 
!MESSAGE NMAKE /f "PQP.MAK" CFG="PQP - Win32 Debug"
!MESSAGE 
!MESSAGE Possible choices for configuration are:
!MESSAGE 
!MESSAGE "PQP - Win32 Release" (based on "Win32 (x86) Static Library")
!MESSAGE "PQP - Win32 Debug" (based on "Win32 (x86) Static Library")
!MESSAGE 

# Begin Project
# PROP Scc_ProjName ""
# PROP Scc_LocalPath ""
CPP=cl.exe

!IF  "$(CFG)" == "PQP - Win32 Release"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 0
# PROP BASE Output_Dir "Release"
# PROP BASE Intermediate_Dir "Release"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 0
# PROP Output_Dir "lib"
# PROP Intermediate_Dir "lib"
# PROP Target_Dir ""
# ADD BASE CPP /nologo /W3 /GX /O2 /D "WIN32" /D "NDEBUG" /D "_WINDOWS" /YX /FD /c
# ADD CPP /nologo /W3 /GX /Ot /Ob2 /D "WIN32" /D "NDEBUG" /D "_WINDOWS" /YX /FD /c
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LIB32=link.exe -lib
# ADD BASE LIB32 /nologo
# ADD LIB32 /nologo
# Begin Special Build Tool
SOURCE=$(InputPath)
PostBuild_Cmds=copy   src\PQP.h   include	copy   src\PQP_Internal.h   include\
	copy    src\PQP_Compile.h   include	copy   src\Tri.h   include	copy   src\BV.h\
  include
# End Special Build Tool

!ELSEIF  "$(CFG)" == "PQP - Win32 Debug"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 1
# PROP BASE Output_Dir "Debug"
# PROP BASE Intermediate_Dir "Debug"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 1
# PROP Output_Dir "lib"
# PROP Intermediate_Dir "lib"
# PROP Target_Dir ""
# ADD BASE CPP /nologo /W3 /GX /Z7 /Od /D "WIN32" /D "_DEBUG" /D "_WINDOWS" /YX /FD /c
# ADD CPP /nologo /W3 /GX /Z7 /Od /D "WIN32" /D "_DEBUG" /D "_WINDOWS" /YX /FD /c
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LIB32=link.exe -lib
# ADD BASE LIB32 /nologo
# ADD LIB32 /nologo
# Begin Special Build Tool
SOURCE=$(InputPath)
PostBuild_Cmds=copy   src\PQP.h   include	copy   src\PQP_Internal.h   include\
	copy    src\PQP_Compile.h   include	copy   src\Tri.h   include	copy   src\BV.h\
  include
# End Special Build Tool

!ENDIF 

# Begin Target

# Name "PQP - Win32 Release"
# Name "PQP - Win32 Debug"
# Begin Source File

SOURCE=.\src\Build.cpp
# End Source File
# Begin Source File

SOURCE=.\src\Build.h
# End Source File
# Begin Source File

SOURCE=.\src\BV.cpp
# End Source File
# Begin Source File

SOURCE=.\src\BV.h
# End Source File
# Begin Source File

SOURCE=.\src\BVTQ.h
# End Source File
# Begin Source File

SOURCE=.\src\GetTime.h
# End Source File
# Begin Source File

SOURCE=.\src\MatVec.h
# End Source File
# Begin Source File

SOURCE=.\src\OBB_Disjoint.h
# End Source File
# Begin Source File

SOURCE=.\src\PQP.cpp
# End Source File
# Begin Source File

SOURCE=.\src\PQP.h
# End Source File
# Begin Source File

SOURCE=.\src\PQP_Compile.h
# End Source File
# Begin Source File

SOURCE=.\src\PQP_Internal.h
# End Source File
# Begin Source File

SOURCE=.\src\RectDist.h
# End Source File
# Begin Source File

SOURCE=.\src\Tri.h
# End Source File
# Begin Source File

SOURCE=.\src\TriDist.cpp
# End Source File
# Begin Source File

SOURCE=.\src\TriDist.h
# End Source File
# End Target
# End Project
