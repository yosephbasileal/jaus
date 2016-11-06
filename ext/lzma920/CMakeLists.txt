# CMake requires we pecify a minimum version, current min is 2.8
if(WIN32)
	# Require 2.8.2 for configuring Debug/Release configuration options in 
	# Visual Studio
	cmake_minimum_required (VERSION 2.8.2) 
else()
	cmake_minimum_required (VERSION 2.8)
endif(WIN32)

# Use path relative to cmake directory.
if(COMMAND cmake_policy)
	if(POLICY CMP0015)
		cmake_policy(SET CMP0015 NEW)
	endif(POLICY CMP0015)
endif(COMMAND cmake_policy)

# Get library macros
include(${ACTIVE_CMAKE_DIRECTORY}/CreateLibrary.cmake)

# Set Solution/Workspace name, this will be re-used
set(PROJECT_NAME lzma920)

set(PROJECT_SRC_FOLDER ${ACTIVE_EXT_DIRECTORY}/${PROJECT_NAME}/C)
if(MSVC)
	set (PROJECT_SOURCE_FILES 
		${PROJECT_SRC_FOLDER}/LzmaDec.h
		${PROJECT_SRC_FOLDER}/LzmaDec.c
		${PROJECT_SRC_FOLDER}/Types.h
		${PROJECT_SRC_FOLDER}/LzmaEnc.h 
		${PROJECT_SRC_FOLDER}/LzmaEnc.c
		${PROJECT_SRC_FOLDER}/LzFind.c
		${PROJECT_SRC_FOLDER}/LzFind.h
		${PROJECT_SRC_FOLDER}/LzFindMt.c 
		${PROJECT_SRC_FOLDER}/LzFindMt.h
		${PROJECT_SRC_FOLDER}/LzHash.h
		${PROJECT_SRC_FOLDER}/Alloc.h
		${PROJECT_SRC_FOLDER}/Alloc.c
		${PROJECT_SRC_FOLDER}/LzmaLib.h
		${PROJECT_SRC_FOLDER}/LzmaLib.c
		${PROJECT_SRC_FOLDER}/Threads.h
		${PROJECT_SRC_FOLDER}/Threads.c
	)
	add_definitions(-wd4996)
else()
	add_definitions(-D_7ZIP_ST)
	set (PROJECT_SOURCE_FILES 
		${PROJECT_SRC_FOLDER}/LzmaDec.c
		${PROJECT_SRC_FOLDER}/LzmaEnc.c
		${PROJECT_SRC_FOLDER}/LzFind.c
		${PROJECT_SRC_FOLDER}/LzmaLib.c
		${PROJECT_SRC_FOLDER}/CpuArch.c
		${PROJECT_SRC_FOLDER}/7zCrc.c
		${PROJECT_SRC_FOLDER}/7zCrcOpt.c
		${PROJECT_SRC_FOLDER}/Bra86.c
	)
endif()

configure_ext_library(${PROJECT_NAME}
						PROJECT_SOURCE_FILES
						""
						STATIC
						LIBRARY_NAME)
set(${PROJECT_NAME}_INCLUDE_DIRS	
		${${PROJECT_NAME}_INCLUDE_DIRS}
		${PROJECT_SRC_FOLDER} PARENT_SCOPE)
