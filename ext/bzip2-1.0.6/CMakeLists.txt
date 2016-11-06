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
set(PROJECT_NAME bzip2-1.0.6)

set(PROJECT_SRC_FOLDER ${ACTIVE_EXT_DIRECTORY}/${PROJECT_NAME})

set (PROJECT_SOURCE_FILES 
	${PROJECT_SRC_FOLDER}/blocksort.c
	${PROJECT_SRC_FOLDER}/bzlib.c
	${PROJECT_SRC_FOLDER}/bzlib.h
	${PROJECT_SRC_FOLDER}/bzlib_private.h
	${PROJECT_SRC_FOLDER}/compress.c
	${PROJECT_SRC_FOLDER}/crctable.c
	${PROJECT_SRC_FOLDER}/decompress.c
	${PROJECT_SRC_FOLDER}/huffman.c
	${PROJECT_SRC_FOLDER}/randtable.c
)

if(MSVC)
	add_definitions(-wd4996)
endif()
configure_ext_library(${PROJECT_NAME}
						PROJECT_SOURCE_FILES
						""
						STATIC
						LIBRARY_NAME)
						
