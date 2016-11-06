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
set(PROJECT_NAME tinyxml-2.6.2)

set(PROJECT_SRC_FOLDER ${ACTIVE_EXT_DIRECTORY}/${PROJECT_NAME})

set (PROJECT_SOURCE_FILES 
	${PROJECT_SRC_FOLDER}/tinystr.cpp
	${PROJECT_SRC_FOLDER}/tinyxml.cpp
	${PROJECT_SRC_FOLDER}/tinyxmlerror.cpp
	${PROJECT_SRC_FOLDER}/tinyxmlparser.cpp
	${PROJECT_SRC_FOLDER}/tinystr.h
	${PROJECT_SRC_FOLDER}/tinyxml.h
)

if(WIN32)
	add_definitions(-D_MBCS -DMBCS)
	configure_ext_library(${PROJECT_NAME}
							PROJECT_SOURCE_FILES
							""
							STATIC
							LIBRARY_NAME)
else()
	configure_ext_library(${PROJECT_NAME}
						PROJECT_SOURCE_FILES
						""
						SHARED
						LIBRARY_NAME)
endif()		

set_target_properties(${LIBRARY_NAME} 
		PROPERTIES COMPILE_FLAGS 
		"-DTIXML_USE_STL")

