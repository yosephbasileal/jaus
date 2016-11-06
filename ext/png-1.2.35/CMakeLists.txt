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
set(PROJECT_NAME png-1.2.35)

set(PROJECT_SRC_FOLDER ${ACTIVE_EXT_DIRECTORY}/${PROJECT_NAME})
		
set (PROJECT_SRC_FILES 
${PROJECT_SRC_FOLDER}/png.c
${PROJECT_SRC_FOLDER}/pngerror.c
${PROJECT_SRC_FOLDER}/pngget.c
${PROJECT_SRC_FOLDER}/pngmem.c
${PROJECT_SRC_FOLDER}/pngpread.c
${PROJECT_SRC_FOLDER}/pngread.c
${PROJECT_SRC_FOLDER}/pngrio.c
${PROJECT_SRC_FOLDER}/pngrtran.c
${PROJECT_SRC_FOLDER}/pngrutil.c
${PROJECT_SRC_FOLDER}/pngset.c
${PROJECT_SRC_FOLDER}/pngtrans.c
${PROJECT_SRC_FOLDER}/pngwio.c
${PROJECT_SRC_FOLDER}/pngwrite.c
${PROJECT_SRC_FOLDER}/pngwtran.c
${PROJECT_SRC_FOLDER}/pngwutil.c
)

configure_ext_library(${PROJECT_NAME}
						PROJECT_SRC_FILES
						""
						STATIC
						LIBRARY_NAME)