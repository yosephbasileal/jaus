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

# Configure project for this build environment
include(${ACTIVE_CMAKE_DIRECTORY}/SetupProject.cmake)
# Get library macros
include(${ACTIVE_CMAKE_DIRECTORY}/CreateLibrary.cmake)
# Get application macros
include(${ACTIVE_CMAKE_DIRECTORY}/CreateApplications.cmake)

set(TARGET_NAME CxUtils)

# Configure subfolders containing code here
# you must do this, or the code will be skipped
set(SOURCE_GROUPS 
				images
				ipc
				math
				networking
				plugin
				xml)
if(WIN32)
	set(BUILD_DEPENDENCIES
			JPEGTurbo
			ZLIB
			PNG
			TinyXML)
else()
	set(BUILD_DEPENDENCIES
			JPEGTurbo
			PNG
			TinyXML)
endif()
if(WIN32)
	set(DEPENDENCIES winmm wsock32)
else()
	find_package(X11)
	find_path(XTST_DIR extensions/XTest.h keysym.h /usr/include/X11 /usr/local/include/X11)
	find_library(XTST_LIBRARY NAMES Xtst PATH /usr/lib /usr/local/lib)
	find_path(PTHREAD_DIR pthread.h /usr/include/ /usr/local/include/)
	find_library(PTHREAD_LIBRARY NAMES pthread PATH /usr/lib /usr/local/lib)
	set(DEPENDENCIES X11 Xtst pthread)
	add_definitions(${JPEGTurbo_CXX_FLAGS})
endif(WIN32)
		
# Create the library
configure_library(${TARGET_NAME}
				  ""
				  SOURCE_GROUPS
				  BUILD_DEPENDENCIES
				  DEPENDENCIES
				  SHARED
				  LIBRARY_NAME)

# Anything following this point will require
# this project as a dependency
set(BUILD_DEPENDENCIES
		${BUILD_DEPENDENCIES}
		${TARGET_NAME})


# Create example programs that depend on this project
option(BUILD_EXAMPLES_${TARGET_NAME} "Build ${TARGET_NAME} Example Programs." FALSE)
if(BUILD_EXAMPLES_${TARGET_NAME})
	configure_examples(${TARGET_NAME}
					"examples"
					BUILD_DEPENDENCIES
					DEPENDENCIES)
endif()
