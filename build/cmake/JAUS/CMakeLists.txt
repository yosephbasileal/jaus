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
# Get applications macros
include(${ACTIVE_CMAKE_DIRECTORY}/CreateApplications.cmake)

set(TARGET_NAME JAUS)

#######################################
#		CORE
#######################################

# Configure subfolders containing code here
# you must do this, or the code will be skipped
set(SOURCE_GROUPS 
				control
				discovery
				events
				liveness
				management
				time
				transport)
set(BUILD_DEPENDENCIES
		CxUtils
		TinyXML)

if(NOT WIN32)
	set(DEPENDENCIES
			Boost)
endif()

if(NOT WIN32)
    set(DEPENDENCIES
            ${DEPENDENCIES}
            rt)
endif()

# Create the library
configure_library(${TARGET_NAME}
				  "Core"
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

#######################################
#		MOBILITY
#######################################

# Configure subfolders containing code here
# you must do this, or the code will be skipped
set(SOURCE_GROUPS 
				drivers
				list
				sensors)

# Create the library
configure_library(${TARGET_NAME}
				  "Mobility"
				  SOURCE_GROUPS
				  BUILD_DEPENDENCIES
				  DEPENDENCIES
				  SHARED
				  LIBRARY_NAME)


#######################################
#		ENVIRONMENT SENSING
#######################################

# Configure subfolders containing code here
# you must do this, or the code will be skipped
set(SOURCE_GROUPS 
		range
		analogvideo
		digitalvideo
		stillimage
		geometric
		visual)

set(DEPENDENCIES 
		${DEPENDENCIES} ZLIB BZip2 LZMA)

# Create the library
configure_library(${TARGET_NAME}
				  "Environment"
				  SOURCE_GROUPS
				  BUILD_DEPENDENCIES
				  DEPENDENCIES
				  SHARED
				  LIBRARY_NAME)

#######################################
#		ENVIRONMENT SENSING
#######################################

add_definitions(-DJPP_IGNORE_DEPRECATED)

# Configure any source code groups
# build dependencies, and regular
# dependencies
set(SOURCE_GROUPS 
		command
		controllers
		mcu
		mobility
		video
		rangesensor)

# Create the library
configure_library(${TARGET_NAME}
				  "Extras"
				  SOURCE_GROUPS
				  BUILD_DEPENDENCIES
				  DEPENDENCIES
				  SHARED
				  LIBRARY_NAME)

# Create example programs that depend on this project
option(BUILD_EXAMPLES_${TARGET_NAME} "Build ${TARGET_NAME} Example Programs." ON)
if(BUILD_EXAMPLES_${TARGET_NAME})
	configure_examples(${TARGET_NAME}
					"examples"
					BUILD_DEPENDENCIES
					DEPENDENCIES
					"Jaus"
					"JAUS++ Examples")
endif()
# Create example programs that depend on this project
option(${TARGET_NAME}_BUILD_TUTORIAL "Build ${TARGET_NAME} Tutorial Programs." ON)
if(${TARGET_NAME}_BUILD_TUTORIAL OR BUILD_JAUS_ONLY)
	configure_examples(${TARGET_NAME}
					"tutorial"
					BUILD_DEPENDENCIES
					DEPENDENCIES
					"Jaus"
					"JAUS++ Tutorial")
endif()
# Create example programs that depend on this project
option(${TARGET_NAME}_BUILD_TESTS "Build ${TARGET_NAME} Example Programs." ON)
if(${TARGET_NAME}_BUILD_TESTS OR BUILD_JAUS_ONLY)
	configure_examples(${TARGET_NAME}
					"tests"
					BUILD_DEPENDENCIES
					DEPENDENCIES
					"Jaus"
					"JAUS++ Unit Tests")
endif()

set(CONSOLE_APP_SOURCE_FILES
		${ACTIVE_SOURCE_DIRECTORY}/jaus/programs/console/JausJoystick.cpp)
if(MSVC)
	set(CONSOLE_APP_SOURCE_FILES
		${CONSOLE_APP_SOURCE_FILES}
		${ACTIVE_RESOURCE_DIRECTORY}/JAUS.rc)
endif()
create_console_app("JausJoystick"
		CONSOLE_APP_SOURCE_FILES
		BUILD_DEPENDENCIES
		DEPENDENCIES)
set(CONSOLE_APP_SOURCE_FILES
		${ACTIVE_SOURCE_DIRECTORY}/jaus/programs/console/JausKeyboard.cpp)
if(MSVC)
	set(CONSOLE_APP_SOURCE_FILES
		${CONSOLE_APP_SOURCE_FILES}
		${ACTIVE_RESOURCE_DIRECTORY}/JAUS.rc)
endif()
create_console_app("JausKeyboard"
		CONSOLE_APP_SOURCE_FILES
		BUILD_DEPENDENCIES
		DEPENDENCIES)
set(CONSOLE_APP_SOURCE_FILES
		${ACTIVE_SOURCE_DIRECTORY}/jaus/programs/console/JausOPC.cpp)
if(MSVC)
	set(CONSOLE_APP_SOURCE_FILES
		${CONSOLE_APP_SOURCE_FILES}
		${ACTIVE_RESOURCE_DIRECTORY}/JAUS.rc)
endif()
create_console_app("JausOPC"
		CONSOLE_APP_SOURCE_FILES
		BUILD_DEPENDENCIES
		DEPENDENCIES)

set(WX_APPLICATIONS 	
NodeManager)

foreach(APP ${WX_APPLICATIONS})

	string(TOLOWER ${APP} APP_LCASE)
	string(TOLOWER ${TARGET_NAME} TARGET_NAME_LC)
	file(GLOB_RECURSE 
                GUI_SOURCES ${ACTIVE_SOURCE_DIRECTORY}/${TARGET_NAME_LC}/programs/gui/${APP_LCASE}/*.cpp)
	file(GLOB_RECURSE 
                GUI_HEADERS ${ACTIVE_SOURCE_DIRECTORY}/${TARGET_NAME_LC}/programs/gui/${APP_LCASE}/*.h)
	
	include_directories(${ACTIVE_SOURCE_DIRECTORY}/${TARGET_NAME_LC}/programs/gui/${APP_LCASE}/)
	set(APP_NAME "Jaus${APP}")
	set(APP_SOURCES 
			${GUI_PANEL_HEADERS}
			${GUI_PANEL_SOURCES}
			${GUI_SOURCES}
			${GUI_HEADERS})
	if(MSVC)
		set(APP_SOURCES
			${APP_SOURCES}
			${ACTIVE_RESOURCE_DIRECTORY}/JAUS.rc)
	endif()
	create_wx_app(${APP_NAME}
				  APP_SOURCES
				  BUILD_DEPENDENCIES
				  DEPENDENCIES)
endforeach()
