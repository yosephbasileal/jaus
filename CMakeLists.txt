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

# Set Solution/Workspace name, this will be re-used
set(WORKSPACE_NAME ACTIVE)
# Set version number
set(WORKSPACE_VERSION 5)

# Create solution/workspace/project
project(${WORKSPACE_NAME})

# Include this file next BEFORE ALL OTHERS
# as it is required to configure variables used
# in all scripts/macros
include(build/cmake/SetupProject.cmake)

option(BUILD_JAUS_ONLY "Build only JAUS++ related software." ON)
# Add any library packages to your workspace by
# updating the LIBRAIRES_TO_SETUP variable with 
# the name of the setup scripts. For example, to
# add Boost, add "SetupBoost.cmake" to to 
# list.

set(LIBRAIRES_TO_SETUP 
		SetupVisualLeakDetector.cmake
		SetupBoost.cmake
		SetupBZip2.cmake
		SetupLZMA.cmake
		SetupJPEGTurbo.cmake
		SetupZLIB.cmake
		SetupPNG.cmake
		SetupTinyXML.cmake
		)

# Add all the projects you want added by creating
# the PROJECTS_IN_BUILD_ORDER variable. If this
# variable is not set, then all libraries
# will be added.

set(PROJECTS_IN_BUILD_ORDER
		CxUtils
		JAUS
		)
		
# Add optional projects and required
# libraries...
if( NOT BUILD_JAUS_ONLY )
	if(BUILD_Cartographer_LIBRARIES OR
		BUILD_Video_LIBRARIES OR
		BUILD_Kinect_LIBRARIES)
		
		set(LIBRAIRES_TO_SETUP 
				${LIBRAIRES_TO_SETUP}
					SetupOpenCV.cmake
			)
	endif()
endif()


# Configure what source code we want to
# release. For example, we do not want to
# release any RIVET related code, which is
# added later in this file.
set(OPEN_SOURCE_PROJECTS
		${PROJECTS_IN_BUILD_ORDER})

# Run the Add Libraries file, which will add all
# library projects to the worksapce based on the 
# LIBRARIES_TO_SETUP and PROJECTS_IN_BUILD_ORDER 
# variables.
include(${ACTIVE_CMAKE_DIRECTORY}/AddProjects.cmake)
	
# Don't go beyond here...
if(NOT WIN32)
	return()
endif()

# Everyone below this point is for configuration
# of CPack options for software installation

# Ensures requiredd system functions are
# included with installer
include(InstallRequiredSystemLibraries)

set(CONTENT_FOLDERS
		bin/icons)
		
if(NOT BUILD_JAUS_ONLY)
	set(CONTENT_FOLDERS
			${CONTENT_FOLDERS}
			bin/settings)
endif()

################ RUNTIME FILES #############

# Install all runtime content
# Binaries are included with SDK
install(DIRECTORY
		${CONTENT_FOLDERS}
		DESTINATION 
		bin
		COMPONENT Binaries
		PATTERN *.svn EXCLUDE)
		
if(BUILD_JAUS_ONLY)
	install(DIRECTORY
		bin/settings/jaus
		DESTINATION 
		bin/settings
		COMPONENT Binaries
		PATTERN *.svn EXCLUDE)
endif()

# Install 3rdParty DLL Files
install(FILES
		${EXTRA_RUNTIME_BINARIES}
		DESTINATION 
		bin
		COMPONENT Binaries)

if(BUILD_JAUS_ONLY)
	# Documentation
	install(DIRECTORY
			doc/JAUS++
			DESTINATION
			doc
			COMPONENT Binaries)
else()
	# Documentation
	install(DIRECTORY
			doc
			DESTINATION
			./
			COMPONENT Binaries)
endif()		
# Documentation
install(DIRECTORY
		doc
		DESTINATION
		./
		COMPONENT Binaries)

################## SDK FILES ###############

foreach(PRJ ${OPEN_SOURCE_PROJECTS})
	string(TOLOWER ${PRJ} DIR_NAME)
	install(DIRECTORY
		src/${DIR_NAME}
		DESTINATION
		./include
		COMPONENT SDK
		FILES_MATCHING
		PATTERN *.h
		PATTERN *.svn EXCLUDE)
endforeach()

# Install 3rdParty DLL Files
install(FILES
		${EXTRA_RUNTIME_BINARIES}
		DESTINATION 
		bin
		COMPONENT SDK)
# Documentation
if(BUILD_JAUS_ONLY)
	# Documentation
	install(DIRECTORY
			doc/JAUS++
			DESTINATION
			doc
			COMPONENT SDK)
else()
	# Documentation
	install(DIRECTORY
			doc
			DESTINATION
			./
			COMPONENT SDK)
endif()	
		
############## SOURCE CODE FILES ###########

# Include same content as part of the
# SourceCode installation
install(DIRECTORY
		${CONTENT_FOLDERS}
		DESTINATION 
		bin
		COMPONENT SourceCode
		PATTERN *.svn EXCLUDE)
if(BUILD_JAUS_ONLY)
	install(DIRECTORY
		bin/settings/jaus
		DESTINATION 
		bin/settings
		COMPONENT SourceCode
		PATTERN *.svn EXCLUDE)
endif()

install(DIRECTORY
		build/cmake
		build/wxFormBuilder
		DESTINATION
		build
		COMPONENT SourceCode
		PATTERN *.svn EXCLUDE)

foreach(PRJ ${OPEN_SOURCE_PROJECTS})
	string(TOLOWER ${PRJ} DIR_NAME)
	install(DIRECTORY
		src/${DIR_NAME}
		DESTINATION
		./src
		COMPONENT SourceCode
		PATTERN *.svn EXCLUDE)
endforeach()

install(DIRECTORY
		resources
		doc
		DESTINATION
		./
		COMPONENT SourceCode
		PATTERN *.svn EXCLUDE)
install(FILES
		./CMakeLists.txt
		DESTINATION
		./
		COMPONENT SourceCode)
		
set(PACKAGE_NAME ${WORKSPACE_NAME})
if(BUILD_JAUS_ONLY)
	set(PACKAGE_NAME "JAUS++")
endif()
set(CPACK_PACKAGE_NAME ${PACKAGE_NAME})
set(CPACK_PACKAGE_VENDOR "Univeristy of Central Florida - IST - ACTIVE Lab")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "ACTIVE Open Source Software.")
# Get the current date
get_date(BUILD_DATE YEAR MONTH DAY)
set(CPACK_PACKAGE_VERSION "${WORKSPACE_VERSION}.${YEAR}${MONTH}${DAY}")
set(CPACK_PACKAGE_INSTALL_DIRECTORY ${PACKAGE_NAME})

# NULLSOFT Installer Options
set(CPACK_NSIS_MENU_LINKS
		"bin" "Programs")
set(CPACK_NSIS_MODIFY_PATH "ON")
	set(CPACK_NSIS_EXTRA_INSTALL_COMMANDS ${CPACK_NSIS_EXTRA_INSTALL_COMMANDS}
		"SetOverwrite on")
		
set(CPACK_COMPONENTS_ALL Binaries SDK SourceCode)

include(CPack)
