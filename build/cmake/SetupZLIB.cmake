################################################################################
#
# File: SetupZLIB.cmake
# Created by: Daniel Barber, Ph.D.
# Description: This file contains a configuration options for addiing 
# ZLIB support to your workspace and projects.
#
# You must include SetupProject.cmake before including this file.
#
################################################################################

# Macros/Scripts for locating SDK Files

# Clear return values in case the module is loaded more than once.
#set(ZLIB_CXX_FLAGS "")	# No Files located yet
#set(ZLIB_BINARIES "")	# Binary files

# Prevent constant execution of this file if included
# multiple times.
if(ZLIB_ADDED)
	return()
endif(ZLIB_ADDED)

if(WIN32)
	set(ZLIB_LIBRARY_NAME zlib-1.2.6)
else()
	set(ZLIB_LIBRARY_NAME zlib)
endif()

# Use the CMakeLists.txt file
if(NOT ZLIB_ADDED)
	install(DIRECTORY
		${ACTIVE_EXT_DIRECTORY}/${ZLIB_LIBRARY_NAME}
		DESTINATION
		./ext
		COMPONENT SourceCode
		PATTERN *.svn EXCLUDE)
	if(WIN32)
		add_subdirectory(${ACTIVE_EXT_DIRECTORY}/${ZLIB_LIBRARY_NAME} ./3rdParty/${ZLIB_LIBRARY_NAME})
		set(ZLIB_INCLUDE_DIRS 
					${ACTIVE_EXT_DIRECTORY}/${ZLIB_LIBRARY_NAME}
					${CMAKE_CURRENT_BINARY_DIR}/3rdParty/${PACKAGE_BINARY_DIR}${ZLIB_LIBRARY_NAME})
		set(ZLIB_LIBRARY_DIRS ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}) 
		set(ZLIB_LIBRARIES 
				optimized ${ZLIB_LIBRARY_NAME}
				debug ${ZLIB_LIBRARY_NAME}${LIBRARY_DEBUG_POSTFIX})
		set(ZLIB_DEPENDENCY ${ZLIB_LIBRARY_NAME})
		# Signal with this variable that we can
		# add zLib as a build dependency
		set(ZLIB_IS_DEPENDENCY TRUE)
		set(ZLIB_ADDED TRUE)
		# Visual Studio Solution folders.
		if(MSVC_SOLUTION_FOLDERS)
			set_property(TARGET ${ZLIB_LIBRARY_NAME} 
				PROPERTY FOLDER "3rd Party Libraries")
		endif(MSVC_SOLUTION_FOLDERS)
		# Cache the variable
		set(ZLIB_FOUND 
			TRUE
			CACHE BOOLEAN 
			"${ZLIB_LIBRARY_NAME} Library found if TRUE"
			FORCE)
	else()
		find_path(ZLIB_INCLUDE_DIRS
			zlib.h
			PATHS
			/usr/local/include
			/usr/include)
		if(ZLIB_INCLUDE_DIRS)
			# Cache the variable
			set(ZLIB_FOUND 
				TRUE
				CACHE BOOLEAN 
				"${ZLIB_LIBRARY_NAME} Library found if TRUE"
				FORCE)
			set(ZLIB_LIBRARIES z FORCE)
		endif()		
	endif()
endif(NOT ZLIB_ADDED)

# Print debug information for testing.
if(ACTIVE_CMAKE_DEBUG)
	message(STATUS "Configuring External Library [${ZLIB_LIBRARY_NAME}]")
	message(STATUS "  ${ZLIB_LIBRARY_NAME} Found.")
	message(STATUS "  Include Directories:")
	foreach(F ${ZLIB_INCLUDE_DIRS})
		message(STATUS "    ${F}")
	endforeach()
	message(STATUS "  Libraries:")
	foreach(F ${ZLIB_LIBRARIES})
		message(STATUS "    ${F}")
	endforeach()
endif(ACTIVE_CMAKE_DEBUG)

