################################################################################
#
# File: SetupJPEGTurbo.cmake
# Created by: Daniel Barber, Ph.D.
# Description: This file contains a configuration options for addiing 
# JPEG support to a project. It first looks to see if the OS supports
# the JPEG Turbo libraries, and if unavailable, defaults to regular JPEG
# library. If JPEG is not installed on the system, then a project for
# compiling JPEG code in the external dependencies folder is used.
#
# You must include SetupProject.cmake before including this file.
#
################################################################################

# Macros/Scripts for locating SDK Files
# This macro determines what version of JPEG is installed on the
# the system or available and configures the appropriate options

# Prevent constant execution of this file if included
# multiple times.
if(JPEGTurbo_ADDED)
	return()
endif(JPEGTurbo_ADDED)

set(JPEG_LIBRARY_NAME jpeg-6b)

if(WIN32)
	install(DIRECTORY
		${ACTIVE_EXT_DIRECTORY}/libjpeg-turbo-1.0.1
		${ACTIVE_EXT_DIRECTORY}/jpeg-6b
		DESTINATION
		./ext
		COMPONENT SourceCode
		PATTERN *.svn EXCLUDE)
	# Use precompiled binary of libjpeg-turbo.
	if(MSVC) 
		set(JPEGTurbo_INCLUDE_DIRS 
					${ACTIVE_EXT_DIRECTORY}/libjpeg-turbo-1.0.1
					${ACTIVE_EXT_DIRECTORY}/libjpeg-turbo-1.0.1/win)
		# Use the appropriate pre-compiled binary name based on
		# version of Visual Studio
		if(MSVC90)
			set(JPEGTurbo_LIBRARIES libjpegturbo.lib)
		elseif(MSVC10)
			set(JPEGTurbo_LIBRARIES libjpegturbo_vs10.lib)
		else()
			set(JPEGTurbo_LIBRARIES libjpegturbo_vs11.lib)
		endif(MSVC90)
		#message(${JPEGTurbo_LIBRARIES})
		# Check for 32 or 64 bit
		if(CMAKE_CL_64)
			find_path(JPEGTurbo_LIBRARY_DIRS 
				${JPEGTurbo_LIBRARIES} 
				${ACTIVE_EXT_DIRECTORY}/libjpeg-turbo-1.0.1/libs/x64)
		else()
			find_path(JPEGTurbo_LIBRARY_DIRS 
				${JPEGTurbo_LIBRARIES} 
				${ACTIVE_EXT_DIRECTORY}/libjpeg-turbo-1.0.1/libs/x32)
		endif(CMAKE_CL_64)
		if(JPEGTurbo_LIBRARY_DIRS)
			set(JPEGTurbo_LIBRARIES  ${JPEGTurbo_LIBRARY_DIRS}/${JPEGTurbo_LIBRARIES})
			# Cache the variable
			set(JPEGTurbo_FOUND 
				TRUE
				CACHE BOOLEAN 
				"JPEG-Turbo Library found if TRUE, otherwise using jpeg-6b" )
			set(JPEGTurbo_ADDED TRUE)
		endif(JPEGTurbo_LIBRARY_DIRS)
	endif(MSVC)
	
	# Build version of jpeg-6b that uses BGR ordering instead of RGB
	if(NOT JPEGTurbo_ADDED)   
		add_subdirectory(${ACTIVE_EXT_DIRECTORY}/jpeg-6b/ ./3rdParty/JPEG-6b/)
		set(JPEGTurbo_INCLUDE_DIRS 
					${ACTIVE_EXT_DIRECTORY}/jpeg-6b/msvc)
		set(JPEGTurbo_LIBRARY_DIRS ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}) 
		set(JPEGTurbo_LIBRARIES 
				optimized ${JPEG_LIBRARY_NAME}
				debug ${JPEG_LIBRARY_NAME}${LIBRARY_DEBUG_POSTFIX})
		set(JPEGTurbo_DEPENDENCY ${LIBRARY_PREFIX}${JPEG_LIBRARY_NAME})
		set(JPEGTurbo_ADDED TRUE)
		# Cache the variable
		set(JPEGTurbo_FOUND 
			FALSE
			CACHE BOOLEAN 
			"JPEG-Turbo Library found if TRUE, otherwise using ${JPEG_LIBRARY_NAME}" )
	endif(NOT JPEGTurbo_ADDED)
	
else(UNIX)
	find_path(JPEGTurbo_LIBRARY_DIRS 
					NAMES
					libjpeg.so 
					PATHS
					/opt/libjpeg-turbo/lib64
					/opt/libjpeg-turbo/lib )


#	if(JPEGTurbo_LIBRARY_DIRS)
#		set(JPEGTurbo_LIBRARIES "${JPEGTurbo_LIBRARY_DIRS}/libjpeg.so")
#		set(JPEGTurbo_CXX_FLAGS
#			-DJPEG_TURBO_DIR="/opt/libjpeg-turbo/include/jpeglib.h")
#		set(JPEGTurbo_FOUND
#				TRUE
#				CACHE BOOLEAN
#				"JPEG-Turbo Library found if TRUE, otherwise using jpeg-6b" )
#		set(JPEGTurbo_ADDED TRUE)
#	endif()
	
	# Build version of jpeg-6b that uses BGR ordering instead of RGB
	if(NOT JPEGTurbo_ADDED)   
		add_subdirectory(${ACTIVE_EXT_DIRECTORY}/${JPEG_LIBRARY_NAME}/ 
			./3rdParty/${JPEG_LIBRARY_NAME}/)
		set(JPEGTurbo_INCLUDE_DIRS 
					${ACTIVE_EXT_DIRECTORY}/jpeg-6b/)
		set(JPEGTurbo_LIBRARY_DIRS ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}) 
		set(JPEGTurbo_LIBRARIES 
				optimized ${JPEG_LIBRARY_NAME}
				debug ${JPEG_LIBRARY_NAME}${LIBRARY_DEBUG_POSTFIX})
		set(JPEGTurbo_DEPENDENCY ${LIBRARY_PREFIX}${JPEG_LIBRARY_NAME})
		set(JPEGTurbo_ADDED TRUE)
		# Cache the variable
		set(JPEGTurbo_FOUND 
			FALSE
			CACHE BOOLEAN 
			"JPEG-Turbo Library found if TRUE, otherwise using jpeg-6b" )
	endif(NOT JPEGTurbo_ADDED)
endif(WIN32)

# Print debug information for testing.
if(DEBUG_ALL_ACTIVE_CMAKE)
	message(STATUS "Configuring External Library [JPEG-Turbo]")
	message(STATUS "  JPEG found.")
	message(STATUS "  Include Directories:")
	foreach(F ${JPEGTurbo_INCLUDE_DIRS})
		message(STATUS "    ${F}")
	endforeach()
	message(STATUS "  Libraries:")
	foreach(F ${JPEGTurbo_LIBRARIES})
		message(STATUS "    ${F}")
	endforeach()
endif()

