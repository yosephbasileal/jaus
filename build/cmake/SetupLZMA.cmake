################################################################################
#
# File: SetupLZMA.cmake
# Created by: Daniel Barber, Ph.D.
# Description: This file contains a configuration options for addiing 
# LZMA support to your workspace and projects.
#
# You must include SetupProject.cmake before including this file.
#
################################################################################

# Macros/Scripts for locating SDK Files

# Clear return values in case the module is loaded more than once.
#set(LZMA_CXX_FLAGS "")	# No Files located yet
#set(LZMA_BINARIES "")	# Binary files

# Prevent constant execution of this file if included
# multiple times.
if(LZMA_ADDED)
	return()
endif(LZMA_ADDED)

set(LZMA_LIBRARY_NAME lzma920)

# Use the CMakeLists.txt file
if(NOT LZMA_ADDED)
	install(DIRECTORY
		${ACTIVE_EXT_DIRECTORY}/${LZMA_LIBRARY_NAME}
		DESTINATION
		./ext
		COMPONENT SourceCode
		PATTERN *.svn EXCLUDE)
	add_subdirectory(${ACTIVE_EXT_DIRECTORY}/${LZMA_LIBRARY_NAME} ./3rdParty/${LZMA_LIBRARY_NAME})
	# Create more user friendly variable names 
	# starting with LZMA
	copy_lib_project_variables("${LZMA_LIBRARY_NAME}"
								"LZMA")
	set(LZMA_ADDED TRUE)
	#Cache the variable
	set(LZMA_FOUND 
		TRUE
		CACHE BOOLEAN 
		"${LZMA_LIBRARY_NAME} Library found if TRUE"
		FORCE)
	print_lib_project_variables("LZMA")
endif(NOT LZMA_ADDED)

