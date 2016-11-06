################################################################################
#
# File: SetupBZip2.cmake
# Created by: Daniel Barber, Ph.D.
# Description: This file contains a configuration options for addiing 
# BZip2 support to your workspace and projects.
#
# You must include SetupProject.cmake before including this file.
#
################################################################################

# Macros/Scripts for locating SDK Files

# Clear return values in case the module is loaded more than once.
#set(BZip2_CXX_FLAGS "")	# No Files located yet
#set(BZip2_BINARIES "")	# Binary files

# Prevent constant execution of this file if included
# multiple times.
if(BZip2_ADDED)
	return()
endif(BZip2_ADDED)

set(BZip2_LIBRARY_NAME bzip2-1.0.6)

# Use the CMakeLists.txt file
if(NOT BZip2_ADDED)
	install(DIRECTORY
		${ACTIVE_EXT_DIRECTORY}/${BZip2_LIBRARY_NAME}
		DESTINATION
		./ext
		COMPONENT SourceCode
		PATTERN *.svn EXCLUDE)
	add_subdirectory(${ACTIVE_EXT_DIRECTORY}/${BZip2_LIBRARY_NAME} ./3rdParty/${BZip2_LIBRARY_NAME})
	# Create more user friendly variable names 
	# starting with BZip2
	copy_lib_project_variables("${BZip2_LIBRARY_NAME}"
								"BZip2")
	set(BZip2_ADDED TRUE)
	#Cache the variable
	set(BZip2_FOUND 
		TRUE
		CACHE BOOLEAN 
		"${BZip2_LIBRARY_NAME} Library found if TRUE"
		FORCE)
	print_lib_project_variables("BZip2")
endif(NOT BZip2_ADDED)

