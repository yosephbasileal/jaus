################################################################################
#
# File: SetupTinyXML.cmake
# Created by: Daniel Barber, Ph.D.
# Description: This file contains a configuration options for addiing 
# TinyXML support to your workspace and projects.
#
# You must include SetupProject.cmake before including this file.
#
################################################################################

# Macros/Scripts for locating SDK Files

# Clear return values in case the module is loaded more than once.
#set(TinyXML_CXX_FLAGS "")	# No Files located yet
#set(TinyXML_BINARIES "")	# Binary files

# Prevent constant execution of this file if included
# multiple times.
if(TinyXML_ADDED)
	return()
endif(TinyXML_ADDED)

set(TinyXML_LIBRARY_NAME tinyxml-2.6.2)

# Use the CMakeLists.txt file
if(NOT TinyXML_ADDED)
	install(DIRECTORY
		${ACTIVE_EXT_DIRECTORY}/${TinyXML_LIBRARY_NAME}
		DESTINATION
		./ext
		COMPONENT SourceCode
		PATTERN *.svn EXCLUDE)
	add_subdirectory(${ACTIVE_EXT_DIRECTORY}/${TinyXML_LIBRARY_NAME} ./3rdParty/${TinyXML_LIBRARY_NAME})
	# Create more user friendly variable names 
	# starting with TinyXML
	copy_lib_project_variables("${TinyXML_LIBRARY_NAME}"
								"TinyXML")
	set(TinyXML_ADDED TRUE)
	#Cache the variable
	set(TinyXML_FOUND 
		TRUE
		CACHE BOOLEAN 
		"${TinyXML_LIBRARY_NAME} Library found if TRUE"
		FORCE)
	print_lib_project_variables("TinyXML")
		
	add_definitions(-DTIXML_USE_STL)
endif(NOT TinyXML_ADDED)


