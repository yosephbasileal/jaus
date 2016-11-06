################################################################################
#
# File: SetupPNG.cmake
# Created by: Daniel Barber, Ph.D.
# Description: This file contains a configuration options for addiing 
# PNG support to your workspace and projects.
#
# You must include SetupProject.cmake before including this file.
#
################################################################################

# Macros/Scripts for locating SDK Files

# Clear return values in case the module is loaded more than once.
#set(PNG_CXX_FLAGS "")	# No Files located yet
#set(PNG_BINARIES "")	# Binary files

# Prevent constant execution of this file if included
# multiple times.
if(PNG_ADDED)
	return()
endif(PNG_ADDED)

set(PNG_LIBRARY_NAME png-1.2.35)

# Use the CMakeLists.txt file
if(NOT PNG_ADDED)
	install(DIRECTORY
		${ACTIVE_EXT_DIRECTORY}/${PNG_LIBRARY_NAME}
		DESTINATION
		./ext
		COMPONENT SourceCode
		PATTERN *.svn EXCLUDE)
	if(WIN32 OR XCODE_VERSION)
		add_subdirectory(${ACTIVE_EXT_DIRECTORY}/${PNG_LIBRARY_NAME} ./3rdParty/${PNG_LIBRARY_NAME})
		# Create more user friendly variable names 
		# starting with PNG
		copy_lib_project_variables("${PNG_LIBRARY_NAME}"
									"PNG")
		print_lib_project_variables("PNG")
		if(WIN32)
			set(PNG_BINARIES ${PNG_DEPENDENCY}.dll)
		endif(WIN32)

		if(ZLIB_IS_DEPENDENCY)
			add_dependencies(${PNG_DEPENDENCY}
				${ZLIB_DEPENDENCY})
		endif()
		set_target_properties(${PNG_DEPENDENCY} 
			PROPERTIES COMPILE_FLAGS "-I${ZLIB_INCLUDE_DIRS} -wd4996")
	else()
		# For linux, you can use the PNG find_package script
		find_package(PNG)
		set(PNG_LIBRARIES png z)
                set(ZLIB_LIBRARIES z)
		print_lib_project_variables("PNG")
	endif()
endif(NOT PNG_ADDED)
