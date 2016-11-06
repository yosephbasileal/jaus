################################################################################
#
# File: SetupVisualLeakDetector.cmake
# Created by: Daniel Barber, Ph.D.
# Description: This file contains a configuration options for adding 
# Visual Leak Detector support to Visual Studio projects.
#
# You must include SetupProject.cmake before including this file.
#
################################################################################


if(WIN32)
	find_path(VLD_ROOT_DIR
				NAMES 
					include/vld.h
				PATHS
					"C:/Program Files (x86)/Visual Leak Detector/"
			  )
	if(VLD_ROOT_DIR)
		set(VLD_FOUND TRUE)
		set(VLD_CXX_FLAGS -DVLD_ENABLED)# Compiler Flags
		include_directories(${VLD_ROOT_DIR}/include)
		if(CMAKE_CL_64) # Get version for architecture type
			link_directories(${VLD_ROOT_DIR}/lib/Win64)
		else()
			link_directories(${VLD_ROOT_DIR}/lib/Win32)
		endif(CMAKE_CL_64)
		add_definitions(${VLD_CXX_FLAGS})
	endif(VLD_ROOT_DIR)
endif()


