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

include(${ACTIVE_CMAKE_DIRECTORY}/SetupProject.cmake)

# Add any library packages to your workspace by
# updating the LIBRAIRES_TO_SETUP variable with 
# the name of the setup scripts. For example, to
# add Boost, add "SetupBoost.cmake" to to 
# list.

if(NOT LIBRAIRES_TO_SETUP)
	# set(LIBRAIRES_TO_SETUP 
			# SetupJPEGTurbo.cmake
			# SetupZLIB.cmake
			# SetupBZip2.cmake
			# SetupPNG.cmake
			# SetupTinyXML.cmake
			# SetupBoost.cmake
			# SetupNatNet.cmake)
endif()
# Add the Setup script files.
foreach(L ${LIBRAIRES_TO_SETUP})
	message(STATUS "\nIncluding setup file: ${L}")
	include(${ACTIVE_CMAKE_DIRECTORY}/${L})
endforeach()

# Add libraries in order of build, this loop also configures
# the build order
if(NOT PROJECTS_IN_BUILD_ORDER)
	# set(PROJECTS_IN_BUILD_ORDER
			# CxUtils
			# JAUS
			# Cartographer)
endif()

foreach(P ${PROJECTS_IN_BUILD_ORDER})
	add_subdirectory(${ACTIVE_CMAKE_DIRECTORY}/${P} ./${P})
	# message(STATUS ${${P}_INCLUDE_DIRS})
	# message(STATUS ${${P}_DEPENDENCY})
	# message(STATUS ${${P}_LIBRARIES})
endforeach()

