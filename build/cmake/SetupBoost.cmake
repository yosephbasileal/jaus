################################################################################
#
# File: SetupBoost.cmake
# Created by: Daniel Barber, Ph.D.
# Description: This file contains a configuration options for adding 
# Boost support to your workspace and projects.
#
# You must include SetupProject.cmake before including this file.
#
################################################################################

# Macros/Scripts for locating SDK Files

# Configure additional boost version
# for the find_package command to use
set(VERSION_NUMBER 57)
set(NUM_VERSIONS 10)
while(${NUM_VERSIONS} GREATER 0)
	set(Boost_ADDITIONAL_VERSIONS 
		${Boost_ADDITIONAL_VERSIONS}
			"1.${VERSION_NUMBER}"
			"1.${VERSION_NUMBER}.0")
	foreach(P ${DRIVE_LETTERS})
		set(Boost_ROOT_PATHS
				${Boost_ROOT_PATHS}
				${P}/Software/boost
				${P}/boost
		)
		set(Boost_SEARCH_PATHS
				${Boost_SEARCH_PATHS}
				"${P}/Program Files/boost/boost_1_${VERSION_NUMBER}"
				"${P}/Program Files/boost/boost_1_${VERSION_NUMBER}_0"
				"${P}/Program Files/boost_1_${VERSION_NUMBER}"
				"${P}/Program Files/boost_1_${VERSION_NUMBER}_0"
				"${P}/Program Files (x86)/boost/boost_1_${VERSION_NUMBER}"
				"${P}/Program Files (x86)/boost/boost_1_${VERSION_NUMBER}_0"
				"${P}/Program Files (x86)/boost_1_${VERSION_NUMBER}"
				"${P}/Program Files (x86)/boost_1_${VERSION_NUMBER}_0"
				${P}/boost_1_${VERSION_NUMBER}
				${P}/boost_1_${VERSION_NUMBER}_0
				${P}/Software/boost/boost_1_${VERSION_NUMBER}
				${P}/Software/boost/boost_1_${VERSION_NUMBER}_0
				${P}/boost/boost_1_${VERSION_NUMBER}
				${P}/boost/boost_1_${VERSION_NUMBER}_0
				${P}/boost_1_${VERSION_NUMBER}
				${P}/boost_1_${VERSION_NUMBER}_0
			)
	endforeach()
	math(EXPR VERSION_NUMBER "${VERSION_NUMBER} - 1")
	math(EXPR NUM_VERSIONS "${NUM_VERSIONS} - 1")
endwhile()

# Try help find Boost, especially in Windows...

foreach(P ${Boost_ROOT_PATHS})		
	if(IS_DIRECTORY ${P})
		set(BOOST_ROOT 
			${P}
			CACHE PATH 
			"Boost root path directory"
		)
		#message(STATUS "BOOST ROOT: ${BOOST_ROOT}")
		foreach(PVER ${Boost_SEARCH_PATHS})
			#message(STATUS "BOOST LIBS: ${PVER}")
			if(IS_DIRECTORY ${PVER})
				if(IS_DIRECTORY ${PVER}/stage/lib)
					set(BOOST_LIBRARYDIR
						${PVER}/stage/lib
						CACHE PATH
						"Boost library directory")
					break()
				endif()
				if(IS_DIRECTORY ${PVER}/lib)
					set(BOOST_LIBRARYDIR
						${PVER}/lib
						CACHE PATH
						"Boost library directory")
					break()
				endif()
				message(STATUS ${PVER})
			endif()
			
		endforeach()
		break()
	endif()			
endforeach()
# Check for static libraries.
if(BOOST_LIBRARYDIR)
	file(GLOB_RECURSE BOOST_TEMP_BINARIES ${BOOST_LIBRARYDIR}/*.dll)
	if(NOT BOOST_TEMP_BINARIES)
		#message("STATIC BOOST")
		set(Boost_USE_STATIC_LIBS   ON)
	endif()
endif()


if(NOT Boost_FOUND)
	find_package(Boost COMPONENTS thread date_time system REQUIRED)
	include_directories(${Boost_INCLUDE_DIRS})
	if(BOOST_LIBRARYDIR)
		set(Boost_LIBRARY_DIRS
				${BOOST_LIBRARYDIR})
	endif()
	link_directories(${Boost_LIBRARY_DIRS})
	print_lib_project_variables("Boost")
endif()

if(Boost_FOUND AND BOOST_ROOT)
	set(Boost_DIR
			${BOOST_ROOT}
			CACHE PATH
			"Directory containing CMake configuration for boost."
			FORCE
			)
endif()

if(WIN32 AND Boost_FOUND)
	# Add option for copying DLL files to current runtime path.
	option(Boost_COPY_BINARIES "Copy DLL files to local runtime path" FALSE)
	if(Boost_COPY_BINARIES AND NOT Boost_COPY_COMPLETE)	
		message(STATUS "Copying Boost DLL files to runtime path.")
	endif()
	foreach(LIB ${Boost_LIBRARIES})
		if(NOT ${LIB} MATCHES "optimized" AND NOT ${LIB} MATCHES "debug")
			set(BIN ${LIB})
			string(REPLACE ".lib" ".dll" BIN ${BIN})
			string(REPLACE "libboost" "boost" BIN ${BIN})
			if(EXISTS ${BIN})
				set(Boost_BINARIES ${Boost_BINARIES} ${BIN})
				if(Boost_COPY_BINARIES AND NOT Boost_COPY_COMPLETE)
					message(STATUS "    ${BIN}")
					execute_process(COMMAND ${CMAKE_COMMAND} -E copy
										${BIN} ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/
										RESULT_VARIABLE COPY_RESULT
										ERROR_VARIABLE ERROR)		
					if(NOT COPY_RESULT EQUAL 0)
								message(STATUS "Copy Boost - ${ERROR}")
								set(Boost_COPY_COMPLETE_FALSE)
					endif()
				endif()
				
			endif()
		endif()
	endforeach()	
	if(Boost_COPY_BINARIES)
		set(Boost_COPY_COMPLETE TRUE)
	endif()
	# Add binaries to master list of
	# runtime binary files
	set(EXTRA_RUNTIME_BINARIES ${EXTRA_RUNTIME_BINARIES} ${Boost_BINARIES})
endif()
