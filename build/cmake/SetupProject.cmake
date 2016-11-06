################################################################################
#
# File: SetupWorkspace.cmake
# Created by: Daniel Barber, Ph.D.
# Description: This file contains configures variables common to all
# libraries and runtimes used within this software environment to be 
# common across different build systems. It also contains macros 
# to support creation of CMakeLists.txt project files, like those needed
# for configuring CPack options.
#
################################################################################
# Make sure that this file is called after the call to project
# to ensure setting of specific CMake environement variables.

##############################################
# VARIABLES GENERATED
##############################################
# ACTIVE_PROJECT_ROOT_DIRECTORY	- Root folder for the project/workspace
# ACTIVE_PROJECT_RUNTIME_OUTPUT_DIRECTORY - Output path for runtime/DLL files
# ACTIVE_LIBRARY_OUTPUT_DIRECTORY - Output path for library files
# ACTIVE_CMAKE_DIRECTORY - CMake folder in the project tree
# ACTIVE_EXT_DIRECTORY - Directory containing small 3rd Party libraries required
# ACTIVE_RESOURCE_DIRECTORY - Directory containing Windows Resources
#						 	files.
# ACTIVE_SOURCE_DIRECTORY - Directory containing source files.
# LIBRARY_PREFIX	- "lib" prefix on library names
# LIBRARY_DEBUG_POSTFIX	- Used to add _d at the end of debug files
# LIBRARY_SEARCH_PATHS - Default paths to look for common 3rd Party libraries
# DEBUG_ALL_ACTIVE_CMAKE - Display debug messages from configuration macros.
# DEBUG_ACTIVE_CMAKE_SHOW_SRC_FILES - Dsiplay source files found when showing
#					debug messages.
if(WIN32)
	if(NOT MSVC AND NOT MINGW)
		message(STATUS "Please declare project before inclusion of this "
					   "Setup file.")
	endif()
endif()


option(DEBUG_ALL_ACTIVE_CMAKE "Print debug messages in CMakeLists.txt files." FALSE)
option(DEBUG_ACTIVE_CMAKE_SHOW_SRC_FILES
			"Print source files info in debug messages." FALSE)

if(MSVC)
	option(MSVC_SOLUTION_FOLDERS 
		"Use solution folders (e.g. Visual Studio)" TRUE)
	if(MSVC_SOLUTION_FOLDERS)
		set_property(GLOBAL PROPERTY USE_FOLDERS ON)
	endif()
endif()

if(NOT WIN32)
	if(${CMAKE_BUILD_TYPE} MATCHES "Debug")
		add_definitions(-DUNIX_DEBUG_BUILD)
		#message(STATUS "\tDEBUG build.")
	else()
		#message(STATUS "\tRELEASE build.")
	endif()
endif()

if(WIN32)
	set(DRIVE_LETTERS
			"C:"
			"D:"
			"F:"
			"Z:")
	foreach(DRIVE ${DRIVE_LETTERS})
		set(LIBRARY_SEARCH_PATHS
				${LIBRARY_SEARCH_PATHS}
				${DRIVE}/
				"${DRIVE}/Active"
				"${DRIVE}/active"
				"${DRIVE}/Software"
				"${DRIVE}/software"
				"${DRIVE}/Software/Active"
				"${DRIVE}/software/active"
				../
				../../
				../../../
				../../../../
				../../../../../
				)
	endforeach()
else()
		set(LIBRARY_SEARCH_PATHS
			"~/software/active"
			"~/software"
			"~/Software/Active"
			"~/Software/active")

endif()		

set(ACTIVE_LIB_VARIABLE_TYPES
				DEPENDENCY
				INCLUDE_DIRS
				LIBRARIES
				LIBS
				LIBRARY_DIR
				LIBRARY_DIRS
				IS_DEPENDENCY
				FOUND
				CXX_FLAGS
				BINARIES)
				
# Set/Discover the output folder for all
# executeables built by this project.
if(NOT ACTIVE_PROJECT_ROOT_DIRECTORY)
	# Setup installer default install path.
	set(CMAKE_INSTALL_PREFIX 
		"C:\\UCF"
		CACHE PATH 
		"Root UCF folder." FORCE )
	get_filename_component(ACTIVE_PROJECT_RUNTIME_OUTPUT_DIRECTORY 
		${CMAKE_SOURCE_DIR}/bin ABSOLUTE)
	if(NOT IS_DIRECTORY ${ACTIVE_PROJECT_RUNTIME_OUTPUT_DIRECTORY})
		get_filename_component(ACTIVE_PROJECT_RUNTIME_OUTPUT_DIRECTORY 
			${CMAKE_SOURCE_DIR}/../../bin ABSOLUTE)
			if(NOT IS_DIRECTORY ${ACTIVE_PROJECT_RUNTIME_OUTPUT_DIRECTORY})
			get_filename_component(ACTIVE_PROJECT_RUNTIME_OUTPUT_DIRECTORY 
				${CMAKE_SOURCE_DIR}/../../../bin ABSOLUTE)
		endif()
	endif()
	# Convert to an absolute path variable.
	get_filename_component(ACTIVE_PROJECT_ROOT_DIRECTORY 
		${ACTIVE_PROJECT_RUNTIME_OUTPUT_DIRECTORY}/../ ABSOLUTE)
	# Cache the variable
	set(ACTIVE_PROJECT_ROOT_DIRECTORY 
		${ACTIVE_PROJECT_ROOT_DIRECTORY}
		CACHE PATH 
		"Root UCF folder." )
endif(NOT ACTIVE_PROJECT_ROOT_DIRECTORY)

# Setup path to CMake files variable.
set(ACTIVE_EXT_DIRECTORY 
		${ACTIVE_PROJECT_ROOT_DIRECTORY}/ext)
		
# Setup path to CMake files variable.
set(ACTIVE_CMAKE_DIRECTORY 
		${ACTIVE_PROJECT_ROOT_DIRECTORY}/build/cmake)

# Setup path to resources files variable.
set(ACTIVE_RESOURCE_DIRECTORY 
		${ACTIVE_PROJECT_ROOT_DIRECTORY}/resources)
		
# Setup path to CMake files variable.
set(ACTIVE_LIBRARY_OUTPUT_DIRECTORY 
		${ACTIVE_PROJECT_ROOT_DIRECTORY}/lib )
		
set(ACTIVE_PROJECT_RUNTIME_OUTPUT_DIRECTORY 
			${ACTIVE_PROJECT_ROOT_DIRECTORY}/bin)

# You can set to true to stop this.
if(NOT CMAKE_OUTPUT_DIRECTORIES_SET)
	# Update CMake output directories.		
	set(CMAKE_RUNTIME_OUTPUT_DIRECTORY 
			${ACTIVE_PROJECT_RUNTIME_OUTPUT_DIRECTORY})
	set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG 
			${ACTIVE_PROJECT_RUNTIME_OUTPUT_DIRECTORY})
	set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE 
			${ACTIVE_PROJECT_RUNTIME_OUTPUT_DIRECTORY})		
	# Make sure DLL files are generated in the RUNTIME path for Windows
	if(WIN32)
		# Make sure DLL files are in the same directory as executeables in Windows
		set(CMAKE_LIBRARY_OUTPUT_DIRECTORY 
				${ACTIVE_PROJECT_RUNTIME_OUTPUT_DIRECTORY})
		set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_DEBUG 
				${ACTIVE_PROJECT_RUNTIME_OUTPUT_DIRECTORY})
		set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_RELEASE 
				${ACTIVE_PROJECT_RUNTIME_OUTPUT_DIRECTORY})
		set(EXECUTABLE_OUTPUT_PATH 
				${ACTIVE_PROJECT_RUNTIME_OUTPUT_DIRECTORY})
	else()
		set(CMAKE_LIBRARY_OUTPUT_DIRECTORY 
				${ACTIVE_LIBRARY_OUTPUT_DIRECTORY})
		set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_DEBUG 
				${ACTIVE_LIBRARY_OUTPUT_DIRECTORY})
		set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_RELEASE 
				${ACTIVE_LIBRARY_OUTPUT_DIRECTORY})
	endif(WIN32)

	set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${ACTIVE_LIBRARY_OUTPUT_DIRECTORY})
	set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG ${ACTIVE_LIBRARY_OUTPUT_DIRECTORY})
	set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE ${ACTIVE_LIBRARY_OUTPUT_DIRECTORY})

	set(CMAKE_OUTPUT_DIRECTORIES_SET TRUE)
endif(NOT CMAKE_OUTPUT_DIRECTORIES_SET)

# Setup variable names for source folders.
set(ACTIVE_SOURCE_DIRECTORY
		${ACTIVE_PROJECT_ROOT_DIRECTORY}/src)
		
include_directories(${ACTIVE_SOURCE_DIRECTORY}
					${ACTIVE_EXT_DIRECTORY})

# All projects will link the same directories.
link_directories(${CMAKE_LIBRARY_OUTPUT_DIRECTORY}
					${CMAKE_LIBRARY_OUTPUT_DIRECTORY_DEBUG}
					${CMAKE_LIBRARY_OUTPUT_DIRECTORY_RELEASE}
					${CMAKE_ARCHIVE_OUTPUT_DIRECTORY}
					${CMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG}
					${CMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE})


# Configure library prefix and postfix values.
if(MSVC)
	set(LIBRARY_PREFIX "lib")
else()
	set(LIBRARY_PREFIX "")
endif(MSVC)
set(LIBRARY_DEBUG_POSTFIX "_d")

if(WIN32)
    set(ACTIVE_LIB_INSTALL_DEST lib)
    set(ACTIVE_BIN_INSTALL_DEST bin)
    set(ACTIVE_INCLUDE_INSTALL_DEST include/)
else()
    set(ACTIVE_LIB_INSTALL_DEST lib/active)
    set(ACTIVE_BIN_INSTALL_DEST lib/active)
    set(ACTIVE_INCLUDE_INSTALL_DEST include/active/)
endif(WIN32)

# Display results of setup.
if(DEBUG_ALL_ACTIVE_CMAKE AND NOT PRINTED_SETUP_VARIABLES)
	message(STATUS "SetupProject.cmake Variables")
	message(STATUS "\tProject Dir: ${ACTIVE_PROJECT_ROOT_DIRECTORY}")
	message(STATUS "\tRuntime Dir: ${ACTIVE_PROJECT_RUNTIME_OUTPUT_DIRECTORY}")
	message(STATUS "\tLibrary Dir: ${ACTIVE_LIBRARY_OUTPUT_DIRECTORY}")
	message(STATUS "\tCMake Dir: ${ACTIVE_CMAKE_DIRECTORY}")
	message(STATUS "\tSource Dir: ${ACTIVE_SOURCE_DIRECTORY}")
	message(STATUS "\tExternal Dir: ${ACTIVE_EXT_DIRECTORY}")
	message(STATUS "\tLibrary Prefix: ${LIBRARY_PREFIX}")
	message(STATUS "\tLibrary Postfix: ${LIBRARY_DEBUG_POSTFIX}")
	if(${CMAKE_BUILD_TYPE} MATCHES "Debug")
		message(STATUS "\tDEBUG build.")
	else()
		message(STATUS "\tRELEASE build.")
	endif()
	set(PRINTED_SETUP_VARIABLES TRUE)
endif(DEBUG_ALL_ACTIVE_CMAKE AND NOT PRINTED_SETUP_VARIABLES)

# Setup any additional compiler flags.
if(MSVC)
	# Setup to use UNICODE for MSVC, and disable warnings for
	# use of s_* standard C functions.
	add_definitions(-D_CRT_SECURE_NO_WARNINGS -DUNICODE -D_UNICODE -MP)
endif(MSVC)

if(UNIX AND NOT MINGW)
	# Ensure using gcc for C code
	set(CMAKE_C_COMPILER gcc)
	# Add fPIC to support 64 bit building issues
	add_definitions(-fPIC)
endif()
		
#############################################################
#		MACROS
#############################################################

# Macro to get the date as a string along with the
# Year, Month, and Day.
macro (get_date RESULT YEAR MONTH DAY)
	if (WIN32)
		execute_process(COMMAND 
			"cmd.exe" "/c" "date" "/T" OUTPUT_VARIABLE ${RESULT})
		string(REGEX REPLACE "....../../..(..).*" "\\1" ${YEAR} ${${RESULT}} )
		string(REGEX REPLACE "....(..)/../.*" "\\1" ${MONTH} ${${RESULT}} )
		string(REGEX REPLACE "....../(..)/.*" "\\1" ${DAY} ${${RESULT}} )
	elseif(UNIX)
        execute_process(COMMAND "date" "+%d" OUTPUT_VARIABLE ${DAY})
        execute_process(COMMAND "date" "+%y" OUTPUT_VARIABLE ${YEAR})
        execute_process(COMMAND "date" "+%m" OUTPUT_VARIABLE ${MONTH})
        string(REPLACE "\n" "" ${YEAR} ${${YEAR}} )
        string(REPLACE "\n" "" ${DAY} ${${DAY}} )
        string(REPLACE "\n" "" ${MONTH} ${${MONTH}} )
	else()
		message(STATUS "Date not implemented for OS")
		set(${RESULT} 000000)
		set(${YEAR} 000000)
		set(${MONTH} 000000)
		set(${DAY} 000000)
	endif (WIN32)
endmacro (get_date)

macro(configure_build_dependencies TARGET_NAME BUILD_DEPENDENCIES)

	if(DEBUG_${TARGET_NAME}_CMAKE OR
		DEBUG_ALL_ACTIVE_CMAKE)
		set(DEBUG_${TARGET_NAME}_CMAKE TRUE)
	endif()

	# Setup internal build dependencies.
	foreach(DEPENDENCY ${${BUILD_DEPENDENCIES}})
		# Print debug information for testing.
		if(DEBUG_${TARGET_NAME}_CMAKE)
			if(${DEPENDENCY}_DEPENDENCY)
				message(STATUS 
						"  Adding Build Dependency: ${DEPENDENCY}"
						" - ${${DEPENDENCY}_DEPENDENCY}")
			else()
				message(STATUS 
					"  Adding Build Dependency: ${DEPENDENCY}")
			endif()
			if(${DEPENDENCY}_INCLUDE_DIRS)
				message(STATUS "    Include Directories:")
				foreach(F ${${DEPENDENCY}_INCLUDE_DIRS})
					message(STATUS "    - ${F}")
				endforeach()
			endif()
			if(${DEPENDENCY}_LIBRARIES)
				message(STATUS "    Libraries:")
				foreach(F ${${DEPENDENCY}_LIBRARIES})
					message(STATUS "    - ${F}")
				endforeach()
			endif()
		endif(DEBUG_${TARGET_NAME}_CMAKE)
		# Set build dependency
		if(${DEPENDENCY}_DEPENDENCY)
			add_dependencies(${TARGET_NAME}
				${${DEPENDENCY}_DEPENDENCY})
		else()
			add_dependencies(${TARGET_NAME}
				${LIBRARY_PREFIX}${DEPENDENCY})
		endif()
		# Link libraries
		if(${DEPENDENCY}_LIBRARIES)
			target_link_libraries(${TARGET_NAME}
				${${DEPENDENCY}_LIBRARIES})
		else()
			target_link_libraries(${TARGET_NAME}
					debug ${LIBRARY_PREFIX}${DEPENDENCY}${LIBRARY_DEBUG_POSTFIX}
					optimized ${LIBRARY_PREFIX}${DEPENDENCY})
		endif()
		if(${DEPENDENCY}_CXX_FLAGS)
			add_definitions(${${DEPENDENCY}_CXX_FLAGS})
		endif()
		if(${DEPENDENCY}_LIBRARY_DIRS)
			if(DEBUG_${TARGET_NAME}_CMAKE)
				message(STATUS "  - ${${DEPENDENCY}_LIBRARY_DIRS}")
			endif(DEBUG_${TARGET_NAME}_CMAKE)
			link_directories(${${DEPENDENCY}_LIBRARY_DIRS})
		endif()
		if(${DEPENDENCY}_LIBRARY_DIR)
			if(DEBUG_${TARGET_NAME}_CMAKE)
				message(STATUS "  - ${${DEPENDENCY}_LIBRARY_DIR}")
			endif(DEBUG_${TARGET_NAME}_CMAKE)
			link_directories(${${DEPENDENCY}_LIBRARY_DIR})
		endif()
		if(${DEPENDENCY}_INCLUDE_DIRS)
			include_directories(${${DEPENDENCY}_INCLUDE_DIRS})
		endif()
	endforeach()
endmacro(configure_build_dependencies)

macro(configure_ext_dependencies TARGET_NAME LIBRARY_DEPENDENCIES)

	if(DEBUG_${TARGET_NAME}_CMAKE OR
		DEBUG_ALL_ACTIVE_CMAKE)
		set(DEBUG_${TARGET_NAME}_CMAKE TRUE)
	endif()
	
	if(DEBUG_${TARGET_NAME}_CMAKE)
			message(STATUS "  Adding Linker Dependencies:")
	endif(DEBUG_${TARGET_NAME}_CMAKE)
		
	foreach(DEPENDENCY ${${LIBRARY_DEPENDENCIES}})
		# Print debug information for testing.
		if(DEBUG_${TARGET_NAME}_CMAKE)
			message(STATUS "  Adding Library Dependency: ${DEPENDENCY}")
		endif(DEBUG_${TARGET_NAME}_CMAKE)
		if(${DEPENDENCY}_USE_FILE)
			include(${${${DEPENDENCY}_USE_FILE}})
		endif()
		if(${DEPENDENCY}_IS_DEPENDENCY)
			if(DEBUG_${TARGET_NAME}_CMAKE)
				message(STATUS "  - Settings as build dependency")
			endif(DEBUG_${TARGET_NAME}_CMAKE)
			add_dependencies(${TARGET_NAME}
					${${DEPENDENCY}_DEPENDENCY}})
		endif()
		if(${DEPENDENCY}_LIBRARIES)
			if(DEBUG_${TARGET_NAME}_CMAKE)
				foreach(L ${${DEPENDENCY}_LIBRARIES})
					message(STATUS "  - ${L}")
				endforeach()
			endif()
			target_link_libraries(${TARGET_NAME}
									${${DEPENDENCY}_LIBRARIES})
		else()
			# Link against a library file, instead of variables.
			target_link_libraries(${TARGET_NAME}
				${DEPENDENCY})
		endif()
		if(${DEPENDENCY}_LIBS)
			target_link_libraries(${TARGET_NAME}
									${${DEPENDENCY}_LIBS})
		endif()
	endforeach()
endmacro(configure_ext_dependencies)

macro(configure_dependency_paths TARGET_NAME LIBRARY_DEPENDENCIES)

	if(DEBUG_${TARGET_NAME}_CMAKE OR
		DEBUG_ALL_ACTIVE_CMAKE)
		set(DEBUG_${TARGET_NAME}_CMAKE TRUE)
	endif()
	
	foreach(DEPENDENCY ${${LIBRARY_DEPENDENCIES}})
		# Print debug information for testing.
		if(DEBUG_${TARGET_NAME}_CMAKE)
			message(STATUS "  Adding Include Paths for: ${DEPENDENCY}")
		endif(DEBUG_${TARGET_NAME}_CMAKE)
		if(${DEPENDENCY}_USE_FILE)
			include(${${${DEPENDENCY}_USE_FILE}})
		endif()

		if(${DEPENDENCY}_INCLUDE_DIRS)
			if(DEBUG_${TARGET_NAME}_CMAKE)
				foreach(L ${${DEPENDENCY}_INCLUDE_DIRS})
					message(STATUS "  - ${L}")
				endforeach()
			endif()
			foreach(L ${${DEPENDENCY}_INCLUDE_DIRS})
					include_directories(${L})
			endforeach()
			include_directories(${${DEPENDENCY}_INCLUDE_DIRS})
		endif()
		if(DEBUG_${TARGET_NAME}_CMAKE)
			message(STATUS "  Adding Linker Paths for: ${DEPENDENCY}")
		endif(DEBUG_${TARGET_NAME}_CMAKE)
		if(${DEPENDENCY}_LIB_DIR)
			foreach(D ${${DEPENDENCY}_LIB_DIR})
				if(DEBUG_${TARGET_NAME}_CMAKE)
					message(STATUS "  - Link Dir: ${D}")
				endif(DEBUG_${TARGET_NAME}_CMAKE)
				link_directories(${D})
			endforeach()
		endif()
		if(${DEPENDENCY}_LIBRARY_DIRS)
			foreach(D ${${DEPENDENCY}_LIBRARY_DIRS})
				if(DEBUG_${TARGET_NAME}_CMAKE)
					message(STATUS "  - Link Dir: ${D}")
				endif(DEBUG_${TARGET_NAME}_CMAKE)
				link_directories(${D})
			endforeach()
		endif()
		if(${DEPENDENCY}_LIBRARY_DIR)
			foreach(D ${${DEPENDENCY}_LIBRARY_DIR})
				if(DEBUG_${TARGET_NAME}_CMAKE)
					message(STATUS "  - Link Dir: ${D}")
				endif(DEBUG_${TARGET_NAME}_CMAKE)
				link_directories(${D})
			endforeach()
		endif()
		if(${DEPENDENCY}_CXX_FLAGS)
			add_definitions(${${DEPENDENCY}_CXX_FLAGS})
		endif()
	endforeach()
endmacro(configure_dependency_paths)

macro(copy_lib_project_variables ORIGINAL_NAME NEW_NAME)
	
	if(DEBUG_${ORIGINAL_NAME}_CMAKE OR
			DEBUG_ALL_ACTIVE_CMAKE)
		set(PRINT_COPY_${ORIGINAL_NAME}_VARIABLES TRUE)
		# Also copy the debug flag...
		set(DEBUG_${NEW_NAME}_CMAKE TRUE)
	endif()
	
	foreach(VTYPE ${ACTIVE_LIB_VARIABLE_TYPES})
		if(${ORIGINAL_NAME}_${VTYPE})
		set(${NEW_NAME}_${VTYPE}
				${${ORIGINAL_NAME}_${VTYPE}})
		# if(DEBUG_${NEW_NAME}_CMAKE)
			# message(STATUS "  Copying ${ORIGINAL_NAME}_${VTYPE} to ${NEW_NAME}_${VTYPE}")
			# message(STATUS "  - ${${NEW_NAME}_${VTYPE}}")
		# endif()
		endif()
	endforeach()
endmacro(copy_lib_project_variables)

macro(set_lib_project_variables_to_parent_scope TARGET_NAME)
	foreach(VTYPE ${ACTIVE_LIB_VARIABLE_TYPES})
		if(${TARGET_NAME}_${VTYPE})
			set(${TARGET_NAME}_${VTYPE}
					${${TARGET_NAME}_${VTYPE}}
					PARENT_SCOPE)
		endif()
	endforeach()
endmacro(set_lib_project_variables_to_parent_scope)

macro(print_lib_project_variables PROJECT_NAME)

	if(DEBUG_${PROJECT_NAME}_CMAKE OR
			DEBUG_ALL_ACTIVE_CMAKE)
			set(DEBUG_${PROJECT_NAME}_CMAKE TRUE)
	endif()

	if(DEBUG_${PROJECT_NAME}_CMAKE)
		foreach(VTYPE ${ACTIVE_LIB_VARIABLE_TYPES})
			if(${PROJECT_NAME}_${VTYPE})
				message(STATUS "  ${PROJECT_NAME}_${VTYPE}: ${${PROJECT_NAME}_${VTYPE}}")
			endif()
		endforeach()
	endif()
endmacro(print_lib_project_variables)
