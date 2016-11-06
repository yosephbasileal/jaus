################################################################################
#
# File: CreateLibrary.cmake
# Created by: Daniel Barber, Ph.D.
# Description: This file contains a macro to create a library project
# for the current software build environment.
#
# You must include SetupProject.cmake before including this file.
#
################################################################################

# Creates a library assuming pre-defined folder
# structures. Also assumes an inclusion of SetupProject.cmake
# Prior to inclusion of this file. 
# \param LIBRARY_NAME Name of the library, must match a library
#					  source folder name (e.g. JAUS)
# \param LIBRARY_SRC_SUB_DIR Any subfolder within the main source folder 
#							containing sub-library code. For example (jaus/core)
# \param LIBRARY_SRC_GROUPS Any subfolder names within the library
#							folder to pull other source code from. This 
#							folder names are also used to generate subfolders
#							in Visual Studio projects.
# \param LIBRARY_BUILD_DEPENDENCIES A list of any other projects/libraries
#									that are part of this code base, already
#								    added to your project that must build
#									before this library
# \param LIBRARY_DEPENDENCIES	Additional library link dependencies. This can
#								be the name of a package, like Boost or
#								wxWidgets
# \param LIBRARY_TYPE	Library type SHARED or STATIC
# \param LIBRARY_PROJECT_NAME_RESULTS Output name of actual project file
#									  created.
macro(configure_library
		LIBRARY_NAME
		LIBRARY_SRC_SUB_DIR
		LIBRARY_SRC_GROUPS
		LIBRARY_BUILD_DEPENDENCIES
		LIBRARY_DEPENDENCIES
		LIBRARY_TYPE
		LIBRARY_PROJECT_NAME_RESULTS)
		
		
		option(DEBUG_${LIBRARY_NAME}_CMAKE "Print debug messages in CMakeLists.txt files." FALSE)
		
		if(DEBUG_${LIBRARY_NAME}_CMAKE OR
			DEBUG_ALL_ACTIVE_CMAKE)
			set(DEBUG_${LIBRARY_NAME}_CMAKE TRUE)
		endif()
		
		# Print debug information for testing.
		if(DEBUG_${LIBRARY_NAME}_CMAKE )
			message(STATUS "\n************************************************")
			message(STATUS "Configuring Library [${LIBRARY_NAME}${LIBRARY_SRC_SUB_DIR}]")
		endif(DEBUG_${LIBRARY_NAME}_CMAKE )
		
		if(${ARGV7} MATCHES "EXCLUDE_CPACK")
			set(${LIBRARY_NAME}_EXCLUDE_CPACK TRUE)
			if(DEBUG_${LIBRARY_NAME}_CMAKE )
				message("Excluding  [${LIBRARY_NAME}${LIBRARY_SRC_SUB_DIR}] from Installers")
			endif()
		endif()
		
		# Convert case, since matching folder name is
		# in lowercase letters.
		string(TOLOWER ${LIBRARY_NAME} LIB_SRC_FOLDER)
		set(MAIN_SOURCE_DIR ${ACTIVE_SOURCE_DIRECTORY}/${LIB_SRC_FOLDER})

		if(NOT ${LIBRARY_SRC_SUB_DIR} STREQUAL "")
			# Convert case, since matching folder name is
			# in lowercase letters.
			string(TOLOWER ${LIBRARY_SRC_SUB_DIR} LIB_SRC_SUB_FOLDER)
			set(MAIN_SOURCE_DIR ${MAIN_SOURCE_DIR}/${LIB_SRC_SUB_FOLDER})
		endif()

		if(DEBUG_${LIBRARY_NAME}_CMAKE  AND DEBUG_ACTIVE_CMAKE_SHOW_SRC_FILES)
			message(STATUS "  Searching folder:  ${MAIN_SOURCE_DIR}")
		endif()
		##################################################
		# Get the header/source files from main folder
		file(GLOB LIBRARY_C_HEADERS
				"${MAIN_SOURCE_DIR}/*.h")
		file(GLOB LIBRARY_CPP_HEADERS
				"${MAIN_SOURCE_DIR}/*.hpp")
		file(GLOB LIBRARY_C_SOURCES
				"${MAIN_SOURCE_DIR}/*.c")
		file(GLOB LIBRARY_CPP_SOURCES
				"${MAIN_SOURCE_DIR}/*.cpp")
		# Combine C/CPP files together.
		set(LIBRARY_HEADERS 
				${LIBRARY_C_HEADERS}
				${LIBRARY_CPP_HEADERS})
		set(LIBRARY_SOURCES 
				${LIBRARY_C_SOURCES}
				${LIBRARY_CPP_SOURCES})		
		# Now glob all subfolders specified.
		foreach(GRP ${${LIBRARY_SRC_GROUPS}})
			string(TOLOWER ${GRP} GRP_SUB_DIR)
			set(MAIN_SOURCE_SUB_DIR ${MAIN_SOURCE_DIR}/${GRP_SUB_DIR})
			if(DEBUG_${LIBRARY_NAME}_CMAKE  AND DEBUG_ACTIVE_CMAKE_SHOW_SRC_FILES)
				message(STATUS "  Searching folder:  ${MAIN_SOURCE_SUB_DIR}")
			endif()
			# Get the header/source files
			file(GLOB_RECURSE LIBRARY_C_HEADERS
					${MAIN_SOURCE_SUB_DIR}/*.h)
			file(GLOB_RECURSE LIBRARY_CPP_HEADERS
					${MAIN_SOURCE_SUB_DIR}/*.hpp)
			file(GLOB_RECURSE LIBRARY_C_SOURCES
					${MAIN_SOURCE_SUB_DIR}/*.c)
			file(GLOB_RECURSE LIBRARY_CPP_SOURCES
					${MAIN_SOURCE_SUB_DIR}/*.cpp)
			# Combine C/CPP files together.
			set(LIBRARY_HEADERS 
					${LIBRARY_HEADERS}
					${LIBRARY_C_HEADERS}
					${LIBRARY_CPP_HEADERS})
			set(LIBRARY_SOURCES 
					${LIBRARY_SOURCES}
					${LIBRARY_C_SOURCES}
					${LIBRARY_CPP_SOURCES})
		endforeach()

		# Print debug information for testing.
		if(DEBUG_${LIBRARY_NAME}_CMAKE  AND DEBUG_ACTIVE_CMAKE_SHOW_SRC_FILES)
			message(STATUS "  Header Files:")
			foreach(F ${LIBRARY_HEADERS})
				message(STATUS "    - ${F}")
			endforeach()
			message(STATUS "  Source Files:")
			foreach(F ${LIBRARY_SOURCES})
				message(STATUS "    - ${F}")
			endforeach()
		endif()
		
		##################################################
		# Setup the library project name.
		set(FULL_NAME
				${LIBRARY_NAME}${LIBRARY_SRC_SUB_DIR})
		set(LIBRARY_PROJECT_NAME
				${LIBRARY_PREFIX}${LIBRARY_NAME}${LIBRARY_SRC_SUB_DIR})

		#message("${${LIBRARY_NAME}_DEPENDENCY}")	 
		if(DEBUG_${LIBRARY_NAME}_CMAKE )
			message(STATUS "  Project name: ${LIBRARY_PROJECT_NAME}")
		endif(DEBUG_${LIBRARY_NAME}_CMAKE )
		
		if(${LIBRARY_DEPENDENCIES})
			# Setup other dependencies
			configure_dependency_paths(${LIBRARY_NAME}
				${LIBRARY_DEPENDENCIES})
		else()
			if(DEBUG_${LIBRARY_NAME}_CMAKE )
				message(STATUS "  No Dependencies")
			endif(DEBUG_${LIBRARY_NAME}_CMAKE )
		endif()
		
		##################################################
		# Create the library project.
		add_library(${LIBRARY_PROJECT_NAME}
					${LIBRARY_TYPE}
					${LIBRARY_HEADERS}
					${LIBRARY_SOURCES})
		set(${LIBRARY_PROJECT_NAME_RESULTS}	${LIBRARY_PROJECT_NAME})
		
		if(DEBUG_${LIBRARY_NAME}_CMAKE)
			set(DEBUG_${LIBRARY_PROJECT_NAME}_CMAKE TRUE)
		endif()
		
		# Configure debug postfix for library
		set_target_properties(${LIBRARY_PROJECT_NAME}
			PROPERTIES DEBUG_POSTFIX
				${LIBRARY_DEBUG_POSTFIX})
		# Link against a library file.
		target_link_libraries(${LIBRARY_PROJECT_NAME}
			${CMAKE_REQUIRED_LIBRARIES})
		
		if(${LIBRARY_BUILD_DEPENDENCIES})
			# Setup internal build dependencies
			configure_build_dependencies(${LIBRARY_PROJECT_NAME}
				${LIBRARY_BUILD_DEPENDENCIES})
		else()
			if(DEBUG_${LIBRARY_NAME}_CMAKE )
				message(STATUS "  No Build Dependencies")
			endif(DEBUG_${LIBRARY_NAME}_CMAKE )
		endif()
		if(${LIBRARY_DEPENDENCIES})
			# Setup other dependencies
			configure_ext_dependencies(${LIBRARY_PROJECT_NAME}
				${LIBRARY_DEPENDENCIES})
		else()
			if(DEBUG_${LIBRARY_NAME}_CMAKE )
				message(STATUS "  No Linker Dependencies")
			endif(DEBUG_${LIBRARY_NAME}_CMAKE )
		endif()
		
		##################################################
		# Make Source Groups, this makes sub folders in Visual Studio
		foreach(GRP ${${LIBRARY_SRC_GROUPS}})
			file(GLOB_RECURSE HEADER_HPP_GROUP 
				${MAIN_SOURCE_DIR}/${GRP}/*.hpp)
			file(GLOB_RECURSE SRC_CPP_GROUP 
				${MAIN_SOURCE_DIR}/${GRP}/*.cpp)
			file(GLOB_RECURSE HEADER_GROUP 
				${MAIN_SOURCE_DIR}/${GRP}/*.h)
			file(GLOB_RECURSE SRC_GROUP 
				${MAIN_SOURCE_DIR}/${GRP}/*.c)
			
			set(ALL_HEADERS ${HEADER_HPP_GROUP} ${HEADER_GROUP})
			set(ALL_SOURCES ${SRC_CPP_GROUP} ${SRC_GROUP})
			source_group("Header Files\\${GRP}" 
				FILES ${ALL_HEADERS})
			source_group("Source Files\\${GRP}" 
				FILES ${ALL_SOURCES})
		endforeach(GRP)

		# Visual Studio Solution folders.
		if(MSVC_SOLUTION_FOLDERS)
			set_property(TARGET ${LIBRARY_PROJECT_NAME} 
				PROPERTY FOLDER "Libraries")
		endif(MSVC_SOLUTION_FOLDERS)
						
		# OS/Compiler specific properties
		if(NOT WIN32)
			set_target_properties(${LIBRARY_PROJECT_NAME}
				PROPERTIES LINK_FLAGS
				-rdynamic)
			# If release build, add optimizations
			if(NOT CMAKE_BUILD_TYPE)
				set_target_properties(${LIBRARY_PROJECT_NAME} 
					PROPERTIES COMPILE_FLAGS "-O3")
			endif()
		elseif(${LIBRARY_TYPE} MATCHES SHARED)
			set(EXPORT_NAME ${LIBRARY_NAME})
			if(NOT ${LIBRARY_SRC_SUB_DIR} STREQUAL "")
				set(EXPORT_NAME "${LIBRARY_NAME}_${LIBRARY_SRC_SUB_DIR}")
			endif()
			string(TOUPPER ${EXPORT_NAME} LIBRARY_NAME_CAPS)
			# set_target_properties(${LIBRARY_PROJECT_NAME}
				# PROPERTIES COMPILE_FLAGS
				# -D${LIBRARY_NAME_CAPS}_DLL_EXPORT)
			set(EXPORT_DEF
					-D${LIBRARY_NAME_CAPS}_DLL_EXPORT)
			# set(${FULL_NAME}_CXX_FLAGS
					# ${${FULL_NAME}_CXX_FLAGS}
					# -D${LIBRARY_NAME_CAPS}_DLL_EXPORT)
			if(DEBUG_${LIBRARY_NAME}_CMAKE )
				message(STATUS "  DLL_EXPORT: ${EXPORT_DEF}")
			endif(DEBUG_${LIBRARY_NAME}_CMAKE )
		endif(NOT WIN32)
		
		# Clear the variable
		set(P_FLAGS)
		
		if(${FULL_NAME}_CXX_FLAGS OR EXPORT_DEF)
			if(DEBUG_${LIBRARY_NAME}_CMAKE )
				message(STATUS "  Comiler flags:")
			endif(DEBUG_${LIBRARY_NAME}_CMAKE )
			foreach(F ${${FULL_NAME}_CXX_FLAGS})
			if(DEBUG_${LIBRARY_NAME}_CMAKE )
					message(STATUS "     ${F}")
				endif(DEBUG_${LIBRARY_NAME}_CMAKE )
				set(P_FLAGS ${P_FLAGS} ${F})
			endforeach()
			if(EXPORT_DEF)
				set(P_FLAGS ${P_FLAGS} ${EXPORT_DEF})
			endif()
			set_target_properties(${LIBRARY_PROJECT_NAME} 
					PROPERTIES COMPILE_FLAGS 
					${P_FLAGS})
		endif()
		
		##################################################
		# Configure library project variables at 
		# both the parent and local subdirectory
		# scope level.
		
		# Dependency exists flag, not really
		# used much
		set(${LIBRARY_NAME}_IS_DEPENDENCY 
				TRUE PARENT_SCOPE)
		set(${LIBRARY_NAME}_IS_DEPENDENCY 
				TRUE)
		
		# Create project build dependency list.
		set(${LIBRARY_NAME}_DEPENDENCY 
				${${LIBRARY_NAME}_DEPENDENCY}
				${LIBRARY_PROJECT_NAME} 
					PARENT_SCOPE)
		set(${LIBRARY_NAME}_DEPENDENCY 
				${${LIBRARY_NAME}_DEPENDENCY}
				${LIBRARY_PROJECT_NAME})
		set(${LIBRARY_NAME}_DEPENDENCIES
				${${LIBRARY_NAME}_DEPENDENCY} 
					PARENT_SCOPE)
		set(${LIBRARY_NAME}_DEPENDENCIES
				${${LIBRARY_NAME}_DEPENDENCY})
				
		# Include directories
		set(${LIBRARY_NAME}_INCLUDE_DIRS
				${ACTIVE_SOURCE_DIRECTORY} 
				PARENT_SCOPE)
		set(${LIBRARY_NAME}_INCLUDE_DIRS
				${ACTIVE_SOURCE_DIRECTORY})
				
		# Libraries output by project
		set(${LIBRARY_NAME}_LIBRARIES
				${${LIBRARY_NAME}_LIBRARIES}
				debug ${LIBRARY_PROJECT_NAME}_d
				optimized ${LIBRARY_PROJECT_NAME} PARENT_SCOPE)
		set(${LIBRARY_NAME}_LIBRARIES
				${${LIBRARY_NAME}_LIBRARIES}
				debug ${LIBRARY_PROJECT_NAME}_d
				optimized ${LIBRARY_PROJECT_NAME})		
				
		set(${LIBRARY_NAME}_LIBRARY_DIR
				 ${ACTIVE_LIBRARY_OUTPUT_DIRECTORY} PARENT_SCOPE)	
		set(${LIBRARY_NAME}_LIBRARY_DIR
				 ${ACTIVE_LIBRARY_OUTPUT_DIRECTORY})
		
		print_lib_project_variables(${LIBRARY_NAME})
		
		if(NOT ${LIBRARY_NAME}_EXCLUDE_CPACK)
			##################################################
			# Software installation
			install(TARGETS ${LIBRARY_PROJECT_NAME}
					DESTINATION ${ACTIVE_BIN_INSTALL_DEST}
					ARCHIVE DESTINATION ${ACTIVE_LIB_INSTALL_DEST}
					COMPONENT SDK)
			install(TARGETS ${LIBRARY_PROJECT_NAME}
					DESTINATION ${ACTIVE_BIN_INSTALL_DEST}
					RUNTIME DESTINATION ${ACTIVE_BIN_INSTALL_DEST}
					COMPONENT SDK)
			install(TARGETS ${LIBRARY_PROJECT_NAME}
					DESTINATION ${ACTIVE_BIN_INSTALL_DEST}
					LIBRARY DESTINATION ${ACTIVE_BIN_INSTALL_DEST}
					COMPONENT SDK)
			install(TARGETS ${LIBRARY_PROJECT_NAME}
					DESTINATION ${ACTIVE_BIN_INSTALL_DEST}
					RUNTIME DESTINATION ${ACTIVE_BIN_INSTALL_DEST}
					COMPONENT Binaries)
			install(TARGETS ${LIBRARY_PROJECT_NAME}
					DESTINATION ${ACTIVE_BIN_INSTALL_DEST}
					LIBRARY DESTINATION ${ACTIVE_BIN_INSTALL_DEST}
					COMPONENT Binaries)
		endif()
		
endmacro(configure_library)

macro(configure_ext_library
		LIBRARY_NAME
		LIBRARY_SOURCE_FILES
		LIBRARY_DEPENDENCIES
		LIBRARY_TYPE
		LIBRARY_PROJECT_NAME_RESULTS)
		
		option(DEBUG_${LIBRARY_NAME}_CMAKE "Print debug messages in CMakeLists.txt files." FALSE)
		
		if(DEBUG_${LIBRARY_NAME}_CMAKE OR
			DEBUG_ALL_ACTIVE_CMAKE)
			set(DEBUG_${LIBRARY_NAME}_CMAKE TRUE)
		endif()
		
		# Print debug information for testing.
		if(DEBUG_${LIBRARY_NAME}_CMAKE)
			message(STATUS "Configuring External Library [${LIBRARY_NAME}]")
		endif(DEBUG_${LIBRARY_NAME}_CMAKE)
		
		if(${ARGV6} MATCHES "EXCLUDE_CPACK")
			set(${LIBRARY_NAME}_EXCLUDE_CPACK TRUE)
			if(DEBUG_${LIBRARY_NAME}_CMAKE)
				message("Excluding  [${LIBRARY_NAME}${LIBRARY_SRC_SUB_DIR}] from Installers")
			endif()
		endif()
		
		# Print debug information for testing.
		if(DEBUG_${LIBRARY_NAME}_CMAKE AND DEBUG_ACTIVE_CMAKE_SHOW_SRC_FILES)
			message(STATUS "  Source Files:")
			foreach(F ${${LIBRARY_SOURCE_FILES}})
				message(STATUS "    - ${F}")
			endforeach()
		endif()
		
		# Setup the library project name.
		set(LIBRARY_PROJECT_NAME
				${${LIBRARY_PREFIX}}${LIBRARY_NAME})
		
		if(DEBUG_${LIBRARY_NAME}_CMAKE)
			message(STATUS "  Project name: ${LIBRARY_PROJECT_NAME}")
		endif(DEBUG_${LIBRARY_NAME}_CMAKE)
		
		if(${LIBRARY_DEPENDENCIES})
			# Setup other dependencies
			configure_dependency_paths(${LIBRARY_NAME}
				${LIBRARY_DEPENDENCIES})
		endif()
		
		# Create the library project.
		add_library(${LIBRARY_PROJECT_NAME}
					${LIBRARY_TYPE}
					${${LIBRARY_SOURCE_FILES}})
		set(${LIBRARY_PROJECT_NAME_RESULTS}	${LIBRARY_PROJECT_NAME})	

		if(DEBUG_${LIBRARY_NAME}_CMAKE)
			set(DEBUG_${LIBRARY_PROJECT_NAME}_CMAKE TRUE)
		endif()		
		# Configure debug postfix for library
		set_target_properties(${LIBRARY_PROJECT_NAME}
			PROPERTIES DEBUG_POSTFIX
				${LIBRARY_DEBUG_POSTFIX})
		# Link against common system libraries
		target_link_libraries(${LIBRARY_PROJECT_NAME}
			${CMAKE_REQUIRED_LIBRARIES})
			
		if(${LIBRARY_DEPENDENCIES})
			# Setup other dependencies
			configure_ext_dependencies(${LIBRARY_PROJECT_NAME}
				${LIBRARY_DEPENDENCIES})
		endif()
		# Visual Studio Solution folders.
		if(MSVC_SOLUTION_FOLDERS)
			set_property(TARGET ${LIBRARY_PROJECT_NAME} 
				PROPERTY FOLDER "3rd Party Libraries")
		endif(MSVC_SOLUTION_FOLDERS)
						
		# OS/Compiler specific properties
		if(NOT WIN32)
			set_target_properties(${LIBRARY_PROJECT_NAME}
				PROPERTIES LINK_FLAGS
				-rdynamic)
		elseif(${LIBRARY_TYPE} MATCHES SHARED)
			string(TOUPPER ${LIBRARY_NAME} LIBRARY_NAME_CAPS)
			set_target_properties(${LIBRARY_PROJECT_NAME}
				PROPERTIES COMPILE_FLAGS
				-DACTIVE_${LIBRARY_NAME_CAPS}_DLL_EXPORT)
		endif(NOT WIN32)
		
		##################################################
		# Configure library project variables at 
		# both the parent and local subdirectory
		# scope level.
		
		# Dependency exists flag, not really
		# used much
		set(${LIBRARY_NAME}_IS_DEPENDENCY 
				TRUE PARENT_SCOPE)
		set(${LIBRARY_NAME}_IS_DEPENDENCY 
				TRUE)
		
		# Create project build dependency list.
		set(${LIBRARY_NAME}_DEPENDENCY 
				${${LIBRARY_NAME}_DEPENDENCY}
				${LIBRARY_PROJECT_NAME} 
					PARENT_SCOPE)
		set(${LIBRARY_NAME}_DEPENDENCY 
				${${LIBRARY_NAME}_DEPENDENCY}
				${LIBRARY_PROJECT_NAME})
		set(${LIBRARY_NAME}_DEPENDENCIES
				${${LIBRARY_NAME}_DEPENDENCY} 
					PARENT_SCOPE)
		set(${LIBRARY_NAME}_DEPENDENCIES
				${${LIBRARY_NAME}_DEPENDENCY})
				
		# Include directories
		set(${LIBRARY_NAME}_INCLUDE_DIRS
				${${LIBRARY_NAME}_INCLUDE_DIRS}
				${ACTIVE_EXT_DIRECTORY}
				${ACTIVE_EXT_DIRECTORY}/${LIBRARY_NAME}
					PARENT_SCOPE)
		set(${LIBRARY_NAME}_INCLUDE_DIRS
				${${LIBRARY_NAME}_INCLUDE_DIRS}
				${ACTIVE_EXT_DIRECTORY}
				${ACTIVE_EXT_DIRECTORY}/${LIBRARY_NAME})
				
		# Libraries output by project
		set(${LIBRARY_NAME}_LIBRARIES
				${${LIBRARY_NAME}_LIBRARIES}
				debug ${LIBRARY_PROJECT_NAME}_d
				optimized ${LIBRARY_PROJECT_NAME} PARENT_SCOPE)
		set(${LIBRARY_NAME}_LIBRARIES
				${${LIBRARY_NAME}_LIBRARIES}
				debug ${LIBRARY_PROJECT_NAME}_d
				optimized ${LIBRARY_PROJECT_NAME})		
				
		set(${LIBRARY_NAME}_LIBRARY_DIR
				 ${ACTIVE_LIBRARY_OUTPUT_DIRECTORY} PARENT_SCOPE)	
		set(${LIBRARY_NAME}_LIBRARY_DIR
				 ${ACTIVE_LIBRARY_OUTPUT_DIRECTORY})
				 
		print_lib_project_variables(${LIBRARY_NAME})
		
		### Software Installations
		if(${LIBRARY_NAME}_EXCLUDE_CPACK)
		
		endif()
		
endmacro(configure_ext_library)

