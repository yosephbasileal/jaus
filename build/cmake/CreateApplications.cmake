################################################################################
#
# File: CreateApplications.cmake
# Created by: Daniel Barber, Ph.D.
# Description: This file contains a macros to create a programs that
# uses wxWidgets, etc.
#
# You must include SetupProject.cmake before including this file.
#
################################################################################

# Creates example programs for a given library project.
# structures. Also assumes an inclusion of SetupProject.cmake
# Prior to inclusion of this file. 
# \param LIBRARY_NAME Name of the library, must match a library
#					  source folder name (e.g. JAUS)
# \param LIBRARY_BUILD_DEPENDENCIES A list of any other projects/libraries
#									that are part of this code base, already
#								    added to your project that must build
#									before this library
# \param LIBRARY_DEPENDENCIES	Additional library link dependencies. This can
#								be the name of a package, like Boost or
#								wxWidgets
macro(configure_examples
		LIBRARY_NAME
		EXAMPLE_FOLDER
		LIBRARY_BUILD_DEPENDENCIES
		LIBRARY_DEPENDENCIES)
		
		set(APP_PREFIX "")
		if(${ARGC} GREATER 4)
			set(APP_PREFIX ${ARGV4})
		endif()
		
		set(APP_V_FOLDER "${LIBRARY_NAME} Examples")
		if(${ARGC} GREATER 5)
			set(APP_V_FOLDER ${ARGV5})
		endif()
		
		if(DEBUG_${LIBRARY_NAME}_CMAKE OR
			DEBUG_ALL_ACTIVE_CMAKE)
			set(DEBUG_${LIBRARY_NAME}_CMAKE TRUE)
		endif()
		
		# Print debug information for testing.
		if(DEBUG_${LIBRARY_NAME}_CMAKE)
			message(STATUS "  Configuring Examples for [${LIBRARY_NAME}]")
		endif(DEBUG_${LIBRARY_NAME}_CMAKE)
		
		string(TOLOWER ${LIBRARY_NAME} LIBRARY_NAME_LC)
		# Get the source files.
		file(GLOB_RECURSE LIBRARY_CPP_SOURCES
				${ACTIVE_SOURCE_DIRECTORY}/${LIBRARY_NAME_LC}/${EXAMPLE_FOLDER}/*.cpp)
				
		# Combine C/CPP files together.
		set(EXAMPLE_SOURCES ${LIBRARY_CPP_SOURCES})
		
		foreach(F ${EXAMPLE_SOURCES})
		
			# Get program name from the file path
			string(REGEX REPLACE "/" ";" FILE_TOKENS ${F})
			list(REVERSE FILE_TOKENS)
			list(GET FILE_TOKENS 0 EXAMPLE_NAME)
			string(REGEX REPLACE ".cpp" "" EXAMPLE_NAME ${EXAMPLE_NAME})
			
			# Add prefix to app name
			set(EXAMPLE_NAME ${APP_PREFIX}${EXAMPLE_NAME})
			# Print debug information for testing.
			if(DEBUG_${LIBRARY_NAME}_CMAKE)
				message(STATUS "  Example: - ${EXAMPLE_NAME}")
			endif(DEBUG_${LIBRARY_NAME}_CMAKE)

			if(${LIBRARY_DEPENDENCIES})
				# Setup other dependencies
				configure_dependency_paths(${F}
					${LIBRARY_DEPENDENCIES})
			endif()
			
			if(MSVC 
					AND EXISTS 
						${ACTIVE_RESOURCE_DIRECTORY}/ActiveIcon.rc
					AND EXISTS 
						${ACTIVE_PROJECT_RUNTIME_OUTPUT_DIRECTORY}/icons/ActiveTriangle_256x256.ico)
				# Create the library project.
				add_executable(${EXAMPLE_NAME}
								${F}
								${ACTIVE_RESOURCE_DIRECTORY}/ActiveIcon.rc)
			else()
				# Create the library project.
				add_executable(${EXAMPLE_NAME}
								${F})
			endif()		
			# Configure debug postfix for library
			set_target_properties(${EXAMPLE_NAME}
				PROPERTIES DEBUG_POSTFIX
					${LIBRARY_DEBUG_POSTFIX})
			# Link against common system libraries
			target_link_libraries(${EXAMPLE_NAME}
				${CMAKE_REQUIRED_LIBRARIES})
				
			if(${LIBRARY_BUILD_DEPENDENCIES})
				# Setup internal build dependencies
				configure_build_dependencies(${EXAMPLE_NAME}
					${LIBRARY_BUILD_DEPENDENCIES})
			endif()
			if(${LIBRARY_DEPENDENCIES})
				# Setup other dependencies
				configure_ext_dependencies(${EXAMPLE_NAME}
					${LIBRARY_DEPENDENCIES})
			endif()
			
			# Visual Studio Solution folders.
			if(MSVC_SOLUTION_FOLDERS)
				set_property(TARGET ${EXAMPLE_NAME} 
					PROPERTY FOLDER ${APP_V_FOLDER})
			endif(MSVC_SOLUTION_FOLDERS)
			
			if(NOT ${LIBRARY_NAME}_EXCLUDE_CPACK)
	
			endif()
		endforeach()
		
endmacro(configure_examples)

# Creates programs for a given library project.
# structures. Also assumes an inclusion of SetupProject.cmake
# Prior to inclusion of this file. 
# \param APPLICATION_NAME Name of the application)
# \param SOURCE_FILES The sources files for the application.
# \param BUILD_DEPENDENCIES A list of any other projects/libraries
#									that are part of this code base, already
#								    added to your project that must build
#									before this library
# \param LIBRARY_DEPENDENCIES	Additional library link dependencies. This can
#								be the name of a package, like Boost.
macro(create_console_app
		APPLICATION_NAME
		SOURCE_FILES
		BUILD_DEPENDENCIES
		LIBRARY_DEPENDENCIES)
	
	option(DEBUG_${APPLICATION_NAME}_CMAKE "Print debug messages in CMakeLists.txt files." FALSE)
		
	if(DEBUG_${APPLICATION_NAME}_CMAKE OR
		DEBUG_ALL_ACTIVE_CMAKE)
		set(DEBUG_${APPLICATION_NAME}_CMAKE TRUE)
	endif()
		
	# Print debug information for testing.
	if(DEBUG_${APPLICATION_NAME}_CMAKE)
		message(STATUS "\n  Configuring Console App [${APPLICATION_NAME}]")
		if(DEBUG_ACTIVE_CMAKE_SHOW_SRC_FILES)
			message(STATUS "  Source Files:")
			foreach(F ${${SOURCE_FILES}})
				message(STATUS "  - ${F}")
			endforeach()
		endif()
	endif(DEBUG_${APPLICATION_NAME}_CMAKE)	
	
	if(${ARGV5} MATCHES "EXCLUDE_CPACK")
		set(${APPLICATION_NAME}_EXCLUDE_CPACK TRUE)
		if(DEBUG_${APPLICATION_NAME}_CMAKE)
			message("Excluding  [${APPLICATION_NAME}] from Installers")
		endif()
	elseif(${ARGC} GREATER 5)
		set(SOLUTION_FOLDER_NAME ${ARGV5})
	endif()
	add_executable(${APPLICATION_NAME}
			${${SOURCE_FILES}}
			)
	# Configure debug postfix for library
	set_target_properties(${APPLICATION_NAME}
		PROPERTIES DEBUG_POSTFIX
			${LIBRARY_DEBUG_POSTFIX})
			
	# Link against common system libraries
	target_link_libraries(${APPLICATION_NAME}
		${CMAKE_REQUIRED_LIBRARIES})
	
	if(${BUILD_DEPENDENCIES})
		# Setup internal build dependencies
		configure_build_dependencies(${APPLICATION_NAME}
			${BUILD_DEPENDENCIES})
	endif()
	if(${LIBRARY_DEPENDENCIES})
		# Setup other dependencies
		configure_ext_dependencies(${APPLICATION_NAME}
			${LIBRARY_DEPENDENCIES})
	endif()
	
	if(${LIBRARY_DEPENDENCIES})
		# Setup other dependencies
		configure_dependency_paths(${APPLICATION_NAME}
			${LIBRARY_DEPENDENCIES})
	endif()

	# Visual Studio Solution folders.
	if(MSVC_SOLUTION_FOLDERS)
		if(SOLUTION_FOLDER_NAME)
			set_property(TARGET ${APPLICATION_NAME} 
				PROPERTY FOLDER "${SOLUTION_FOLDER_NAME}")
		else()
			set_property(TARGET ${APPLICATION_NAME} 
				PROPERTY FOLDER "Console Applications")
		endif()
	endif(MSVC_SOLUTION_FOLDERS)
	
	if(NOT ${APPLICATION_NAME}_EXCLUDE_CPACK)
		install(TARGETS ${APPLICATION_NAME}
				DESTINATION bin
				ARCHIVE DESTINATION lib
				COMPONENT Binaries)
		install(TARGETS ${APPLICATION_NAME}
				DESTINATION bin
				RUNTIME DESTINATION bin
				COMPONENT Binaries)
		install(TARGETS ${APPLICATION_NAME}
				DESTINATION bin
				LIBRARY DESTINATION bin
				COMPONENT Binaries)
	endif()
	
endmacro(create_console_app)

# Creates wxWidgets programs for a given library project.
# structures. Also assumes an inclusion of SetupProject.cmake
# Prior to inclusion of this file. 
# \param APPLICATION_NAME Name of the application)
# \param SOURCE_FILES The sources files for the application.
# \param BUILD_DEPENDENCIES A list of any other projects/libraries
#									that are part of this code base, already
#								    added to your project that must build
#									before this library
# \param LIBRARY_DEPENDENCIES	Additional library link dependencies. This can
#								be the name of a package, like Boost or
#								wxWidgets
macro(create_wx_app
		APPLICATION_NAME
		SOURCE_FILES
		BUILD_DEPENDENCIES
		LIBRARY_DEPENDENCIES)
		
	option(DEBUG_${APPLICATION_NAME}_CMAKE "Print debug messages in CMakeLists.txt files." FALSE)
		
	if(DEBUG_${APPLICATION_NAME}_CMAKE OR
		DEBUG_ALL_ACTIVE_CMAKE)
		set(DEBUG_${APPLICATION_NAME}_CMAKE TRUE)
	endif()
	
	# Add GUI Programs to the Workspace
	#  If wxWidgets is found, build wxWidget applications.
	if(MSVC)
		set(wxWidgets_CONFIGURATION mswu)
		set(wxWidgets_USE_REL_AND_DBG 1)
	endif(MSVC)

	find_package(wxWidgets COMPONENTS net gl core base xml adv media)
	  
	if(wxWidgets_FOUND)
	
		# Print debug information for testing.
		if(DEBUG_${APPLICATION_NAME}_CMAKE)
			message(STATUS "\n  Configuring wxWidgets App [${APPLICATION_NAME}]")
			if(DEBUG_ACTIVE_CMAKE_SHOW_SRC_FILES)
				message(STATUS "  Source Files:")
				foreach(F ${${SOURCE_FILES}})
					message(STATUS "  - ${F}")
				endforeach()
			endif()
		endif(DEBUG_${APPLICATION_NAME}_CMAKE)	
		
		if(${ARGV5} MATCHES "EXCLUDE_CPACK")
			set(${APPLICATION_NAME}_EXCLUDE_CPACK TRUE)
			if(DEBUG_${APPLICATION_NAME}_CMAKE)
				message("Excluding  [${APPLICATION_NAME}] from Installers")
			endif()
		elseif(${ARGC} GREATER 5)
			set(SOLUTION_FOLDER_NAME ${ARGV5})
		endif()
	
		if(${LIBRARY_DEPENDENCIES})
			# Setup other dependencies
			configure_dependency_paths(${APPLICATION_NAME}
				${LIBRARY_DEPENDENCIES})
		endif()
		
		include(${wxWidgets_USE_FILE})
		add_executable(${APPLICATION_NAME} WIN32
				${${SOURCE_FILES}}
				)
		# Configure debug postfix for library
		set_target_properties(${APPLICATION_NAME}
			PROPERTIES DEBUG_POSTFIX
				${LIBRARY_DEBUG_POSTFIX})
				
		# Link against common system libraries
		target_link_libraries(${APPLICATION_NAME}
			${CMAKE_REQUIRED_LIBRARIES}
			${wxWidgets_LIBRARIES})
		
		if(${BUILD_DEPENDENCIES})
			# Setup internal build dependencies
			configure_build_dependencies(${APPLICATION_NAME}
				${BUILD_DEPENDENCIES})
		endif()
		if(${LIBRARY_DEPENDENCIES})
			# Setup other dependencies
			configure_ext_dependencies(${APPLICATION_NAME}
				${LIBRARY_DEPENDENCIES})
		endif()

		# Visual Studio Solution folders.
		if(MSVC_SOLUTION_FOLDERS)
			if(SOLUTION_FOLDER_NAME)
				set_property(TARGET ${APPLICATION_NAME} 
					PROPERTY FOLDER "${SOLUTION_FOLDER_NAME}")
			else()
				set_property(TARGET ${APPLICATION_NAME} 
					PROPERTY FOLDER "GUI Applications")
			endif()
		endif(MSVC_SOLUTION_FOLDERS)
		
		if(NOT ${APPLICATION_NAME}_EXCLUDE_CPACK)
			install(TARGETS ${APPLICATION_NAME}
					DESTINATION bin
					ARCHIVE DESTINATION lib
					COMPONENT Binaries)
			install(TARGETS ${APPLICATION_NAME}
					DESTINATION bin
					RUNTIME DESTINATION bin
					COMPONENT Binaries)
			install(TARGETS ${APPLICATION_NAME}
					DESTINATION bin
					LIBRARY DESTINATION bin
					COMPONENT Binaries)
		endif()
	endif()
endmacro(create_wx_app)
