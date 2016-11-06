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

# Get library macros
include(${ACTIVE_CMAKE_DIRECTORY}/CreateLibrary.cmake)

# Set Solution/Workspace name, this will be re-used
set(PROJECT_NAME jpeg-6b)

if(MSVC)
	set(PROJECT_SRC_FOLDER ${ACTIVE_EXT_DIRECTORY}/${PROJECT_NAME}/msvc)
else()
	set(PROJECT_SRC_FOLDER ${ACTIVE_EXT_DIRECTORY}/${PROJECT_NAME}/linux)
endif()

set (PROJECT_SOURCE_FILES 
	${PROJECT_SRC_FOLDER}/jcapimin.c
	${PROJECT_SRC_FOLDER}/jcapistd.c
	${PROJECT_SRC_FOLDER}/jctrans.c
	${PROJECT_SRC_FOLDER}/jcparam.c
	${PROJECT_SRC_FOLDER}/jdatadst.c
	${PROJECT_SRC_FOLDER}/jcinit.c
	${PROJECT_SRC_FOLDER}/jcmaster.c
	${PROJECT_SRC_FOLDER}/jcmarker.c
	${PROJECT_SRC_FOLDER}/jcmainct.c
	${PROJECT_SRC_FOLDER}/jcprepct.c
	${PROJECT_SRC_FOLDER}/jccoefct.c
	${PROJECT_SRC_FOLDER}/jccolor.c
	${PROJECT_SRC_FOLDER}/jcsample.c
	${PROJECT_SRC_FOLDER}/jchuff.c
	${PROJECT_SRC_FOLDER}/jcphuff.c
	${PROJECT_SRC_FOLDER}/jcdctmgr.c
	${PROJECT_SRC_FOLDER}/jfdctfst.c
	${PROJECT_SRC_FOLDER}/jfdctflt.c
	${PROJECT_SRC_FOLDER}/jfdctint.c
	${PROJECT_SRC_FOLDER}/jdapimin.c
	${PROJECT_SRC_FOLDER}/jdapistd.c
	${PROJECT_SRC_FOLDER}/jdtrans.c
	${PROJECT_SRC_FOLDER}/jdatasrc.c
	${PROJECT_SRC_FOLDER}/jdmaster.c
	${PROJECT_SRC_FOLDER}/jdinput.c
	${PROJECT_SRC_FOLDER}/jdmarker.c
	${PROJECT_SRC_FOLDER}/jdhuff.c
	${PROJECT_SRC_FOLDER}/jdphuff.c
	${PROJECT_SRC_FOLDER}/jdmainct.c
	${PROJECT_SRC_FOLDER}/jdcoefct.c
	${PROJECT_SRC_FOLDER}/jdpostct.c
	${PROJECT_SRC_FOLDER}/jddctmgr.c
	${PROJECT_SRC_FOLDER}/jidctfst.c
	${PROJECT_SRC_FOLDER}/jidctflt.c
	${PROJECT_SRC_FOLDER}/jidctint.c
	${PROJECT_SRC_FOLDER}/jidctred.c
	${PROJECT_SRC_FOLDER}/jdsample.c
	${PROJECT_SRC_FOLDER}/jdcolor.c
	${PROJECT_SRC_FOLDER}/jquant1.c
	${PROJECT_SRC_FOLDER}/jquant2.c
	${PROJECT_SRC_FOLDER}/jdmerge.c
	${PROJECT_SRC_FOLDER}/jcomapi.c
	${PROJECT_SRC_FOLDER}/jutils.c
	${PROJECT_SRC_FOLDER}/jerror.c
	${PROJECT_SRC_FOLDER}/jmemmgr.c
)

# Add extra file needed for unix build
if(WIN32)
	set(PROJECT_SOURCE_FILES 
		${PROJECT_SOURCE_FILES} ${PROJECT_SRC_FOLDER}/jmemansi.c)
else()
	set(PROJECT_SOURCE_FILES 
		${PROJECT_SOURCE_FILES} ${PROJECT_SRC_FOLDER}/jmemnobs.c)
endif(WIN32)

configure_ext_library(${PROJECT_NAME}
						PROJECT_SOURCE_FILES
						""
						STATIC
						LIBRARY_NAME)
						
