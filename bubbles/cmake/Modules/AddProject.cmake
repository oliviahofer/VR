# The function add_project() sets up an invrs tutorial

cmake_minimum_required(VERSION 2.8)

include(CMakeParseArguments)

# copies files from src-tree to build-tree and adds them to a source group
function(_add_project_resource_files _NAME)
	set_source_files_properties(${ARGN} PROPERTIES HEADER_FILE_ONLY TRUE)
	# copy files to build-tree
	foreach(resource ${ARGN})
		configure_file(${resource} ${resource} COPYONLY)
	endforeach()
	# files in src-tree, not build-tree are referenced in project-file, so changes affect the src-tree
	# changed files will be copied into build-tree automatically
	source_group(${_NAME} FILES ${ARGN})
endfunction()

function(_add_project_files _RESULT)
	if(NOT "${ARGN}" STREQUAL "")
		file(GLOB_RECURSE ${_RESULT} RELATIVE ${CMAKE_CURRENT_LIST_DIR} ${ARGN})
	endif()
	set(${_RESULT} ${${_RESULT}} PARENT_SCOPE)
endfunction()

function(_add_project_setup ADD_PROJECT_NAME)
	set(_options CSM_MODULE VRPN_MODULE TRACKD_MODULE)
	set(_oneValueArgs "")
	set(_multiValueArgs SOURCES CONFIGS MODELS IMAGES DOCS OPENSG_MODULES INVRS_MODULES BOOST_MODULES LIBRARIES DEFINITIONS INCLUDE_DIRS)
	cmake_parse_arguments(ADD_PROJECT "${_options}" "${_oneValueArgs}" "${_multiValueArgs}" ${ARGN})
	if(ADD_PROJECT_UNPARSED_ARGUMENTS)
		message(FATAL_ERROR "Error: '" "${ADD_PROJECT_UNPARSED_ARGUMENTS}" "': unexpected arguments")
	endif()

	# sources
	_add_project_files(SOURCE_FILES ${ADD_PROJECT_SOURCES})

	# configuration
	_add_project_files(CONFIGURATION_FILES ${ADD_PROJECT_CONFIGS})
	_add_project_resource_files("Configuration Files" ${CONFIGURATION_FILES})

	# models
	_add_project_files(MODEL_FILES ${ADD_PROJECT_MODELS})
	_add_project_resource_files("Model Files" ${MODEL_FILES})

	# images
	_add_project_files(IMAGE_FILES ${ADD_PROJECT_IMAGES})
	_add_project_resource_files("Image Files" ${IMAGE_FILES})
	
	# documents
	_add_project_files(DOC_FILES ${ADD_PROJECT_DOCS})
	_add_project_resource_files("Document Files" ${DOC_FILES})

	# OpenSG
	if(ADD_PROJECT_OPENSG_MODULES)
		find_package(OpenSG REQUIRED ${ADD_PROJECT_OPENSG_MODULES})
		list(APPEND ${ADD_PROJECT_NAME}_DEFINITIONS ${OpenSG_DEFINITIONS})
		list(APPEND ${ADD_PROJECT_NAME}_INCLUDE_DIRS ${OpenSG_INCLUDE_DIRS})
		list(APPEND ${ADD_PROJECT_NAME}_LIBRARIES ${OpenSG_LIBRARIES})
		
		# needs GLUT but doesn't attempt to find it
		find_package(GLUT REQUIRED)
		list(APPEND ${ADD_PROJECT_NAME}_DEFINITIONS ${GLUT_DEFINITIONS})
		list(APPEND ${ADD_PROJECT_NAME}_INCLUDE_DIRS ${GLUT_INCLUDE_DIR})
		list(APPEND ${ADD_PROJECT_NAME}_LIBRARIES ${GLUT_LIBRARIES})
	endif()

	# Boost
	if(ADD_PROJECT_BOOST_MODULES)
		find_package(Boost REQUIRED ${ADD_PROJECT_BOOST_MODULES})
		list(APPEND ${ADD_PROJECT_NAME}_DEFINITIONS ${Boost_DEFINITIONS})
		list(APPEND ${ADD_PROJECT_NAME}_INCLUDE_DIRS ${Boost_INCLUDE_DIRS})
		list(APPEND ${ADD_PROJECT_NAME}_LIBRARIES ${Boost_LIBRARIES})
	endif()
	
	# CSM
	set(ENABLE_CSM_SUPPORT OFF CACHE BOOL "Support for CAVE Scene Manager (CSM)")
	if(ADD_PROJECT_CSM_MODULE OR ENABLE_CSM_SUPPORT)
		find_package(CAVESceneManager REQUIRED)
		list(APPEND ${ADD_PROJECT_NAME}_DEFINITIONS ${CAVESCENEMANAGER_DEFINITIONS} -DENABLE_CSM_SUPPORT)
		list(APPEND ${ADD_PROJECT_NAME}_INCLUDE_DIRS ${CAVESCENEMANAGER_INCLUDE_DIRS})
		list(APPEND ${ADD_PROJECT_NAME}_LIBRARIES ${CAVESCENEMANAGER_LIBRARIES})
	endif()

	# VRPN
	set(ENABLE_VRPN_SUPPORT OFF CACHE BOOL "Support for VRPN Tracking")
	if(ADD_PROJECT_VRPN_MODULE OR ENABLE_VRPN_SUPPORT)
		find_package(VRPN REQUIRED)
		list(APPEND ${ADD_PROJECT_NAME}_DEFINITIONS ${VRPN_DEFINITIONS} -DENABLE_VRPN_SUPPORT)
		list(APPEND ${ADD_PROJECT_NAME}_INCLUDE_DIRS ${VRPN_INCLUDE_DIRS})
		list(APPEND ${ADD_PROJECT_NAME}_LIBRARIES ${VRPN_LIBRARIES})
	endif()
	
	# TrackD
	set(ENABLE_TRACKD_SUPPORT OFF CACHE BOOL "Support for TrackD Tracking")
	if(ADD_PROJECT_TRACKD_MODULE OR ENABLE_TRACKD_SUPPORT)
		find_package(TRACKD REQUIRED)
		list(APPEND ${ADD_PROJECT_NAME}_DEFINITIONS ${TRACKD_DEFINITIONS} -DENABLE_TRACKD_SUPPORT)
		list(APPEND ${ADD_PROJECT_NAME}_INCLUDE_DIRS ${TRACKD_INCLUDE_DIRS})
		list(APPEND ${ADD_PROJECT_NAME}_LIBRARIES ${TRACKD_LIBRARIES})
	endif()
	
	# INVRS
	set(ENABLE_INVRSVRPNDEVICE_SUPPORT OFF CACHE BOOL "Support for inVRsVrpnDevice Tracking")
	set(ENABLE_INVRSTRACKDDEVICE_SUPPORT OFF CACHE BOOL "Support for inVRsTrackdDevice Tracking")
	if(ADD_PROJECT_INVRS_MODULES)
		if(ENABLE_INVRSVRPNDEVICE_SUPPORT)
			list(APPEND ADD_PROJECT_INVRS_MODULES VrpnDevice)
		endif()
		if(ENABLE_INVRSTRACKDDEVICE_SUPPORT)
			list(APPEND ADD_PROJECT_INVRS_MODULES TrackdDevice)
		endif()
		find_package(inVRs REQUIRED ${ADD_PROJECT_INVRS_MODULES})
		list(APPEND ${ADD_PROJECT_NAME}_DEFINITIONS ${inVRs_DEFINITIONS})
		list(APPEND ${ADD_PROJECT_NAME}_INCLUDE_DIRS ${inVRs_INCLUDE_DIRS})
		list(APPEND ${ADD_PROJECT_NAME}_LIBRARIES ${inVRs_LIBRARIES})
		list(APPEND ${ADD_PROJECT_NAME}_LIBRARY_DIRS ${inVRs_LIBRARY_DIRS})
	endif()

	link_directories(${${ADD_PROJECT_NAME}_LIBRARY_DIRS})
	add_executable (${ADD_PROJECT_NAME} ${SOURCE_FILES} ${CONFIGURATION_FILES} ${MODEL_FILES} ${IMAGE_FILES} ${DOC_FILES})
	add_definitions(${${ADD_PROJECT_NAME}_DEFINITIONS} ${ADD_PROJECT_DEFINITIONS})
	include_directories(${${ADD_PROJECT_NAME}_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR} ${ADD_PROJECT_INCLUDE_DIRS})
	target_link_libraries(${ADD_PROJECT_NAME} ${${ADD_PROJECT_NAME}_LIBRARIES} ${ADD_PROJECT_LIBRARIES})
	
	set_property(GLOBAL PROPERTY USE_FOLDERS ON)
	set_property(TARGET ${ADD_PROJECT_NAME} PROPERTY FOLDER "Tutorials")

	# PORTAUDIO
	include_directories("../../portaudio/include") # include header
	link_directories("../../../portaudio/bin/Debug") # libraries
	target_link_libraries(${ADD_PROJECT_NAME} ../../../portaudio/bin/Debug/portaudio_x86) # .lib file
	file(COPY "../../portaudio/bin/Debug/portaudio_x86.dll" DESTINATION "./Debug" )
	
	# add path to runtime libraries (dll-files)
	if (MSVC10 OR MSVC11)
		set (RUNTIME_LIBRARY_DIRS_RELEASE ${${ADD_PROJECT_NAME}_LIBRARY_DIRS} CACHE PATH "path to release dlls")
		set (RUNTIME_LIBRARY_DIRS_DEBUG ${${ADD_PROJECT_NAME}_LIBRARY_DIRS} CACHE PATH "path to debug dlls")
		configure_file("../cmake/Modules/template_vs10-11.vcxproj.user.in" "${ADD_PROJECT_NAME}.vcxproj.user" @ONLY)
	endif()
endfunction()

function(add_project ADD_PROJECT_NAME)
	set(BUILD_${ADD_PROJECT_NAME} ON CACHE BOOL "Build ${ADD_PROJECT_NAME}")
	if(BUILD_${ADD_PROJECT_NAME})
		_add_project_setup(${ADD_PROJECT_NAME} ${ARGN})
	endif()
endfunction()
