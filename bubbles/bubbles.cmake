# Rename this file to, e.g. linux_64.cmake or vs11_32.cmake, and configure your default settings below.
# In your build-tree-directory call 'cmake -C this-file-with-path path-to-source-code'.

set(USER_CMAKE_TEXT "initialized by '${CMAKE_CURRENT_LIST_FILE}'")

set(CMAKE_GENERATOR "Visual Studio 11" CACHE INTERNAL ${USER_CMAKE_TEXT})
set(CMAKE_CONFIGURATION_TYPES Release Debug CACHE STRING ${USER_CMAKE_TEXT})

# path to runtime library (dll-files)
set(RUNTIME_LIBRARY_DIRS_DEBUG   "C:/libraries/install/vs11_32/freeglut/bin;C:/libraries/install/vs11_32/invrs/bin;C:/libraries/install/vs11_32/opensg/bin/debug" CACHE PATH ${USER_CMAKE_TEXT})
set(RUNTIME_LIBRARY_DIRS_RELEASE "C:/libraries/install/vs11_32/freeglut/bin;C:/libraries/install/vs11_32/invrs/bin;C:/libraries/install/vs11_32/opensg/bin/rel" CACHE PATH ${USER_CMAKE_TEXT})

set(Boost_USE_MULTITHREADED ON CACHE BOOL ${USER_CMAKE_TEXT})
set(Boost_USE_STATIC_LIBS ON CACHE BOOL ${USER_CMAKE_TEXT})
set(Boost_USE_STATIC_RUNTIME OFF CACHE BOOL ${USER_CMAKE_TEXT})
set(BOOST_ROOT "C:/libraries/install/vs11_32/boost" CACHE PATH ${USER_CMAKE_TEXT})
set(GLUT_INCLUDE_DIR "C:/libraries/install/vs11_32/freeglut/include" CACHE PATH ${USER_CMAKE_TEXT})
set(GLUT_glut_LIBRARY "optimized;C:/libraries/install/vs11_32/freeglut/lib/freeglut.lib;debug;C:/libraries/install/vs11_32/freeglut/lib/freeglutd.lib" CACHE PATH ${USER_CMAKE_TEXT})
set(OPENSG_ROOT "C:/libraries/install/vs11_32/opensg" CACHE PATH ${USER_CMAKE_TEXT})
set(inVRs_ROOT_DIR "C:/libraries/install/vs11_32/invrs" CACHE PATH ${USER_CMAKE_TEXT})
set(CAVESceneManager_DIR "${inVRs_ROOT_DIR}/cmake" CACHE PATH ${USER_CMAKE_TEXT})
set(TRACKD_ROOT_DIR "D:/vs11/libraries/trackd/trackd-2.4" CACHE PATH ${USER_CMAKE_TEXT})
set(VRPN_ROOT_DIR "C:/libraries/install/vs11_32/vrpn" CACHE PATH ${USER_CMAKE_TEXT})

# optionals
set(ENABLE_VRPN_SUPPORT ON CACHE BOOL "Support for VRPN Tracking")
#set(ENABLE_TRACKD_SUPPORT ON CACHE BOOL "Support for TrackD Tracking")
#set(ENABLE_INVRSVRPNDEVICE_SUPPORT ON CACHE BOOL "Support for inVRsVrpnDevice Tracking")
#set(ENABLE_INVRSTRACKDDEVICE_SUPPORT ON CACHE BOOL "Support for inVRsTrackdDevice Tracking")

# set single turorials off
set(BUILD_GoingImmersive OFF CACHE BOOL ${USER_CMAKE_TEXT})
set(BUILD_GoingImmersiveFinal OFF CACHE BOOL ${USER_CMAKE_TEXT})
set(BUILD_MedievalTown OFF CACHE BOOL ${USER_CMAKE_TEXT})
set(BUILD_MedievalTownFinal OFF CACHE BOOL ${USER_CMAKE_TEXT})
set(BUILD_MedievalTownCSM OFF CACHE BOOL ${USER_CMAKE_TEXT})
set(BUILD_MedievalTownNewAppBase OFF CACHE BOOL ${USER_CMAKE_TEXT})
set(BUILD_MedievalTownPhysics OFF CACHE BOOL ${USER_CMAKE_TEXT})
set(BUILD_MedievalTownPhysicsFinal OFF CACHE BOOL ${USER_CMAKE_TEXT})
set(BUILD_OpenSG OFF CACHE BOOL ${USER_CMAKE_TEXT})
set(BUILD_OpenSGFinal OFF CACHE BOOL ${USER_CMAKE_TEXT})
set(BUILD_opensg_csm OFF CACHE BOOL ${USER_CMAKE_TEXT})
set(BUILD_opensg_trackd OFF CACHE BOOL ${USER_CMAKE_TEXT})
set(BUILD_opensg_vrpn OFF CACHE BOOL ${USER_CMAKE_TEXT})
set(BUILD_opensg_oculus OFF CACHE BOOL ${USER_CMAKE_TEXT})
