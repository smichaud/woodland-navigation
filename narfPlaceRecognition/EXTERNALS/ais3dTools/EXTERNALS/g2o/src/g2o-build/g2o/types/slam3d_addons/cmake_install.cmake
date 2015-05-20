# Install script for directory: /home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/EXTERNALS/ais3dTools/EXTERNALS/g2o/src/g2o/g2o/types/slam3d_addons

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/build")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if(EXISTS "$ENV{DESTDIR}/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/build/lib/libg2o_types_slam3d_addons.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/build/lib/libg2o_types_slam3d_addons.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/build/lib/libg2o_types_slam3d_addons.so"
         RPATH "")
  endif()
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/build/lib/libg2o_types_slam3d_addons.so")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/build/lib" TYPE SHARED_LIBRARY FILES "/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/EXTERNALS/ais3dTools/EXTERNALS/g2o/src/g2o/lib/libg2o_types_slam3d_addons.so")
  if(EXISTS "$ENV{DESTDIR}/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/build/lib/libg2o_types_slam3d_addons.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/build/lib/libg2o_types_slam3d_addons.so")
    file(RPATH_REMOVE
         FILE "$ENV{DESTDIR}/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/build/lib/libg2o_types_slam3d_addons.so")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/build/lib/libg2o_types_slam3d_addons.so")
    endif()
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/build/include/g2o/types/slam3d_addons/edge_se3_plane_calib.h;/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/build/include/g2o/types/slam3d_addons/plane3d.h;/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/build/include/g2o/types/slam3d_addons/line3d.h;/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/build/include/g2o/types/slam3d_addons/edge_line3d.h;/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/build/include/g2o/types/slam3d_addons/edge_se3_calib.h;/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/build/include/g2o/types/slam3d_addons/edge_se3_euler.h;/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/build/include/g2o/types/slam3d_addons/vertex_plane.h;/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/build/include/g2o/types/slam3d_addons/types_slam3d_addons.h;/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/build/include/g2o/types/slam3d_addons/vertex_line3d.h;/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/build/include/g2o/types/slam3d_addons/vertex_se3_euler.h;/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/build/include/g2o/types/slam3d_addons/edge_se3_line.h;/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/build/include/g2o/types/slam3d_addons/edge_plane.h;/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/build/include/g2o/types/slam3d_addons/g2o_types_slam3d_addons_api.h")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/build/include/g2o/types/slam3d_addons" TYPE FILE FILES
    "/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/EXTERNALS/ais3dTools/EXTERNALS/g2o/src/g2o/g2o/types/slam3d_addons/edge_se3_plane_calib.h"
    "/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/EXTERNALS/ais3dTools/EXTERNALS/g2o/src/g2o/g2o/types/slam3d_addons/plane3d.h"
    "/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/EXTERNALS/ais3dTools/EXTERNALS/g2o/src/g2o/g2o/types/slam3d_addons/line3d.h"
    "/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/EXTERNALS/ais3dTools/EXTERNALS/g2o/src/g2o/g2o/types/slam3d_addons/edge_line3d.h"
    "/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/EXTERNALS/ais3dTools/EXTERNALS/g2o/src/g2o/g2o/types/slam3d_addons/edge_se3_calib.h"
    "/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/EXTERNALS/ais3dTools/EXTERNALS/g2o/src/g2o/g2o/types/slam3d_addons/edge_se3_euler.h"
    "/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/EXTERNALS/ais3dTools/EXTERNALS/g2o/src/g2o/g2o/types/slam3d_addons/vertex_plane.h"
    "/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/EXTERNALS/ais3dTools/EXTERNALS/g2o/src/g2o/g2o/types/slam3d_addons/types_slam3d_addons.h"
    "/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/EXTERNALS/ais3dTools/EXTERNALS/g2o/src/g2o/g2o/types/slam3d_addons/vertex_line3d.h"
    "/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/EXTERNALS/ais3dTools/EXTERNALS/g2o/src/g2o/g2o/types/slam3d_addons/vertex_se3_euler.h"
    "/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/EXTERNALS/ais3dTools/EXTERNALS/g2o/src/g2o/g2o/types/slam3d_addons/edge_se3_line.h"
    "/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/EXTERNALS/ais3dTools/EXTERNALS/g2o/src/g2o/g2o/types/slam3d_addons/edge_plane.h"
    "/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/EXTERNALS/ais3dTools/EXTERNALS/g2o/src/g2o/g2o/types/slam3d_addons/g2o_types_slam3d_addons_api.h"
    )
endif()

