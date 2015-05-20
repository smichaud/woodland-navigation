# Install script for directory: /home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/EXTERNALS/ais3dTools/EXTERNALS/g2o/src/g2o/g2o/apps/g2o_viewer

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
  if(EXISTS "$ENV{DESTDIR}/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/build/bin/g2o_viewer" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/build/bin/g2o_viewer")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/build/bin/g2o_viewer"
         RPATH "")
  endif()
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/build/bin/g2o_viewer")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/build/bin" TYPE EXECUTABLE FILES "/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/EXTERNALS/ais3dTools/EXTERNALS/g2o/src/g2o/bin/g2o_viewer")
  if(EXISTS "$ENV{DESTDIR}/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/build/bin/g2o_viewer" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/build/bin/g2o_viewer")
    file(RPATH_REMOVE
         FILE "$ENV{DESTDIR}/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/build/bin/g2o_viewer")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/build/bin/g2o_viewer")
    endif()
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if(EXISTS "$ENV{DESTDIR}/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/build/lib/libg2o_viewer.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/build/lib/libg2o_viewer.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/build/lib/libg2o_viewer.so"
         RPATH "")
  endif()
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/build/lib/libg2o_viewer.so")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/build/lib" TYPE SHARED_LIBRARY FILES "/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/EXTERNALS/ais3dTools/EXTERNALS/g2o/src/g2o/lib/libg2o_viewer.so")
  if(EXISTS "$ENV{DESTDIR}/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/build/lib/libg2o_viewer.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/build/lib/libg2o_viewer.so")
    file(RPATH_REMOVE
         FILE "$ENV{DESTDIR}/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/build/lib/libg2o_viewer.so")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/build/lib/libg2o_viewer.so")
    endif()
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/build/include/g2o/apps/g2o_viewer/gui_hyper_graph_action.h;/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/build/include/g2o/apps/g2o_viewer/main_window.h;/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/build/include/g2o/apps/g2o_viewer/stream_redirect.h;/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/build/include/g2o/apps/g2o_viewer/run_g2o_viewer.h;/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/build/include/g2o/apps/g2o_viewer/g2o_qglviewer.h;/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/build/include/g2o/apps/g2o_viewer/properties_widget.h;/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/build/include/g2o/apps/g2o_viewer/g2o_viewer_api.h;/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/build/include/g2o/apps/g2o_viewer/viewer_properties_widget.h")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/build/include/g2o/apps/g2o_viewer" TYPE FILE FILES
    "/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/EXTERNALS/ais3dTools/EXTERNALS/g2o/src/g2o/g2o/apps/g2o_viewer/gui_hyper_graph_action.h"
    "/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/EXTERNALS/ais3dTools/EXTERNALS/g2o/src/g2o/g2o/apps/g2o_viewer/main_window.h"
    "/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/EXTERNALS/ais3dTools/EXTERNALS/g2o/src/g2o/g2o/apps/g2o_viewer/stream_redirect.h"
    "/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/EXTERNALS/ais3dTools/EXTERNALS/g2o/src/g2o/g2o/apps/g2o_viewer/run_g2o_viewer.h"
    "/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/EXTERNALS/ais3dTools/EXTERNALS/g2o/src/g2o/g2o/apps/g2o_viewer/g2o_qglviewer.h"
    "/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/EXTERNALS/ais3dTools/EXTERNALS/g2o/src/g2o/g2o/apps/g2o_viewer/properties_widget.h"
    "/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/EXTERNALS/ais3dTools/EXTERNALS/g2o/src/g2o/g2o/apps/g2o_viewer/g2o_viewer_api.h"
    "/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/EXTERNALS/ais3dTools/EXTERNALS/g2o/src/g2o/g2o/apps/g2o_viewer/viewer_properties_widget.h"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/build/include/g2o/apps/g2o_viewer/ui_base_main_window.h;/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/build/include/g2o/apps/g2o_viewer/ui_base_properties_widget.h")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/build/include/g2o/apps/g2o_viewer" TYPE FILE FILES
    "/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/EXTERNALS/ais3dTools/EXTERNALS/g2o/src/g2o-build/g2o/apps/g2o_viewer/ui_base_main_window.h"
    "/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/EXTERNALS/ais3dTools/EXTERNALS/g2o/src/g2o-build/g2o/apps/g2o_viewer/ui_base_properties_widget.h"
    )
endif()

