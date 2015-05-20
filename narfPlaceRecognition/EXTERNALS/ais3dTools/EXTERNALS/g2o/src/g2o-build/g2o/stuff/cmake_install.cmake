# Install script for directory: /home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/EXTERNALS/ais3dTools/EXTERNALS/g2o/src/g2o/g2o/stuff

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
  if(EXISTS "$ENV{DESTDIR}/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/build/lib/libg2o_stuff.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/build/lib/libg2o_stuff.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/build/lib/libg2o_stuff.so"
         RPATH "")
  endif()
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/build/lib/libg2o_stuff.so")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/build/lib" TYPE SHARED_LIBRARY FILES "/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/EXTERNALS/ais3dTools/EXTERNALS/g2o/src/g2o/lib/libg2o_stuff.so")
  if(EXISTS "$ENV{DESTDIR}/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/build/lib/libg2o_stuff.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/build/lib/libg2o_stuff.so")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/build/lib/libg2o_stuff.so")
    endif()
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if(EXISTS "$ENV{DESTDIR}/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/build/lib/libg2o_opengl_helper.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/build/lib/libg2o_opengl_helper.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/build/lib/libg2o_opengl_helper.so"
         RPATH "")
  endif()
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/build/lib/libg2o_opengl_helper.so")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/build/lib" TYPE SHARED_LIBRARY FILES "/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/EXTERNALS/ais3dTools/EXTERNALS/g2o/src/g2o/lib/libg2o_opengl_helper.so")
  if(EXISTS "$ENV{DESTDIR}/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/build/lib/libg2o_opengl_helper.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/build/lib/libg2o_opengl_helper.so")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/build/lib/libg2o_opengl_helper.so")
    endif()
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/build/include/g2o/stuff/macros.h;/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/build/include/g2o/stuff/sparse_helper.h;/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/build/include/g2o/stuff/tictoc.h;/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/build/include/g2o/stuff/filesys_tools.h;/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/build/include/g2o/stuff/opengl_wrapper.h;/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/build/include/g2o/stuff/scoped_pointer.h;/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/build/include/g2o/stuff/command_args.h;/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/build/include/g2o/stuff/g2o_stuff_api.h;/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/build/include/g2o/stuff/opengl_primitives.h;/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/build/include/g2o/stuff/unscented.h;/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/build/include/g2o/stuff/os_specific.h;/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/build/include/g2o/stuff/misc.h;/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/build/include/g2o/stuff/timeutil.h;/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/build/include/g2o/stuff/sampler.h;/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/build/include/g2o/stuff/string_tools.h;/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/build/include/g2o/stuff/property.h;/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/build/include/g2o/stuff/color_macros.h")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/build/include/g2o/stuff" TYPE FILE FILES
    "/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/EXTERNALS/ais3dTools/EXTERNALS/g2o/src/g2o/g2o/stuff/macros.h"
    "/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/EXTERNALS/ais3dTools/EXTERNALS/g2o/src/g2o/g2o/stuff/sparse_helper.h"
    "/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/EXTERNALS/ais3dTools/EXTERNALS/g2o/src/g2o/g2o/stuff/tictoc.h"
    "/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/EXTERNALS/ais3dTools/EXTERNALS/g2o/src/g2o/g2o/stuff/filesys_tools.h"
    "/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/EXTERNALS/ais3dTools/EXTERNALS/g2o/src/g2o/g2o/stuff/opengl_wrapper.h"
    "/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/EXTERNALS/ais3dTools/EXTERNALS/g2o/src/g2o/g2o/stuff/scoped_pointer.h"
    "/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/EXTERNALS/ais3dTools/EXTERNALS/g2o/src/g2o/g2o/stuff/command_args.h"
    "/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/EXTERNALS/ais3dTools/EXTERNALS/g2o/src/g2o/g2o/stuff/g2o_stuff_api.h"
    "/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/EXTERNALS/ais3dTools/EXTERNALS/g2o/src/g2o/g2o/stuff/opengl_primitives.h"
    "/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/EXTERNALS/ais3dTools/EXTERNALS/g2o/src/g2o/g2o/stuff/unscented.h"
    "/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/EXTERNALS/ais3dTools/EXTERNALS/g2o/src/g2o/g2o/stuff/os_specific.h"
    "/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/EXTERNALS/ais3dTools/EXTERNALS/g2o/src/g2o/g2o/stuff/misc.h"
    "/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/EXTERNALS/ais3dTools/EXTERNALS/g2o/src/g2o/g2o/stuff/timeutil.h"
    "/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/EXTERNALS/ais3dTools/EXTERNALS/g2o/src/g2o/g2o/stuff/sampler.h"
    "/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/EXTERNALS/ais3dTools/EXTERNALS/g2o/src/g2o/g2o/stuff/string_tools.h"
    "/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/EXTERNALS/ais3dTools/EXTERNALS/g2o/src/g2o/g2o/stuff/property.h"
    "/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/EXTERNALS/ais3dTools/EXTERNALS/g2o/src/g2o/g2o/stuff/color_macros.h"
    )
endif()

