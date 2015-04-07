CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

project(narfPlaceRecognition)

# The library prefix
SET(LIB_PREFIX ${PROJECT_NAME}_)

SET(${PROJECT_NAME}_C_FLAGS)
SET(${PROJECT_NAME}_CXX_FLAGS)

IF (UNIX)
  SET(BUILD_SHARED_LIBS ON)
ELSEIF (WIN32)
  SET(BUILD_SHARED_LIBS OFF)
ENDIF()

# default built type
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING
      "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
      FORCE)
ENDIF(NOT CMAKE_BUILD_TYPE)

OPTION (BUILD_GUI_ELEMENTS "Build GUI elements that use QT and the QGLViewer)" ON)
IF (BUILD_GUI_ELEMENTS)
  MESSAGE(STATUS "Compiling with GUI support")
  ADD_DEFINITIONS(-DUSE_GUI)
  
  SET(ais3dTools_BUILD_VISUALIZATION ON CACHE BOOL "Build Visualization in Ais3dTools" FORCE)
  
  # Find the QGLViewer library
  # Set search directory for looking for our custom CMake scripts to look for QGLViewer
  SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/EXTERNALS/ais3dTools/cmake_modules)
  SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules)
  FIND_PACKAGE(QGLViewer)
  INCLUDE_DIRECTORIES(${QGLVIEWER_INCLUDE_DIR})
ELSE (BUILD_GUI_ELEMENTS)
  MESSAGE(STATUS "Compiling without GUI support")
  SET(ais3dTools_BUILD_VISUALIZATION OFF CACHE BOOL "Build specific element in Ais3dTools" FORCE)
ENDIF(BUILD_GUI_ELEMENTS)

# USE G2O
SET(ais3dTools_BUILD_G2O ON CACHE BOOL "Build G2O in Ais3dTools" FORCE)

# SWITCH EVERYTHING ELSE IN AIS3DTOOLS OFF
SET(ais3dTools_BUILD_SURFACEPRIMITIVES OFF CACHE BOOL "Build specific element in Ais3dTools" FORCE)
SET(ais3dTools_BUILD_APPS OFF CACHE BOOL "Build specific element in Ais3dTools" FORCE)
SET(ais3dTools_BUILD_TOOLS OFF CACHE BOOL "Build specific element in Ais3dTools" FORCE)
SET(ais3dTools_BUILD_EXAMPLES OFF CACHE BOOL "Build specific element in Ais3dTools" FORCE)
SET(ais3dTools_BUILD_OPENNI OFF CACHE BOOL "Build specific element in Ais3dTools" FORCE)
SET(ais3dTools_BUILD_SSA OFF CACHE BOOL "Build specific element in Ais3dTools" FORCE)
SET(ais3dTools_BUILD_MODELLING OFF CACHE BOOL "Build specific element in Ais3dTools" FORCE)

# Overwrite variables from ais3dTools
SET(CMAKE_DEBUG_POSTFIX "" CACHE STRING "postfix applied to debug build of libraries")
SET(CMAKE_RELEASE_POSTFIX "" CACHE STRING "postfix applied to release build of libraries")
SET(CMAKE_RELWITHDEBINFO_POSTFIX "" CACHE STRING "postfix applied to release-with-debug-information libraries")
SET(CMAKE_MINSIZEREL_POSTFIX "" CACHE STRING "postfix applied to minimium-size-build libraries")

# Set the output directory for the build executables and libraries
SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
IF(WIN32)
  SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
ELSE()
  SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
ENDIF()


# Detect OS and define macros appropriately
IF(WIN32)
  IF(CYGWIN)
    MESSAGE(STATUS "Compiling on Cygwin")
    ADD_DEFINITIONS(-DCYGWIN)
  ELSE(CYGWIN)
    ADD_DEFINITIONS(-DWINDOWS)
    MESSAGE(STATUS "Compiling on Windows")
  ENDIF(CYGWIN)
  IF(MSVC)
    MESSAGE(STATUS "Compiling on Windows (MSVC)")
    ADD_DEFINITIONS(-DNOMINMAX)
    ADD_DEFINITIONS(-D_USE_MATH_DEFINES)
    ADD_DEFINITIONS("/wd4305 /wd4244")  # disable warning on implicit casts from double to float
    ADD_DEFINITIONS("/wd4181")          # disable warning "qualifier applied to reference type"
    IF (BUILD_SHARED_LIBS)
      # disable warning on missing DLL interfaces
      ADD_DEFINITIONS("/wd4251")
    ENDIF()
  ENDIF(MSVC)
ELSEIF(UNIX)
  ADD_DEFINITIONS(-DUNIX)
  MESSAGE(STATUS "Compiling on Unix")
ELSEIF(APPLE)
  ADD_DEFINITIONS(-DUNIX)
  MESSAGE(STATUS "Compiling on OSX")
ENDIF(WIN32)

# Eigen library parallelise itself, though, presumably due to performance issues
# OPENMP is experimental. We experienced some slowdown with it
#SET( ${PROJECT_NAME}_USE_OPENMP OFF CACHE BOOL "Build with OpenMP support (EXPERIMENTAL)")
SET( ${PROJECT_NAME}_USE_OPENMP ON CACHE BOOL "Build with OpenMP support (EXPERIMENTAL)")
IF (${PROJECT_NAME}_USE_OPENMP)
  FIND_PACKAGE(OpenMP)
  IF(OPENMP_FOUND)
    SET(${PROJECT_NAME}_C_FLAGS "${${PROJECT_NAME}_C_FLAGS} ${OpenMP_C_FLAGS}")
    SET(${PROJECT_NAME}_CXX_FLAGS "${${PROJECT_NAME}_CXX_FLAGS} -DEIGEN_DONT_PARALLELIZE ${OpenMP_CXX_FLAGS}")
    MESSAGE(STATUS "Compiling with OpenMP support")
    ADD_DEFINITIONS(-DOPENMP_SUPPORT)
  ENDIF(OPENMP_FOUND)
ENDIF(${PROJECT_NAME}_USE_OPENMP)


# Find Eigen3
#FIND_PACKAGE(Eigen3)
#INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})

# PCL
find_package(PCL 1.7 REQUIRED)
include_directories(SYSTEM ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
#INCLUDE_DIRECTORIES(EXTERBALS/pcl/common/include)

# Boost
FIND_PACKAGE(Boost 1.36.0 COMPONENTS filesystem system)
IF(Boost_FOUND)
  MESSAGE(STATUS "Boost found")
  IF($ENV{VERBOSE})
    MESSAGE(STATUS "   Boost_INCLUDE_DIR = ${Boost_INCLUDE_DIR}")
    MESSAGE(STATUS "   Boost_LINK_DIRECTORIES = ${Boost_LINK_DIRECTORIES}")
  ENDIF($ENV{VERBOSE})
  INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIR})
  LINK_DIRECTORIES(${Boost_LINK_DIRECTORIES})
ELSE(Boost_FOUND)
	#MESSAGE(FATAL_ERROR "	Boost not found.\n Boost can be found on http://www.boost.org")
        MESSAGE(WARNING "  Boost not found.\n  It can be found on http://www.boost.org")
ENDIF(Boost_FOUND)

# Compiler specific options for gcc
IF(CMAKE_COMPILER_IS_GNUCXX)
  # Generic settings for optimisation
  SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -msse4") 
  SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3 -msse4") 
  # activate warnings !!!
  SET(${PROJECT_NAME}_C_FLAGS "${${PROJECT_NAME}_C_FLAGS} -Wall -W")
  SET(${PROJECT_NAME}_CXX_FLAGS "${${PROJECT_NAME}_CXX_FLAGS} -Wall -W")
  
  IF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native") 
    SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native") 
  ENDIF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
ENDIF(CMAKE_COMPILER_IS_GNUCXX)

## There seems to be an issue with MSVC8
## see http://eigen.tuxfamily.org/bz/show_bug.cgi?id=83
#if(MSVC90)
  #add_definitions(-DEIGEN_DONT_ALIGN_STATICALLY=1)
  #message(STATUS "Disabling memory alignment for MSVC8")
#endif(MSVC90)

# specifying compiler flags
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${${PROJECT_NAME}_CXX_FLAGS}")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${${PROJECT_NAME}_C_FLAGS}")

# Set up the top-level include directories
#INCLUDE_DIRECTORIES(${${PROJECT_NAME}_SOURCE_DIR})
#INCLUDE_DIRECTORIES(${PROJECT_BINARY_DIR})
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/EXTERNALS/ais3dTools)
#INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/EXTERNALS/ais3dTools/EXTERNALS/g2o/include)
INCLUDE_DIRECTORIES(${CMAKE_BINARY_DIR}/include)
LINK_DIRECTORIES(${CMAKE_BINARY_DIR}/lib)
#LINK_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/EXTERNALS/ais3dTools/EXTERNALS/g2o/src/g2o/lib)

ADD_SUBDIRECTORY(EXTERNALS)
ADD_SUBDIRECTORY(narfPlaceRecognitionLib)
ADD_SUBDIRECTORY(narfPlaceRecognition)
