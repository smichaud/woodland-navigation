project(WoodlandNavigation)
cmake_minimum_required(VERSION 2.8)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/build)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)
# set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# Set compiler to C++11
add_definitions(-std=c++0x)

#Set the warning level (all)
if(MSVC)
  # Force to always compile with W4
  if(CMAKE_CXX_FLAGS MATCHES "/W[0-4]")
    string(REGEX REPLACE "/W[0-4]" "/W4" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
  else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4")
  endif()
elseif(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
  # Update if necessary
  set(
    CMAKE_CXX_FLAGS
    "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wno-long-long -pedantic -Wconversion")
endif()

# PCL
find_package(PCL 1.7 REQUIRED COMPONENTS common visualization)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Libpointmatcher
find_package(pointmatcher 1.1.0 REQUIRED)
include_directories("${POINTMATCHER_INCLUDE_DIRS}")
message(STATUS "Using libpointmatcher version ${pointmatcher_VERSION}")

# Ceres Solver
find_package(Ceres REQUIRED)
link_directories(${CERES_INCLUDE_DIRS})

# Alglib
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/thirdpartylibraries/alglib/include)
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/thirdpartylibraries/alglib/lib)

# Either put complete path to headers or call #include "partial/header.h"
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/datatype)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/util)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/placerecognition)

# CMAKE_CURRENT_SOURCE_DIR: directory of the currently processed CMakeLists.txt
# GLOB_RECURSE: will search in all folder and subfolder
file(GLOB_RECURSE
     SOURCE_FILES
     ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
# Adding the headers to the "add_executable" show them in the IDE files tree
file(GLOB_RECURSE
     HEADER_FILES
     ${CMAKE_CURRENT_SOURCE_DIR}/src/*.h)

add_executable(${PROJECT_NAME}
               ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
               ${SOURCE_FILES}
               ${HEADER_FILES})

target_link_libraries(${PROJECT_NAME}
                      ${POINTMATCHER_LIBRARIES}
                      ${CERES_LIBRARIES}
                      ${PCL_COMMONLIBRARIES}
                      ${PCL_VISUALIZATION_LIBRARIES}
                      alglib)


# Include Google Mock/Test
include_directories($ENV{GMOCK_HOME}/include $ENV{GMOCK_HOME}/gtest/include)
link_directories($ENV{GMOCK_HOME}/build $ENV{GMOCK_HOME}/gtest/build)

file(GLOB_RECURSE
     TEST_SOURCE_FILES
     ${CMAKE_CURRENT_SOURCE_DIR}/test/*.cpp)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/test)

add_executable(test
               ${CMAKE_CURRENT_SOURCE_DIR}/test.cpp
               ${TEST_SOURCE_FILES}
               ${SOURCE_FILES})
target_link_libraries(test
                      pthread
                      gmock
                      gtest
                      ${POINTMATCHER_LIBRARIES}
                      ${PCL_COMMONLIBRARIES}
                      ${PCL_VISUALIZATION_LIBRARIES}
                      alglib)

