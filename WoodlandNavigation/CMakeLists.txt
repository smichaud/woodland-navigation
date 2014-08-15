project(WoodlandNavigation)
cmake_minimum_required(VERSION 2.8)

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


# Libpointmatcher
find_package(pointmatcher 1.1.0 REQUIRED)
include_directories("${POINTMATCHER_INCLUDE_DIRS}")
message(STATUS "Using libpointmatcher version ${pointmatcher_VERSION}")

# Ceres Solver
FIND_PACKAGE(Ceres REQUIRED)
INCLUDE_DIRECTORIES(${CERES_INCLUDE_DIRS})


# CMAKE_CURRENT_SOURCE_DIR: directory of the currently processed CMakeLists.txt
# GLOB_RECURSE: will search in all folder and subfolder
file(GLOB_RECURSE
     SOURCE_FILES
     ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
# Adding the headers to the "add_executable" show them in the IDE files tree
file(GLOB_RECURSE
     HEADER_FILES
     ${CMAKE_CURRENT_SOURCE_DIR}/src/*.h)
aux_source_directory(. SRC_LIST)

# Either put complete path to headers or call #include "partial/header.h"
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/datatype)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/util)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/thirdpartylibraries/alglib)

add_executable(${PROJECT_NAME}
               ${SRC_LIST}
               ${SOURCE_FILES}
               ${HEADER_FILES})

target_link_libraries(${PROJECT_NAME}
                      ${POINTMATCHER_LIBRARIES}
                      ${CERES_LIBRARIES})


# Include Google Mock/Test
include_directories($ENV{GMOCK_HOME}/include $ENV{GMOCK_HOME}/gtest/include)
link_directories($ENV{GMOCK_HOME}/build $ENV{GMOCK_HOME}/gtest/build)

file(GLOB_RECURSE
     TEST_SOURCE_FILES
     ${CMAKE_CURRENT_SOURCE_DIR}/test/*.cpp)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/test)

add_executable(test
               ${TEST_SOURCE_FILES}
               ${SOURCE_FILES}
               ${CMAKE_CURRENT_SOURCE_DIR}/test/)
target_link_libraries(test pthread)
target_link_libraries(test gmock)
target_link_libraries(test gtest)
target_link_libraries(test ${POINTMATCHER_LIBRARIES})

