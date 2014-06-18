project(WoodlandNavigation)
cmake_minimum_required(VERSION 2.8)

#set(EXECUTABLE_OUTPUT_PATH build/${CMAKE_BUILD_TYPE})

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


# Libpointmatcher 
find_package(pointmatcher 1.1.0 REQUIRED)
include_directories("${POINTMATCHER_INCLUDE_DIRS}")
message(STATUS "Using libpointmatcher version ${pointmatcher_VERSION}")

# Either put complete path to headers or call #include "partial/header.h"
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/datatype)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/processing)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/util)

add_executable(${PROJECT_NAME}
               ${SRC_LIST}
               ${SOURCE_FILES}
               ${HEADER_FILES})

target_link_libraries(${PROJECT_NAME} ${POINTMATCHER_LIBRARIES})
