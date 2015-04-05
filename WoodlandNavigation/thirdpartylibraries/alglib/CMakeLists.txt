project(alglib)
cmake_minimum_required(VERSION 2.8)
set(CMAKE_BUILD_TYPE Release)

# Set compiler to C++11
add_definitions(-std=c++0x)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/build)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/lib)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
file(GLOB_RECURSE SOURCE_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)

add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES})
target_include_directories (${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
