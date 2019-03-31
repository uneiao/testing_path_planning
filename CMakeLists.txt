cmake_minimum_required (VERSION 2.6)
project (MotionPlanning)
# The version number.
set (MotionPlanning_VERSION_MAJOR 1)
set (MotionPlanning_VERSION_MINOR 0)
set(CMAKE_BUILD_TYPE Debug)
SET(CMAKE_C_FLAGS "-g")
SET(CMAKE_CXX_FLAGS "-g")

set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/bin)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})
include_directories("${PROJECT_SOURCE_DIR}")

#add_library(Eigen ${CMAKE_SOURCE_DIR}/Eigen)
#add_subdirectory(Eigen)

add_library(mp STATIC ${PROJECT_SOURCE_DIR}/rrt.cpp ${PROJECT_SOURCE_DIR}/map.cpp)

add_executable(mp_test main.cpp)
target_link_libraries(
    mp_test
    mp
)
