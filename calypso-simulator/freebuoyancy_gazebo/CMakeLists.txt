cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

PROJECT(freebuoyancy_gazebo)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(freebuoyancy_gazebo SHARED
    src/freebuoyancy.cpp
    src/freebuoyancy.h
)

target_link_libraries(freebuoyancy_gazebo ${GAZEBO_LIBRARIES})

install(TARGETS freebuoyancy_gazebo DESTINATION ${GAZEBO_PLUGIN_PATH})