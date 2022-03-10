cmake_minimum_required(VERSION 3.2)

project(DRRgenerator)

message("CMAKE_BUILD_TYPE is ${CMAKE_BUILD_TYPE}")

set(SOURCES main.cpp)

add_executable(DRRgenerator ${SOURCES})

target_include_directories(DRRgenerator PRIVATE src)
target_link_libraries(DRRgenerator PRIVATE DRR)
