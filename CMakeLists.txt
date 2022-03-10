cmake_minimum_required(VERSION 3.2)

project(DRRgenerator)

message("CMAKE_BUILD_TYPE is ${CMAKE_BUILD_TYPE}")

find_package(OpenMP)

if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

find_package(OpenCV REQUIRED)

add_executable(${PROJECT_NAME} main.cpp DRRgenerator.cpp )

target_compile_options(DRRgeberatir PUBLIC -std=c++11 -O3)

target_include_directories(DRRgenerator PRIVATE ${OpenCV_INCLUDE_DIRS})
target_include_directories(DRRgenerator PRIVATE ${EIGEN3_INCLUDE_DIR})

target_link_libraries(DRRgenerator PRIVATE ${OpenCV_LIBS})


