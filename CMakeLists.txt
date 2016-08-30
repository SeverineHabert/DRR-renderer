project(DRRgenerator)
cmake_minimum_required(VERSION 2.8)
ADD_DEFINITIONS(
    -std=c++11
 -fpermissive -O3
# Or -std=c++0x
    # Other flags
)
find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
add_executable(${PROJECT_NAME} main.cpp DRRgenerator.cpp )
target_link_libraries( ${PROJECT_NAME}   ${OpenCV_LIBS}  )


