cmake_minimum_required(VERSION 3.15)
project(DetectWithLidar)

set(CMAKE_CXX_STANDARD 17)

#PCL CMake Settings
find_package(PCL REQUIRED)
set(PCL_DIR /usr/lib/x86_64-linux-gnu/cmake/pcl)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(DetectWithLidar main.cpp)

target_link_libraries(DetectWithLidar ${PCL_LIBRARIES} )