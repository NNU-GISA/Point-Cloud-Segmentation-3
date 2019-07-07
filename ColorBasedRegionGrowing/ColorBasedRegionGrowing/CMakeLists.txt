cmake_minimum_required(VERSION 3.8 FATAL_ERROR)

project(ColorBasedRegionGrowing)

find_package(PCL 1.6.0 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (ColorBasedRegionGrowing ColorBasedRegionGrowing.cpp)
target_link_libraries (ColorBasedRegionGrowing ${PCL_LIBRARIES})
