cmake_minimum_required(VERSION 3.8 FATAL_ERROR)
variable_watch(VTK_DIR)
project(Seg1)

find_package(PCL 1.8.1 REQUIRED)
find_package(VTK REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (RegionGrowing RegionGrowing.cpp)
target_link_libraries (RegionGrowing ${PCL_LIBRARIES})
add_executable (MinCut MinCut.cpp)
target_link_libraries (MinCut ${PCL_LIBRARIES})
add_executable (ColorBasedRegionGrowing ColorBasedRegionGrowing.cpp)
target_link_libraries (ColorBasedRegionGrowing ${PCL_LIBRARIES})
add_executable (DifferenceOfNormalsEstimation DifferenceOfNormalsEstimation.cpp)
target_link_libraries (DifferenceOfNormalsEstimation ${PCL_LIBRARIES})
add_executable (CEC CEC.cpp)
target_link_libraries (CEC ${PCL_LIBRARIES})
add_executable (PCDViewer PCDViewer.cpp)
target_link_libraries (PCDViewer ${PCL_LIBRARIES})

