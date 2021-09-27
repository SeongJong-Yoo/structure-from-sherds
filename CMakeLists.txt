cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(SfS-1.1)

# Ceres
find_package(Ceres ${SYSTEM_PACKAGE_REQUIRED})
if(CERES_FOUND)
	include_directories(${CERES_INCLUDE_DIRS})
	link_directories(${CERES_LIBRARY_DIRS})
	add_definitions(${CERES_DEFINITIONS})
endif()

# PCL
find_package(PCL 1.9.1 REQUIRED)  
find_package(PCL 1.9.1 REQUIRED COMPONENTS common io)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(SfS-1.1 main.cpp
class/data_path.h
class/axis_estimation.cpp 
class/axis_estimation.h
class/data_structure.cpp 
class/data_structure.h
class/feature_matching.cpp 
class/feature_matching.h
class/filter.cpp 
class/filter.h
class/KDTree.cpp 
class/KDTree.h
class/ranking_system.cpp 
class/ranking_system.h
class/reconstruction.cpp 
class/reconstruction.h
class/visualize.cpp 
class/visualize.h
)
target_link_libraries(SfS-1.1 ${PCL_LIBRARIES})
target_link_libraries(SfS-1.1 ceres)