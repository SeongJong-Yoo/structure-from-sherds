cmake_minimum_required(VERSION 3.11)
project(SfS-1.1)

# Ceres
find_package(Ceres CONFIG REQUIRED)

# PCL  
find_package(PCL CONFIG REQUIRED)  
find_package(PCL CONFIG REQUIRED COMPONENTS common io)  

#OpenMP
find_package(OpenMP)

set(CMAKE_CXX_STANDARD 17)

file(GLOB PROJ_SRC "*.cpp", "*.h", "class/*.cpp", "class/*.h")
add_executable(Hierarchy-Clear ${PROJ_SRC})

if (MSVC)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj")
  add_definitions(-D _CRT_SECURE_NO_WARNINGS)
endif()

if(OpenMP_CXX_FOUND)
    target_link_libraries(Hierarchy-Clear 
        PUBLIC 
            OpenMP::OpenMP_CXX
    )
endif()
target_link_libraries(SfS-1.1
    PRIVATE
        Ceres::ceres
        ${PCL_LIBRARIES}
)
target_include_directories(SfS-1.1
    PUBLIC
        ${CMAKE_SOURCE_DIR}/class
)
