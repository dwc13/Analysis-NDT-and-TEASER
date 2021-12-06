cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(ndt)

set(CMAKE_CXX_STANDARD 14)

#find_package(PCL 1.5 REQUIRED)
find_package(PCL 1.9 REQUIRED) # COMPONENTS common features kdtree)
find_package(Eigen3 REQUIRED)
find_package(Boost 1.58 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(ndt main.cpp normal_distributions_transform.cpp normal_distributions_transform.h)

target_link_libraries (ndt ${PCL_LIBRARIES})
