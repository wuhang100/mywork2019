cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(sydney_pcl)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (sydney_pcl main.cpp)
target_link_libraries (sydney_pcl ${PCL_LIBRARIES})

install(TARGETS sydney_pcl RUNTIME DESTINATION bin)

