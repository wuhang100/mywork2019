cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(feature)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (feature main.cpp)
target_link_libraries (feature ${PCL_LIBRARIES})

install(TARGETS feature RUNTIME DESTINATION bin)

