cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(pfh)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (pfh main.cpp)
target_link_libraries (pfh ${PCL_LIBRARIES})

install(TARGETS pfh RUNTIME DESTINATION bin)
