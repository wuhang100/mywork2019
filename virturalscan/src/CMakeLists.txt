cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(virtual_cam)

find_package(PCL 1.2 REQUIRED)
find_package(OpenCV 3.2 REQUIRED )
find_package(Boost REQUIRED)
find_package(VTK REQUIRED)

if(NOT VTK_FOUND)
    message("VTK library Not found")
endif()

include_directories(${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
include(${VTK_USE_FILE})

add_library(obj_scan SHARED obj_scan.cpp)

add_executable (virtual_cam main.cpp)
target_link_libraries (virtual_cam obj_scan ${VTK_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

install(TARGETS virtual_cam RUNTIME DESTINATION bin)
