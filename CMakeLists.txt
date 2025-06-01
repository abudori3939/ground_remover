cmake_minimum_required(VERSION 3.10)
project(ground_remover)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(PCL 1.8 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(ground_remover src/main.cpp)
target_link_libraries(ground_remover ${PCL_LIBRARIES})

if(NOT TARGET PCL::Common)
    message(WARNING "PCL::Common target not found, linking PCL libraries directly.")
    target_link_libraries(ground_remover ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_FILTERS_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES})
else()
    target_link_libraries(ground_remover PCL::Common PCL::IO PCL::Filters PCL::Visualization)
endif()
