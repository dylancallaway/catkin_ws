cmake_minimum_required(VERSION 2.8.3)
project(dylan_msc)
catkin_package()

add_executable(imageProc1 src/imageProc1.cpp)

find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
target_link_libraries (imageProc1 ${PCL_LIBRARIES})

find_package(catkin REQUIRED roscpp std_msgs)
include_directories(include ${catkin_INCLUDE_DIRS})
target_link_libraries(imageProc1 ${catkin_LIBRARIES})

