cmake_minimum_required(VERSION 3.7)
project(pcl)

add_definitions(-DDEBUG)

#set(CMAKE_CXX_STANDARD 11)

find_package(PCL)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(SOURCE_FILES main.cpp)
add_executable(pcl ${SOURCE_FILES})
target_link_libraries (pcl ${PCL_LIBRARIES})

set(TEST_FILE test.cpp)
add_executable(mytest ${TEST_FILE})
target_link_libraries(mytest ${PCL_LIBRARIES})

set(TEST_FILE test1.cpp)
add_executable(mytest1 ${TEST_FILE})
target_link_libraries(mytest1 ${PCL_LIBRARIES})

set(TEST_FILE Process.cpp)
add_executable(process ${TEST_FILE})
target_link_libraries(process ${PCL_LIBRARIES})