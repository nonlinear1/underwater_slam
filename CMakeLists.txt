cmake_minimum_required(VERSION 2.8.3)
project(underwater_slam)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  laser_geometry
  tf
  image_transport
  cv_bridge
  geometry_msgs
  underwater_sensor_msgs
  nav_msgs
)

find_package(PCL 1.8  REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

add_service_files(
  FILES 
  PointDetection.srv
  RequireControl.srv
  )
generate_messages(
  DEPENDENCIES
  sensor_msgs
  geometry_msgs
  )

catkin_package(
  DEPENDS Boost
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  include/
)

add_library(
  EKF_core
  STATIC 
  lib/EKF_core.cpp
  )

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(laser_convey_now src/laser_convey_now.cpp)
add_executable(save_pcl src/save_pcl.cpp)
add_executable(point_detection src/point_detection.cpp)
add_executable(request_control src/request_control.cpp)
add_executable(process src/process.cpp)

 target_link_libraries(laser_convey
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${Boost_LIBRARIES}
 )
  target_link_libraries(laser_convey_now
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${Boost_LIBRARIES}
 )
  target_link_libraries(pcl_reg
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${Boost_LIBRARIES}
 )
  target_link_libraries(pcl_add
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${Boost_LIBRARIES}
 )
  target_link_libraries(save_pcl
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
 )
  target_link_libraries(point_detection
    ${catkin_LIBRARIES}
  )
  target_link_libraries(process
    ${catkin_LIBRARIES}
    EKF_core
  )
  target_link_libraries(request_control
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
  )
