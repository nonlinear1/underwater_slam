cmake_minimum_required(VERSION 2.8.3)
project(underwater_slam)

add_definitions(-std=c++11)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake_module/")

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
  pcl_ros
  message_generation
)

find_package(PCL 1.8  REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(G2O REQUIRED)

add_service_files(
  FILES 
  PointDetection.srv
  RequireControl.srv
  )

add_message_files(
  FILES
  Node.msg
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
  geometry_msgs
  )

catkin_package(
  DEPENDS Boost
  CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  include/
)

add_library(
  EKF_lib
  STATIC 
  lib/EKF_core.cpp
  lib/accumulater.cpp
  )

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(laser_accumulater src/laser_accumulater.cpp)
add_executable(point_detection src/point_detection.cpp)
add_executable(request_control src/request_control.cpp)
add_executable(process src/process.cpp)
add_executable(controller src/boat_tele.cpp)
add_executable(scan src/scan.cpp)
add_executable(front_end src/front_end.cpp)
add_executable(back_end src/back_end.cpp)
add_executable(slam src/slam.cpp)

  target_link_libraries(laser_accumulater
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${Boost_LIBRARIES}
   EKF_lib
  )
  
  target_link_libraries(point_detection
    ${catkin_LIBRARIES}
  )
  
  target_link_libraries(process
    ${catkin_LIBRARIES}
    EKF_lib
  )
  target_link_libraries(request_control
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
  )
  target_link_libraries(controller
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
  )
  target_link_libraries(scan
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
  )
  target_link_libraries(front_end
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${Boost_LIBRARIES}
  )

 target_link_libraries(back_end
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
 )

 target_link_libraries(slam
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  )
