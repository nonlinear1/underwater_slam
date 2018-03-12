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
        pcl_ros
        message_generation
        )

find_package(PCL 1.8 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

add_service_files(
        FILES
        RequireControl.srv
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
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CSPARSE_INCLUDE_DIR}
        include/
)

add_library(
        EKF_file
        STATIC
        src/ScanBuilder.cpp
        src/EkfSlam.cpp
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(request_control src/request_control.cpp)
add_executable(controller src/boat_tele.cpp)
add_executable(ekf_slam src/underwater_slam.cpp)
add_executable(ground_true src/show_path.cpp)

target_link_libraries(ground_true
        ${catkin_LIBRARIES}
        )

target_link_libraries(request_control
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
        )

target_link_libraries(controller
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
        )

target_link_libraries(ekf_slam
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${Boost_LIBRARIES}
        EKF_file)