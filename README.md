# Underwater_SLAM

A SLAM system based on Delayed State Extended Kalman Filter to map in underwater environment. 

## Getting Started

For more details about the algorithm, please visit [this page](https://yezhenzhao.github.io/underwater_slam). 

### Prerequisites

* [ROS(kinetic)](http://wiki.ros.org/kinetic)

* [uwsim](http://wiki.ros.org/uwsim) 
* [joy](http://wiki.ros.org/joy)

### Dependence 

* PCL 1.8 
* Eigen3 
* Boost 

### Build the Package

Use [catkin_make](http://wiki.ros.org/catkin/commands/catkin_make) to build all the package.  

## Running the tests

Initialize the ROS. 

```
roscore 
```

Launch the uwsim.

```
bash bash/uwsim.sh
```

Launch all package.

```bash
rosrun joy joy_node
rosrun underwater_slam controller
rosrun underwater_slam request_control
rosrun underwater_slam ekf_slam
```

## Authors

* [**Yezhen Zhao**](https://github.com/yezhenzhao) - *Initial work* 
