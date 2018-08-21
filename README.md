# kinect_bringup
## Installation
Follow this [process](http://demura.net/%E6%9C%AA%E5%88%86%E9%A1%9E/13560.html) to install `libfreenect2` and `iai_kinect2`.
## Initial build
```
catkin build -DCMAKE_BUILD_TYPE=”Release”  -Dfreenect2_DIR=~/src/freenect2/lib/cmake/freenect2
```
## Running single kinect
```
roscore
```
```
roslaunch kinect2_bridge kinect2_bridge.launch base_name:=kinect_center
```
```
roslaunch kinect_bringup kinect_center_streaming.launch
```
