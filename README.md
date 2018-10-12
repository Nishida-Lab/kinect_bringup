# kinect_bringup

## Requirements
- Ubuntu 16.04
- ROS kinetic (including PCL)
- libfreenect2
- iai_kinect2
- Kinect v2

### libfreenect2 installation
```
$ cd
$ mkdir src
$ cd src
$ git clone https://github.com/OpenKinect/libfreenect2.git
$ cd libfreenect2
$ sudo apt-get install build-essential cmake pkg-config libusb-1.0-0-dev libturbojpeg libjpeg-turbo8-dev libglfw3-dev
$ mkdir build
$ cd build
$ cmake .. -DCMAKE_INSTALL_PREFIX=$HOME/src/freenect2
$ cmake .. -Dfreenect2_DIR=$HOME/src/freenect2/lib/cmake/freenect2
$ make -j8
$ sudo make install
$ sudo cp ../platform/linux/udev/90-kinect2.rules /etc/udev/rules.d/
```
#### Test of kinect v2 via libfreenect2
Connect the kinect to USB3.0 port.
```
./bin/Protonect
```

### iai_kinect2 installation
```
$ cd <your_workspace>/src
$ git clone https://github.com/code-iai/iai_kinect2.git
$ cd iai_kinect2
$ rosdep install -r --from-paths .
$ cd ~/catkin_ws
$ catkin build -DCMAKE_BUILD_TYPE=”Release”  -Dfreenect2_DIR=~/src/freenect2/lib/cmake/freenect2
```
#### Test of kinect v2 via ROS
Run the streaming script for kinect v2.
```
$ roslaunch kinect2_bridge kinect2_bridge.launch
```
pointcloud viewer
```
$ rosrun kinect2_viewer kinect2_viewer kinect2 sd cloud
```
image viewer
```
$ rosrun kinect2_viewer kinect2_viewer kinect2 sd image
```

## Initial build of this repository
```
$ cd <your_workspace>/src
$ git clone https://github.com/Nishida-Lab/kinect_bringup.git
```
Specify the location where you installed libfreenect2.
```
$ catkin build -DCMAKE_BUILD_TYPE=”Release”  -Dfreenect2_DIR=~/src/freenect2/lib/cmake/freenect2
```

## Running single kinect with urdf
```
$ roscore
```
```
$ roslaunch kinect2_bridge kinect2_bridge.launch base_name:=kinect_center
```
```
$ roslaunch kinect_bringup kinect_center_streaming.launch
```
### depth_method:=cpu
```
$ roslaunch kinect2_bridge kinect2_bridge.launch base_name:=kinect_center depth_method:=cpu
```
###  extrinsic calibration
```
$ rosrun kinect_bringup tf_interactive_marker.py world kinect_test 1.4784 0.0693 0.4358 0.046 0.4315 -3.1227
```
