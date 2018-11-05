# kinect_bringup
## Python Scripts
###  extrinsic calibration
```
$ rosrun kinect_bringup tf_interactive_marker.py world kinect_test 1.4784 0.0693 0.4358 0.046 0.4315 -3.1227
```
###  image pixel to pointcloud projection
- image topic : /kinect_head/qhd/image_color
- pointcloud topic : /kinect_head/qhd/points
```
$ rosrun kinect_bringup image_pixel_to_3d_point.py
```
```
$ rosrun kinect_bringup specify_image_pixel.py 
```
type key `w`,`s`,`a`,`d` on the image window.
