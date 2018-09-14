## Motoman の周りでカメラを動かす（手動）

#### 1. Moveit! + Gazebo + Rviz を起動

```
$ roslaunch motion_planning initialization_sia5.launch
```



#### 2. カメラモデルを Gazebo に登場させる

```
$cd /motoman_ws/src/kinect_bringup/urdf/models
```

##### urdf を使う場合

```
$ rosrun gazebo_ros spawn_model -urdf -file camera.urdf -model camera -x 1 -y 0 -z 0.5 -r 0.1 -p 0 -y 0.0
```

urdf そのままでは重力で下に落ちるので，Gazebo の画面の左にある World/Models/camera/camera_link で gravity の check を外す．

##### sdf を使う場合

```
$ rosrun gazebo_ros spawn_model -sdf -file camera.sdf -model camera -x 1 -y 0 -z 0.5 -r 0.1 -p 0 -y 0.0
```

sdf ではファイル内で重力無しに設定している．



### Kinect でやる場合

上記の手順において`camera.urdf` を`kinect.urdf`，`camera.sdf` を `kinect.sdf` に変えれば kinect で同じことができる． 



##### urdf から sdf への変換

```
$ gz sdf -p camera.urdf > camera.sdf
```



#### 3. rqt でトピック /gazebo/set_model_state にカメラの位置姿勢を指定する

```
$ rqt
```

Plugins →Topics →Message Publisher を起動し，トピック： /gazebo/set_model_state に下記の値を入力して Publish する．

```
 /gazebo/set_model_state
   model_name: 'camera'
   pose:
    position: 
      x: 0.6
      y: 0.8
      z: 0.7
    orientation: 
      x: 0.116377479004
      y: 0.101149084878
      z: -0.845668530279
      w: 0.510949979847
```

現在の値を確認したい時は下記コマンド．

```
$ rostopic echo -n 1 /gazebo/model_states
```



#### カメラモデルを Gazebo から消し去る

```
$ rosservice call gazebo/delete_model '{model_name: camera}'
```



#### xacro から urdf への変換

```
$ rosrun xacro xacro.py kinect.urdf.xacro > kinect.urdf
```

