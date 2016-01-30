# net-ft-ros
ROS package for the ATI 6-axis force torque sensor.

## Hardware setup

TODO: description of the ATI setup, with ATI box, how to connect to ethernet and ping the box.

Once you have connected the ATI box with an ethernet cable and power supply you should first proceed to check 
that you have a correctly working networking communication between your PC and the ATI FT sensor.

```
$ ping 128.178.145.98
```
If you can ping the FT sensor then you are ready to use it.

## Launch

```
$ roslaunch ft_sensor.launch 
```

The force-torque information will be published on the topic: **geometry_msgs/WrenchStamped**
```
header: 
  seq: 9377
  stamp: 
    secs: 1454162697
    nsecs: 667636236
  frame_id: ''
wrench: 
  force: 
    x: -0.248077
    y: -1.12601
    z: -1.639932
  torque: 
    x: -0.106468
    y: -0.108128
    z: 0.22924
```

There are four arguments the node takes, see the  [**launch file*](https://github.com/epfl-lasa/net-ft-ros/blob/master/launch/ft_sensor.launch)  for more details.

* ROS service
```
 rosservice call /ft_sensor/bias_cmd "cmd: 'bias'"
```

* Visualisation


* Rviz
* 


