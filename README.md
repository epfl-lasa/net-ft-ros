# net-ft-ros
[![Build Status](https://travis-ci.org/epfl-lasa/net-ft-ros.svg?branch=master)](https://travis-ci.org/epfl-lasa/net-ft-ros)

ROS package for the ATI 6-axis force torque sensor.

## Hardware setup

TODO: description of the ATI setup, with ATI box, how to connect to ethernet and ping the box.

Once you have connected the ATI box with an ethernet and power supply cable, check 
the network connection. From your PC try and ping the FT sensor.
```
$ ping 128.178.145.98
```
If you can ping the FT sensor then you are ready to use it. You can also put the ip into your web-brower 
and (if you could ping it) you should be able to see a webpage which shows the status of the FT. Normaly
the status should be: Healthy.

## Launch

```
$ roslaunch netft_rdt_driver ft_sensor.launch 
```

The force-torque information will be published on the topic: **/ft_sensor/netft_data**.
The message type is **geometry_msgs/WrenchStamped**.
```
rostopic echo /ft_sensor/netft_data


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

There are four arguments the node takes, see the  [**launch file**](https://github.com/epfl-lasa/net-ft-ros/blob/master/launch/ft_sensor.launch)  for more details.

* **ROS service**
```
 rosservice call /ft_sensor/bias_cmd "cmd: 'bias'"
```
To call the service in C++ code make sure to include the service message type:
```
#include "netft_rdt_driver/String_cmd.h"
...
ros::ServiceClient ft_client = nh.serviceClient<netft_rdt_driver::String_cmd>("/ft_sensor/bias_cmd");
netft_rdt_driver::String_cmd srv;
srv.request.cmd  = "bias";
srv.response.res = "";
f (ft_client.call(srv))
{
  ROS_INFO_STREAM("net_ft res: " << srv.response.res);
}else{
  ROS_ERROR("Failed to call netft bias service");
}
```


* **rqt_plot**
You can visualise the force-torque sensor topic with the rqt perspective. In the 
launch file run the script rqt script:
```
./launch_rqt.sh
```

* **Rviz**
Add a [**Wrench**](http://wiki.ros.org/rviz/DisplayTypes/Wrench) message type in Rviz and make sure
it is subscribing to the appropriate ros message, namely **/ft_sensor/netft_data** 


:bangbang: The node publishes force-torque data in a left-handed system (LHS).

