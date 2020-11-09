
# Launch file

There are four default arguments which you can set.

* **ip_address**:
This is the static IP address of NET-FT box and is always set to 128.178.145.98. You should not have to change
this variable. 
* **bias**: 
True or False, if you want the bias of FT to be computed and substracted when the node is launched
* **frame_id**:
The net-ft-ros node published a [**Wrench**](http://wiki.ros.org/rviz/DisplayTypes/Wrench) message which can be 
visualised in Rviz. The frame of frame_id is the reference frame to which this Rviz wrench message will be draw on.
Typically you would set the frame_id to the last link of the robot where the FT sensor is attached.
* **alpha**: exponential filter value range is [0,1].
* **rot**:
This variable performs a rotation around the z-axis of the force and torque vectors. On the FT sensor there is a little
dent which is a reference point which should be aligned with your world frame of reference. If it is not, because of how 
it is fixed to the robot, the rot variable allows you to make the necessary rotational correction·
* **scale**: values which are multiplied to the force vector, typical values would be either 1 or -1.

