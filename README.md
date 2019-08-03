# Line Follower - PID controller with perception(OpevCV2)

[OpenCV_bridge](http://wiki.ros.org/cv_bridge) package allows the ROS imaging topics to use the OpenCV image variable format.

OpenCV images come in BGR image format, while regular ROS images are in the more standard RGB encoding. OpenCV_bridge provides a nice feature to convert between them. Also, there are many other functions to transfer images to OpenCV variables transparently.

The robot follows the yellow line. This task can be divided into the following phases:

-   **Step 01** Geting images from the ROS topic and convert them into OpenCV format
-   **Step 02** Processing the images using OpenCV libraries to obtain the data we want for the task
-   **Step 03** Moving the robot along the yellow line, based on the data obtained

![enter image description here](https://i.ibb.co/qknybTp/perception-unit2-linefollower1.png)

![enter image description here](https://i.ibb.co/sbgcMkb/perception-unit2-map.png)


## PID controller with perception

One way of controlling the movement of the robot a little better and making it smoother is to apply a PID controller to the control values. There is a  [PID ROS package](http://wiki.ros.org/pid)  that makes using PIDs much easier.

PID Controller is implemented in ```follow_line_step_pid.py``` file. Its launch file is ```follow_line_with_pid.launch```