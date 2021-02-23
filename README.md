# ros2-marvelmind
ROS2 Foxy Marvelmind publisher node

## Installation
Tested on Ubuntu 5.4.0-65-generic

Go to your workspace :
`cd your_workspace/src`

Clone workspace :
`git clone https://github.com/Nykri/ros2-marvelmind.git`

Build workspace :
```
cd your_workspace/
source install/setup.bash
colcon build
```

## Launch
Source workspace (for every new terminal) :
`source install/setup.bash`

Run the publisher node :
`ros2 run ros2-marvelmind marvelmind_publisher`

### Node an topics overview
![Rosgraph](Documents/rosgraph.png?raw=true "Node an topics overview")

3 topics are used :
 - /accel_marvelmind_readings [geometry_msgs/msg/Accel](http://docs.ros.org/en/api/geometry_msgs/html/msg/Accel.html)
 - /pose_marvelmind_readings [geometry_msgs/msg/Pose](http://docs.ros.org/en/api/geometry_msgs/html/msg/Pose.html)
 - /twist_marvelmind_readings [geometry_msgs/msg/Twist](http://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html)
