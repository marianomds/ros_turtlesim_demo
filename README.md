# ros_turtlesim_demo
ROS node for making turtlesim draw a figure.

## Steps for building and running the demo:

Create a catkin workspace:
http://wiki.ros.org/catkin/Tutorials/create_a_workspace

In the catkin workspace, clone the repository and use catkin_create_pkg to create a new package called 'ros_turtlesim_demo':
```bash
cd ~/catkin_ws/src
git clone https://github.com/marianomds/ros_turtlesim_demo.git
catkin_create_pkg ros_turtlesim_demo geometry_msgs rospy
```

Build the packages in the catkin workspace:
```bash
cd ~/catkin_ws
catkin_make
```

To add the workspace to your ROS environment you need to source the generated setup file:
```bash
. ~/catkin_ws/devel/setup.bash
```

Start the roscore:
```bash
roscore
```

In a new Terminal run turtlesim:
```bash
rosmake turtlesim
rosrun turtlesim turtlesim_node
```

In a new Termial run the node:
```bash
source ~/catkin_ws/devel/setup.bash
rosrun ros_turtlesim_demo demo.py
```
