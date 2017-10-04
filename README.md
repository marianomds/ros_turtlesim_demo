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

In a new Termial run the node (_filename parameter is only needed when drawing a custom figure):
```bash
source ~/catkin_ws/devel/setup.bash
rosrun ros_turtlesim_demo demo.py _filename:=full_path_CSVfilename
```

## Drawing a custom figure

The _filename parameter is not required when drawing the pre-defined star. If the user wants to draw a custom figure instead, _filename should be specified with the full path to the CSV file containing its definition.

The figure should be defined in a CSV file with two columns (one for the 'X' values and one for the 'Y' values) and with as many rows as points in the figure. For example:

X,Y  
5,6  
3,8  
1,5  
6,6  

The turtle can only draw figures with staight lines between the specified points (not curves).
