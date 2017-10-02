#!/usr/bin/env python
import roslib; roslib.load_manifest('ros_turtlesim_demo')
import rospy
from geometry_msgs.msg  import Twist
from turtlesim.msg import Pose
from std_srvs.srv import Empty

# Star coordinates
points = ((6,9),(2,3),(5,6))

# Class from which the turtle object will be instanced
class turtlebot():

	# Initialization funcion
    def __init__(self):
    	# Create the velocity publisher
        self.velocity_publisher = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=10)

        # Create the position subscriber
        self.pose_subscriber = rospy.Subscriber('turtle1/pose', Pose, self.callback)
        self.pose = Pose()
        self.rate = rospy.Rate(10) # Refresh frequency [Hz]

    # Callback function formatting the pose value received
    def callback(self, data):
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)

    # Move the turtle to the specified point 
    def move(self, point): # Receives a tuple with the point coordinates
    	# Format the point coordinates
        goal_pose = Pose()
        goal_pose.x = point[0]
        goal_pose.y = point[1]

        print(goal_pose)


if __name__ == '__main__':
    try:
        # Initialize node
        rospy.init_node('turtlebot_figure', anonymous=True)

        # Clear the screen
        rospy.wait_for_service('clear')
        clear = rospy.ServiceProxy('clear', Empty)
        clear()

        # Instance a turtle objet       
        x = turtlebot()

        # Move the turtle object to each point
        for p in points:
            x.move(p)

    except rospy.ROSInterruptException: pass
