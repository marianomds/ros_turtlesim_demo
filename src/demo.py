#!/usr/bin/env python
import roslib; roslib.load_manifest('ros_turtlesim_demo')
import rospy
from geometry_msgs.msg  import Twist
from turtlesim.msg import Pose
from std_srvs.srv import Empty
from turtlesim.srv import TeleportAbsolute, SetPen
from math import pow,atan2,sqrt
import csv

# Star coordinates
starpoints = ((4,2),(6.25,3.25),(9,2),(8.25,4.5),(10,6.25),(7.5,6.5),(6.25,9),(5.5,6.5),(3,6.25),(4.75,4.5),(4,2))


# Function for obtaining the coordinates of a custom figure from a csv file
def getpoints(filename):
    reader = csv.DictReader(open(filename), delimiter=',')
    points = [] # The points' coordinates will be saved in a list of tuples
    for row in reader:
        points.append((ord(row['X']) - 48,ord(row['Y']) - 48)) # Each row has one point, whose coordinates are in a 'X' and a 'Y' column
    return points


# Class from which the turtle object will be instanced
class turtlebot():

	# Initialization funcion
    def __init__(self, ini_point): # The init funcion receives the initial point for positioning the turtle
    	# Create the velocity publisher
        self.velocity_publisher = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=10)

        # Create the position subscriber
        self.pose_subscriber = rospy.Subscriber('turtle1/pose', Pose, self.callback)
        self.pose = Pose()
        self.rate = rospy.Rate(10) # Refresh frequency [Hz]

        # Turn off the pen
        rospy.wait_for_service('turtle1/set_pen')
        set_pen = rospy.ServiceProxy('turtle1/set_pen', SetPen)
        set_pen(0,0,0,0,1)

        # Move the turtle to the initial position
        rospy.wait_for_service('turtle1/teleport_absolute')
        teleport = rospy.ServiceProxy('turtle1/teleport_absolute', TeleportAbsolute)
        teleport(ini_point[0],ini_point[1],0)

        # Turn on the pen
        set_pen(255,255,255,4,0)


    # Callback function formatting the pose value received
    def callback(self, data):
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)

    # Calculate the linear (euclidean) distance to the goal point
    def get_linear_distance(self, goal_x, goal_y):
        distance = sqrt(pow((goal_x - self.pose.x), 2) + pow((goal_y - self.pose.y), 2))
        return distance

    # Calculate the angular distance to the goal point
    def get_angular_distance(self, goal_x, goal_y):
        distance = atan2(goal_y - self.pose.y, goal_x - self.pose.x) - self.pose.theta
        return distance

    # Move the turtle to the specified point 
    def move(self, point): # Receives a tuple with the point coordinates
    	# Format the point coordinates
        goal_pose = Pose()
        goal_pose.x = point[0]
        goal_pose.y = point[1]

        # Create the velocity vector atribute
        self.vel_msg = Twist()

        # Linear velocity only in the x-axis
        self.vel_msg.linear.y = 0
        self.vel_msg.linear.z = 0

        # Angular velocity only in the z-axis
        self.vel_msg.angular.x = 0
        self.vel_msg.angular.y = 0
        
        # The turtle will first rotate => linear velocity = 0
        self.vel_msg.linear.x = 0

        # The loop will remain until the angular distance starts to increase again
        prev_angular_distance = 100
        angular_distance = self.get_angular_distance(goal_pose.x, goal_pose.y)
        while abs(angular_distance) <= abs(prev_angular_distance) and abs(angular_distance) > 0.001:

            # Porportional Controller
            self.vel_msg.angular.z = 2 * angular_distance

            # Publish the velocity
            self.velocity_publisher.publish(self.vel_msg)
            self.rate.sleep()

            # Re-calculate the distance
            prev_angular_distance = angular_distance
            angular_distance = self.get_angular_distance(goal_pose.x, goal_pose.y)


        # The turtle will now translate => angular velocity = 0
        self.vel_msg.angular.z = 0

        # The loop will remain until the linear distance starts to increase again
        prev_linear_distance = 100
        linear_distance = self.get_linear_distance(goal_pose.x, goal_pose.y)
        while linear_distance <= prev_linear_distance and linear_distance != 0:

            # Porportional Controller
            self.vel_msg.linear.x = 5 * linear_distance 

            # Publish the velocity
            self.velocity_publisher.publish(self.vel_msg)
            self.rate.sleep()

            # Re-calculate the distance
            prev_linear_distance = linear_distance
            linear_distance = self.get_linear_distance(goal_pose.x, goal_pose.y)
        
        # Stop turtle
        self.vel_msg.linear.x = 0
        self.vel_msg.angular.z =0
        self.velocity_publisher.publish(self.vel_msg)


if __name__ == '__main__':
    try:
        # Initialize node
        rospy.init_node('turtlebot_figure', anonymous=True)

        # Clear the screen
        rospy.wait_for_service('clear')
        clear = rospy.ServiceProxy('clear', Empty)
        clear()

        # The turtle can either draw a star (with pre-specified coordinates) or a custom figure (with coordinates specified in a csv file)
        ans = raw_input("Would you like turtlesim to draw a star (s) or some custom figure (f)?:")
        if (ans == 's') or (ans == 'S'):
            points = starpoints # Use the pre-specified star coordinates
        elif (ans == 'f') or (ans == 'F'):
            filename = rospy.get_param('~filename') # run the node with parameter: _filename:=full_path_filename
            points = getpoints(filename) # Use the custom figure coordinates

        # Instance a turtle objet       
        x = turtlebot(points[0])

        # Move the turtle object to each point
        for p in points:
            x.move(p)

    except rospy.ROSInterruptException: pass
