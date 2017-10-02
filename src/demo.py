#!/usr/bin/env python
import roslib; roslib.load_manifest('ros_turtlesim_demo')
import rospy
from std_srvs.srv import Empty


if __name__ == '__main__':
    try:
        # Initialize node
        rospy.init_node('turtlebot_figure', anonymous=True)

        # Clear the screen
        rospy.wait_for_service('clear')
        clear = rospy.ServiceProxy('clear', Empty)
        clear()

    except rospy.ROSInterruptException: pass
