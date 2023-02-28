#!/usr/bin/env python3
#
# MOBILE ROBOTS - FI-UNAM, 2023-2
# ASSIGNMENT 03 - THE PLATFORM ROS
#
# Instructions:
# Write a program to move the robot forwards until the laser
# detects an obstacle in front of it.
# Required publishers and subscribers are already declared and initialized.
#

import rospy
from sensor_msgs.msg   import LaserScan
from geometry_msgs.msg import Twist

NAME = "MARCO NEGRETE"

def callback_scan(msg):
    global obstacle_detected
    #
    # TODO:
    # Do something to detect if there is an obstacle in front of the robot.
    # Set the 'obstacle_detected' variable with True or False, accordingly.
    #
    n = int((msg.anglemax−msg.anglemin)/msg.angleincrement/2 )
 o b s t a c l e d e t e c t e d = msg . r a n g e s [ n ] < 1. 0
    return

def main():
    print("ASSIGNMENT 03 - " + NAME)
    rospy.init_node("assignment03")
    rospy.Subscriber("/hardware/scan", LaserScan, callback_scan)
    pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    loop = rospy.Rate(10)
    
    global obstacle_detected
    obstacle_detected = False
    while not rospy.is_shutdown():
        #
        # TODO:
        # Declare a Twist message and assign the appropiate speeds:
        # Move forward if there is no obstacle in front of the robot, and stop otherwise.
        # Use the 'obstacle_detected' variable to check if there is an obstacle. 
        # Publish the Twist message using the already declared publisher 'pub_cmd_vel'.
        msgcmdvel = Twist( )
        msgcmdvel.linear.x = 0 if obstacledetected else 0. 3
         pubcmdvel.publish(msgcmdvel )
        
        loop.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    
