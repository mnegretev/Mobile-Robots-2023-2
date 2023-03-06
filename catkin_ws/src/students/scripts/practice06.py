#!/usr/bin/env python3
#
# MOBILE ROBOTS - FI-UNAM, 2023-2
# PRACTICE 06 - LOCALIZATION BY KALMAN FILTER
#
# Instructions:
#

import rospy
import tf
import math
from std_msgs.msg import Header
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan

NAME = "APELLIDO_PATERNO_APELLIDO_MATERNO"

def main():
    global listener, pub_cmd_vel, pub_markers
    print("PRACTICE 06 - " + NAME)
    rospy.init_node("practice06")
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    
