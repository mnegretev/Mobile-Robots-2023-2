#!/usr/bin/env python3
#
# MOBILE ROBOTS - FI-UNAM, 2023-1
# SPLIT AND MERGE ALGORITHM
#
# Instructions:
# Test different parameteres and compare the results.
# Parameters:
# 
#

import rospy
import numpy
import math
import tf
from sensor_msgs.msg   import LaserScan
from geometry_msgs.msg import Twist, Point
from visualization_msgs.msg import Marker

NAME = "Martinez Vazquez Brayan Alexis"

def adjust_line(points):
    [xm,ym] = numpy.mean(points, 0)
    n,d = 0,0
    for x,y in points:
        n += (xm - x)*(ym - y)
        d += (ym - y)**2 - (xm - x)**2
    theta = 0.5*math.atan2(-2*n , d)
    rho   = xm*math.cos(theta) + ym*math.sin(theta)
    length= numpy.linalg.norm(points[0] - points[-1])
    return rho, theta, xm, ym, length

def find_farthest_point(points, rho, theta):
    distances = [abs(points[i][0]*math.cos(theta) + points[i][1]*math.sin(theta) - rho) for i in range(len(points))]
    idx = numpy.argmax(distances)
    return idx, distances[idx]
        
def split(points, threshold, min_points):
    if len(points) < min_points:
        return []
    rho, theta, xm, ym, length = adjust_line(points)
    idx, dist  = find_farthest_point(points, rho, theta)
    if dist < threshold:
        return [[rho, theta, xm, ym, length]]
    lines1 = split(points[0:idx], threshold, min_points)
    lines2 = split(points[idx+1:len(points)], threshold, min_points)
    return lines1 + lines2

def merge(lines, rho_tol, theta_tol):
    if len(lines) < 2:
        return lines
    new_lines = []
    for i in range(1, len(lines)):
        rho1, theta1, xm1, ym1, length1 = lines[i]
        rho2, theta2, xm2, ym2, length2 = lines[i-1]
        e_rho   = abs((rho1 - rho2)/min(rho1, rho2))
        e_theta = abs(theta1 - theta2)
        if e_rho < rho_tol and e_theta < theta_tol:
            new_lines.append([(rho1+rho2)/2, (theta1+theta2)/2, (xm1+xm2)/2, (ym1+ym2)/2, length1+length2])
        else:
            new_lines.append([rho1, theta1, xm1, ym1, length1])
            new_lines.append([rho2, theta2, xm2, ym2, length2])
    return new_lines
            
def split_and_merge(points, threshold, min_points, rho_tol, theta_tol):
    lines = split(points, threshold, min_points)
    lines = merge(lines, rho_tol, theta_tol)
    return lines

def get_line_markers(lines):
    marker = Marker()
    marker.header.frame_id = "map";
    marker.header.stamp = rospy.Time.now();
    marker.ns = "segmented_lines";
    marker.id = 7;
    marker.type = Marker.LINE_LIST;
    marker.action = Marker.ADD;
    marker.scale.x = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    for [rho, theta, xm, ym, length] in lines:
        a  = math.cos(theta)
        b  = math.sin(theta)
        marker.points.append(Point(xm + length/2*(-b), ym + length/2*(a), 0.5))
        marker.points.append(Point(xm - length/2*(-b), ym - length/2*(a), 0.5))
    return marker

def get_laser_pose():
    global listener
    try:
        ([x, y, z], rot) = listener.lookupTransform('map', 'laser_link', rospy.Time(0))
        a = 2*math.atan2(rot[2], rot[3])
        a = a - 2*math.pi if a > math.pi else a
        return [x, y, z, a]
    except:
        pass
    return [0,0,0,0]

def callback_scan(msg):
    global obstacle_detected, pub_line_markers
    x,y,z,a = get_laser_pose()
    points = []
    for i in range(len(msg.ranges)):
        if not (math.isnan(msg.ranges[i]) or msg.ranges[i] >= msg.range_max):
            r,theta = msg.ranges[i], i*msg.angle_increment + msg.angle_min + a
            points.append([r*math.cos(theta) + x, r*math.sin(theta) + y])
    points = numpy.asarray(points)
    #
    # TODO:
    # Modify the following parameters and compare the results:
    #
    DISTANCE_THRESHOLD  = 0.03     #Distance threshold to consider a point as part of a candidate line. 
    MIN_POINTS_COUNTING = 4       #Minimum number of points a line should contain.
    RHO_TOLERANCE       = 0.00005    #RHO and THETA error tolerance to consider two lines as one.
    THETA_TOLERANCE     = 0.00005
    lines = split_and_merge(points, DISTANCE_THRESHOLD, MIN_POINTS_COUNTING, RHO_TOLERANCE, THETA_TOLERANCE)
    pub_line_markers.publish(get_line_markers(lines))
    return

def main():
    global pub_line_markers, listener
    print("INITIALIZING SPLIT AND MERGE ALGORITHM")
    rospy.init_node("split_and_merge")
    rospy.Subscriber("/hardware/scan", LaserScan, callback_scan)
    pub_line_markers = rospy.Publisher("/navigation/segmented_lines_marker", Marker, queue_size=1)
    listener = tf.TransformListener()
    loop = rospy.Rate(10)

    while not rospy.is_shutdown():
        loop.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    
