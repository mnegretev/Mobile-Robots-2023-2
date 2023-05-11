#!/usr/bin/env python3
#
# MOBILE ROBOTS - FI-UNAM, 2023-2
# PRACTICE 06 - COLOR SEGMENTATION
#
# Instructions:
# Complete the code to estimate the position of an object 
# given a colored point cloud using color segmentation.
#

import numpy
import cv2
import ros_numpy
import rospy
import math
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PointStamped, Point
from custom_msgs.srv import FindObject, FindObjectResponse

NAME = "PERALES RODRIGUEZ DAVID"

def segment_by_color(img_bgr, points, obj_name):
    #
    # TODO:
    # - Assign lower and upper color limits according to the requested object:
    #   If obj_name == 'pringles': [25, 50, 50] - [35, 255, 255]
    #   otherwise                : [10,200, 50] - [20, 255, 255]
    # - Change color space from RGB to HSV.
    #   Check online documentation for cv2.cvtColor function
    # - Determine the pixels whose color is in the selected color range.
    #   Check online documentation for cv2.inRange
    # - Calculate the centroid of all pixels in the given color range (ball position).
    #   Check online documentation for cv2.findNonZero and cv2.mean
    # - Calculate the centroid of the segmented region in the cartesian space
    #   using the point cloud 'points'. Use numpy array notation to process the point cloud data.
    #   Example: 'points[240,320][1]' gets the 'y' value of the point corresponding to
    #   the pixel in the center of the image.
    # - Return a tuple of the form: [img_x, img_y, centroid_x, centroid_y, centroid_z]
    #   where img_x, img_y are the center of the object in image coordinates and
    #   centroid_x, y, z are the center of the object in cartesian coordinates. 
    #

    lower = [27, 50, 50] if obj_name == "pringles" else [10, 200, 50] # Lower color limit for both objects
    upper = [35, 255, 255] if obj_name == "pringles" else [20, 255, 255] # Upper color limit for both objects
    lower = numpy.asarray(lower) # Convert to numpy array
    upper = numpy.asarray(upper) 
    img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV) # Transform to HSV code
    img_bin = cv2.inRange(img_hsv, lower, upper) # Pixel selection from color range
    # Erosion and dilatation
    erosion_size = 3 # Kernel size for both operations (2*n+1)
    erosion_shape = cv2.MORPH_ELLIPSE  # Shape of kernel (circle)
    element = cv2.getStructuringElement(erosion_shape, (2 * erosion_size + 1, 2 * erosion_size + 1)) # Construction of kernel
    img_bin = cv2.dilate(cv2.erode(img_bin, element),element) # Opening operation 
    cv2.imshow("bin",img_bin) # Displays B&W image of selected color range
    # Centroids
    idx = cv2.findNonZero(img_bin) # Pixel indexes which are not black
    mean_img = cv2.mean(idx) # Centroid of reconginzed object in 2D image
 
    counter = 0 # Counter for mean
    xt, yt, zt = 0, 0, 0 # Coordenates in 3D space
    
    for [[c, r]] in idx: # Point cloud loop
        xt, yt, zt = xt + points[r, c][0], yt + points[r, c][1], zt + points[r, c][2] # Rearrangement of columns and rows
        counter +=1
    
    xt, yt, zt = xt/counter, yt/counter, zt/counter # Centroid of region in 3D space.

    return [mean_img[0],mean_img[1],xt,yt,zt] # Centroid in 2D image, centroid in 3D space.

def callback_find_object(req):
    global pub_point, img_bgr
    print("Trying to find object: " + req.name)
    arr = ros_numpy.point_cloud2.pointcloud2_to_array(req.cloud)
    rgb_arr = arr['rgb'].copy()
    rgb_arr.dtype = numpy.uint32
    r,g,b = ((rgb_arr >> 16) & 255), ((rgb_arr >> 8) & 255), (rgb_arr & 255)
    img_bgr = cv2.merge((numpy.asarray(b,dtype='uint8'),numpy.asarray(g,dtype='uint8'),numpy.asarray(r,dtype='uint8')))
    [r, c, x, y, z] = segment_by_color(img_bgr, arr, req.name)
    hdr = Header(frame_id='realsense_link', stamp=rospy.Time.now())
    pub_point.publish(PointStamped(header=hdr, point=Point(x=x, y=y, z=z)))
    cv2.circle(img_bgr, (int(r), int(c)), 20, [0, 255, 0], thickness=3)
    resp = FindObjectResponse()
    resp.x, resp.y, resp.z = x, y, z
    return resp

def main():
    global pub_point, img_bgr
    print("PRACTICE 06 - " + NAME)
    rospy.init_node("color_segmentation")
    rospy.Service("/vision/find_object", FindObject, callback_find_object)
    pub_point = rospy.Publisher('/detected_object', PointStamped, queue_size=10)
    img_bgr = numpy.zeros((480, 640, 3), numpy.uint8)
    loop = rospy.Rate(10)
    while not rospy.is_shutdown():
        cv2.imshow("Color Segmentation", img_bgr)
        cv2.waitKey(1)
        loop.sleep()
    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
