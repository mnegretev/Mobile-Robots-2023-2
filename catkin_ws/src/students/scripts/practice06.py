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

NAME = "Cruz Carrizosa Samael Xecotcovach"

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
    color_inf= [25,50,50] if obj_name == 'pringles' else [12,180,140]
    color_sup= [35,255,255] if obj_name == 'pringles' else [16,220,210]
    color_inf = numpy.asarray(color_inf)
    color_sup = numpy.asarray(color_sup)

    img_blur=cv2.GaussianBlur(img_bgr,(5,5),0) # suavizado 
    img_hsv = cv2.cvtColor(img_blur,cv2.COLOR_BGR2HSV)  # BGR a HSV
    img_bin = cv2.inRange(img_hsv,color_inf,color_sup) #imagen segmentada

    # Eliminacion de outliers con erosion y dilatacion
    kernel = numpy.ones((5, 5), numpy.uint8)
    img_erosion = cv2.erode(img_bin, kernel, iterations=1)
    img_dilation = cv2.dilate(img_erosion, kernel, iterations=1)

    idx = cv2.findNonZero(img_dilation) # indices de segmentacion

    mean_img = cv2.mean(idx) #Promedio de imagen

    xt,yt,zt=0,0,0
    counter=0
    # Promedio de posicion en 3D
    for [[c,r]] in idx:
        xt,yt,zt = xt+points[r,c][0], yt+points[r,c][1],zt+points[r,c][2]
        counter+=1
    xt,yt,zt = xt/counter,yt/counter,zt/counter

    print(mean_img)
    print([xt,yt,zt])

    return [int(mean_img[0]),int(mean_img[1]),xt,yt,zt]

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
