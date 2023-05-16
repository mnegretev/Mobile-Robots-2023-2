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

NAME = "Eduardo Argüelles Braulio"

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

    #lower = [25,50,50] if obj_name == "pringles" else [30,50,50]
    #upper = [35,255,255] if obj_name == "pringles" else [28,96,165]

    lower = []
    upper = []
    #Se cambia el espacio de color de RGB a HSV
    img_bgr = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    #Se asignan limites de colores para los dos objetos
    if obj_name == 'pringles':
     lower = [25,50,50]
     upper = [35,255,255]
    if obj_name == 'drink':
     lower = [10,200,50]
     upper = [20,255,255]

    lower = numpy.asarray(lower)
    upper = numpy.asarray(upper)
  
    img_bgr = cv2.inRange(img_bgr, lower, upper)
    nonZero_pixels = cv2.findNonZero(img_bgr)
    centroid_pixels = cv2.mean(nonZero_pixels)
    
    x,y,z = 0,0,0
    for p in nonZero_pixels:
        [[column,row]] = p
        if math.isnan(points[row,column][0]) or math.isnan(points[row,column][1]) or math.isnan(points[row,column][2]):
            pass
        else:
            x = x + points[row,column][0]
            y = y + points[row,column][1]
            z = z + points[row,column][2]

    x = x/len(nonZero_pixels)
    y = y/len(nonZero_pixels)
    z = z/len(nonZero_pixels)

    #Se devuelve el centroide en coordenadas de imagen y coordenadas cartesianas
    return [centroid_pixels[0],centroid_pixels[1],x,y,z]


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

