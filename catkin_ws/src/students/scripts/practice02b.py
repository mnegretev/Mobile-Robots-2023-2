#!/usr/bin/env python3
#
# MOBILE ROBOTS - FI-UNAM, 2023-2
# PRACTICE 3b - PATH PLANNING BY A-STAR
#
# Instructions:
# Write the code necessary to plan a path using an
# occupancy grid and the A* algorithm
# MODIFY ONLY THE SECTIONS MARKED WITH THE 'TODO' COMMENT
#

import numpy
import heapq
import rospy
import math
from geometry_msgs.msg import PoseStamped, Pose, Point
from nav_msgs.msg import Path
from nav_msgs.srv import *
from collections import deque

NAME = "LÓPEZ HERNÁNDEZ EMANUEL"

msg_path = Path()

def a_star(start_r, start_c, goal_r, goal_c, grid_map, cost_map):
    #
    # TODO:
    # Review the A* algorithm to find a path in an occupancy grid map given the start cell
    # [start_r, start_c], the goal cell [goal_r, goal_c] and the map 'grid_map'.
    # The function returns a set of points of the form [[start_r, start_c], [r1,c1], [r2,c2], ..., [goal_r, goal_c]]
    # indicating the indices (cell coordinates) of the path cells.
    # If path cannot be found, function returns an empty tuple []
    #
    in_open_list   = numpy.full(grid_map.shape, False)
    in_closed_list = numpy.full(grid_map.shape, False)
    g_values       = numpy.full(grid_map.shape, float("inf"))
    f_values       = numpy.full(grid_map.shape, float("inf"))
    parent_nodes   = numpy.full((grid_map.shape[0],grid_map.shape[1],2),-1)
    #
    # TODO:
    # Modify the list of adjacent node-offsets to use 8-connectiviy instead of 4-connectiviy
    #
    #adjacent_idx   = [[1,0],[0,1],[-1,0],[0,-1]]
    adjacent_idx      = [[1,0],[0,1],[-1,0],[0,-1], [1,1], [-1,1], [-1,-1],[1,-1]]
    #

    open_list = [] 
    heapq.heappush(open_list, (0, [start_r, start_c]))
    in_open_list [start_r, start_c] = True
    g_values     [start_r, start_c] = 0
    [row, col] = [start_r, start_c]
    iterations = 0
    
    while len(open_list) > 0 and [row, col] != [goal_r, goal_c]:
        [row, col] = heapq.heappop(open_list)[1]         
        in_closed_list[row,col] = True                  
        adjacent_nodes = [[row+i, col+j] for [i,j] in adjacent_idx]
        for [r,c] in adjacent_nodes:
            if grid_map[r,c] > 40 or grid_map[r,c] < 0 or in_closed_list[r,c]: 
                continue
            #
            # TODO:
            # Modify calculations of 'g' and 'h' to use euclidean distance
            # instead of Manhattan distance
            #
            #g = g_values[row, col] + abs(row-r) + abs(col-c) + cost_map[r][c]
            #h = abs(goal_r - r) + abs(goal_c - c)
            g = g_values[row, col] + math.sqrt((row-r)**2 + (col - c)**2) + cost_map[r][c]
            #h = math.sqrt((goal_r-r)**2 + (goal_c - c)**2)
            h=0
            
            f = g + h                         
            if g < g_values[r,c]:           
                g_values[r,c]     = g           
                f_values[r,c]     = f           
                parent_nodes[r,c] = [row, col]   
            if not in_open_list[r,c]:
                in_open_list[r,c] = True              
                heapq.heappush(open_list, (f, [r,c]))            
            iterations += 1
            
    if [row, col] != [goal_r, goal_c]:
        print("Cannot calculate path by A* :'(")
        return []
    print("Path calculated after " + str(iterations) + " iterations.")
    path = []
    while parent_nodes[row, col][0] != -1:
        path.insert(0, [row, col])
        [row, col] = parent_nodes[row, col]
    return path

def get_maps():
    print("Getting inflated and cost maps...")
    clt_static_map = rospy.ServiceProxy("/static_map"  , GetMap)
    clt_cost_map   = rospy.ServiceProxy("/cost_map"    , GetMap)
    clt_inflated   = rospy.ServiceProxy("/inflated_map", GetMap)
    try:
        static_map = clt_static_map().map
    except:
        print("Cannot get static map. Terminating program. ")
        exit()
    try:
        inflated_map = clt_inflated().map
        cost_map     = clt_cost_map().map
        print("Using inflated map with " +str(len(inflated_map.data)) + " cells.")
        print("Using cost map with "     +str(len(cost_map.data))     + " cells.")
    except:
        inflated_map = static_map
        cost_map     = static_map
        print("Cannot get augmented maps. Using static map instead.")
    inflated_map = numpy.reshape(numpy.asarray(inflated_map.data), (static_map.info.height, static_map.info.width))
    cost_map     = numpy.reshape(numpy.asarray(cost_map.data)    , (static_map.info.height, static_map.info.width))
    return [static_map, inflated_map, cost_map]

def callback_a_star(req):
    [s_map, inflated_map, cost_map] = get_maps()
    res = s_map.info.resolution
    [sx, sy] = [req.start.pose.position.x, req.start.pose.position.y]
    [gx, gy] = [req.goal .pose.position.x, req.goal .pose.position.y]
    [zx, zy] = [s_map.info.origin.position.x, s_map.info.origin.position.y]
    print("Calculating path by A* from " + str([sx, sy])+" to "+str([gx, gy]))
    path = a_star(int((sy-zy)/res), int((sx-zx)/res), int((gy-zy)/res), int((gx-zx)/res), inflated_map, cost_map)
    msg_path.poses = []
    for [r,c] in path:
        msg_path.poses.append(PoseStamped(pose=Pose(position=Point(x=(c*res + zx), y=(r*res + zy)))))
    return GetPlanResponse(msg_path)

def main():
    print("PRACTICE 03b - " + NAME)
    rospy.init_node("practice03b")
    rospy.wait_for_service('/static_map')
    rospy.Service('/path_planning/a_star_search'  , GetPlan, callback_a_star)
    pub_path = rospy.Publisher('/path_planning/a_star_path', Path, queue_size=10)
    loop = rospy.Rate(2)
    msg_path.header.frame_id = "map"
    while not rospy.is_shutdown():
        pub_path.publish(msg_path)
        loop.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    
