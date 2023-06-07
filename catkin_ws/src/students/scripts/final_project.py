#!/usr/bin/env python3
#
# MOBILE ROBOTS - FI-UNAM, 2023-2
# FINAL PROJECT - SIMPLE SERVICE ROBOT
# 
# Instructions:
# Write the code necessary to make the robot to perform the following possible commands:
# * Robot take the <pringles|drink> to the <table|kitchen>
# You can choose where the table and kitchen are located within the map.
# Pringles and drink are the two objects on the table used in practice 07.
# The Robot must recognize the orders using speech recognition.
# Entering the command by text or similar way is not allowed.
# The Robot must announce the progress of the action using speech synthesis,
# for example: I'm going to grab..., I'm going to navigate to ..., I arrived to..., etc.
# Publishers and suscribers to interact with the subsystems (navigation,
# vision, manipulation, speech synthesis and recognition) are already declared. 
#

import rospy
import tf
import math
import time
from std_msgs.msg import String, Float64MultiArray, Float64, Bool

from nav_msgs.msg import Path
from nav_msgs.srv import GetPlan, GetPlanRequest
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist, PoseStamped, Pose, Point, PointStamped
from sound_play.msg import SoundRequest
from custom_msgs.srv import *
from custom_msgs.msg import *

NAME = "VALDERRABANO VEGA ABRAHAM"

#
# Global variable 'speech_recognized' contains the last recognized sentence
#
def callback_recognized_speech(msg):
    global recognized_speech, new_task, executing_task
    recognized_speech = msg.hypothesis[0]
    print("New command received: " + recognized_speech)
    new_task=True

#
# Global variable 'goal_reached' is set True when the last sent navigation goal is reached
#
def callback_goal_reached(msg):
    global goal_reached
    goal_reached = msg.data
    print("Received goal reached: " + str(goal_reached))

def parse_command(cmd):
    obj = "pringles" if "PRINGLES" in cmd else "drink"
    loc = [8.5,8.5] if "TABLE" in cmd else [3.5, 7.4]
    return obj, loc

#
# This function sends the goal articular position to the left arm and sleeps 2 seconds
# to allow the arm to reach the goal position. 
#
def move_left_arm(q1,q2,q3,q4,q5,q6,q7):
    global pubLaGoalPose
    msg = Float64MultiArray()
    msg.data.append(q1)
    msg.data.append(q2)
    msg.data.append(q3)
    msg.data.append(q4)
    msg.data.append(q5)
    msg.data.append(q6)
    msg.data.append(q7)
    pubLaGoalPose.publish(msg)
    time.sleep(2.0)

#
# This function sends the goal angular position to the left gripper and sleeps 1 second
# to allow the gripper to reach the goal angle. 
#
def move_left_gripper(q):
    global pubLaGoalGrip
    msg = Float64MultiArray()
    msg.data.append(q)
    pubLaGoalGrip.publish(q)
    time.sleep(1.0)

#
# This function sends the goal articular position to the right arm and sleeps 2 seconds
# to allow the arm to reach the goal position. 
#
def move_right_arm(q1,q2,q3,q4,q5,q6,q7):
    global pubRaGoalPose
    msg = Float64MultiArray()
    msg.data.append(q1)
    msg.data.append(q2)
    msg.data.append(q3)
    msg.data.append(q4)
    msg.data.append(q5)
    msg.data.append(q6)
    msg.data.append(q7)
    pubRaGoalPose.publish(msg)
    time.sleep(2.0)

#
# This function sends the goal angular position to the right gripper and sleeps 1 second
# to allow the gripper to reach the goal angle. 
#
def move_right_gripper(q):
    global pubRaGoalGrip
    pubRaGoalGrip.publish(q)
    time.sleep(1.0)

#
# This function sends the goal pan-tilt angles to the head and sleeps 1 second
# to allow the head to reach the goal position. 
#
def move_head(pan, tilt):
    global pubHdGoalPose
    msg = Float64MultiArray()
    msg.data.append(pan)
    msg.data.append(tilt)
    pubHdGoalPose.publish(msg)
    time.sleep(1.0)

#
# This function sends a linear and angular speed to the mobile base to perform
# low-level movements. The mobile base will move at the given linear-angular speeds
# during a time given by 't'
#
def move_base(linear, angular, t):
    global pubCmdVel
    cmd = Twist()
    cmd.linear.x = linear
    cmd.angular.z = angular
    pubCmdVel.publish(cmd)
    time.sleep(t)
    pubCmdVel.publish(Twist())
    time.sleep(1.0)

#
# This function publishes a global goal position. This topic is subscribed by
# pratice04 and performs path planning and tracking.
#
def go_to_goal_pose(goal_x, goal_y):
    global pubGoalPose
    goal_pose = PoseStamped()
    goal_pose.pose.orientation.w = 1.0
    goal_pose.pose.position.x = goal_x
    goal_pose.pose.position.y = goal_y
    pubGoalPose.publish(goal_pose)

#
# This function sends a text to be synthetized.
#
def say(text):
    global pubSay
    msg = SoundRequest()
    msg.sound   = -3
    msg.command = 1
    msg.volume  = 3.0
    msg.arg2    = "voice_kal_diphone"
    msg.arg = text
    pubSay.publish(msg)
    time.sleep(2.0)

#
# This function calls the service for calculating inverse kinematics for left arm (practice 08)
# and returns the calculated articular position.
#
def calculate_inverse_kinematics_left(x,y,z,roll, pitch, yaw):
    req_ik=InverseKinematicsRequest()
    req_ik.x = x
    req_ik.y = y
    req_ik.z = z
    req_ik.roll  = roll
    req_ik.pitch = pitch
    req_ik.yaw   = yaw
    clt = rospy.ServiceProxy("/manipulation/la_inverse_kinematics", InverseKinematics)
    resp = clt(req_ik)
    return [resp.q1, resp.q2, resp.q3, resp.q4, resp.q5, resp.q6, resp.q7]

#
# This function calls the service for calculating inverse kinematics for right arm (practice 08)
# and returns the calculated articular position.
#
def calculate_inverse_kinematics_right(x,y,z,roll, pitch, yaw):
    req_ik = InverseKinematicsRequest()
    req_ik.x = x
    req_ik.y = y
    req_ik.z = z
    req_ik.roll  = roll
    req_ik.pitch = pitch
    req_ik.yaw   = yaw
    clt = rospy.ServiceProxy("/manipulation/ra_inverse_kinematics", InverseKinematics)
    resp = clt(req_ik)
    return [resp.q1, resp.q2, resp.q3, resp.q4, resp.q5, resp.q6, resp.q7]

#
# Calls the service for finding object (practice 08) and returns
# the xyz coordinates of the requested object w.r.t. "realsense_link"
#
def find_object(object_name):
    clt_find_object = rospy.ServiceProxy("/vision/find_object", FindObject)
    req_find_object = FindObjectRequest()
    req_find_object.cloud = rospy.wait_for_message("/hardware/realsense/points", PointCloud2)
    req_find_object.name  = object_name
    resp = clt_find_object(req_find_object)
    return [resp.x, resp.y, resp.z]

#
# Transforms a point xyz expressed w.r.t. source frame to the target frame
#
def transform_point(x,y,z, source_frame, target_frame):
    listener = tf.TransformListener()
    listener.waitForTransform(target_frame, source_frame, rospy.Time(), rospy.Duration(4.0))
    obj_p = PointStamped()
    obj_p.header.frame_id = source_frame
    obj_p.header.stamp = rospy.Time(0)
    obj_p.point.x, obj_p.point.y, obj_p.point.z = x,y,z
    obj_p = listener.transformPoint(target_frame, obj_p)
    return [obj_p.point.x, obj_p.point.y, obj_p.point.z]

def main():
    global new_task, recognized_speech, executing_task, goal_reached
    global pubLaGoalPose, pubRaGoalPose, pubHdGoalPose, pubLaGoalGrip, pubRaGoalGrip
    global pubGoalPose, pubCmdVel, pubSay
    print("FINAL PROJECT - " + NAME)
    rospy.init_node("final_project")
    rospy.Subscriber('/hri/sp_rec/recognized', RecognizedSpeech, callback_recognized_speech)
    rospy.Subscriber('/navigation/goal_reached', Bool, callback_goal_reached)
    pubGoalPose = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    pubCmdVel   = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    pubSay      = rospy.Publisher('/robotsound', SoundRequest, queue_size=10)
    pubLaGoalPose = rospy.Publisher("/hardware/left_arm/goal_pose" , Float64MultiArray, queue_size=10);
    pubRaGoalPose = rospy.Publisher("/hardware/right_arm/goal_pose", Float64MultiArray, queue_size=10);
    pubHdGoalPose = rospy.Publisher("/hardware/head/goal_pose"     , Float64MultiArray, queue_size=10);
    pubLaGoalGrip = rospy.Publisher("/hardware/left_arm/goal_gripper" , Float64, queue_size=10);
    pubRaGoalGrip = rospy.Publisher("/hardware/right_arm/goal_gripper", Float64, queue_size=10);
    listener = tf.TransformListener()
    loop = rospy.Rate(10)
    print("Waiting for services...")
    rospy.wait_for_service('/manipulation/la_inverse_kinematics')
    rospy.wait_for_service('/manipulation/ra_inverse_kinematics')
    rospy.wait_for_service('/vision/find_object')
    print("Services are now available:")
    say("Starting nodes .Your wish is my command ")

    #
    # FINAL PROJECT 
    #
    new_task=False
    goal_reached=False
    executing_task=False
    state= "SM_INIT"

    while not rospy.is_shutdown():
        if state=="SM_INIT":
            print("Starting final project. Waiting for new task")
           
            goal_reached=False
            executing_task=False
            new_task=False
            state="SM_WAITING_FOR_NEW_TASK"
    	
        elif state=="SM_WAITING_FOR_NEW_TASK":
            if(new_task):
                obj,loc=parse_command(recognized_speech)
                print("New task received. Requested object: "+obj+" Requested location: "+str(loc))
                say("I've heard you")
                state="SM_MOVE_HEAD"
    		
        elif state=="SM_MOVE_HEAD":
            print("Moving head")
            move_head(0,-1.0)
            state="SM_RECOGNIZE_OBJECT"
    		
        elif state=="SM_RECOGNIZE_OBJECT":
            print("Trying to find "+obj)
            say("I'm looking for " +obj)
            x,y,z=find_object(obj)
            print("Found object at : "+ str([x,y,z]))
            say("I've found the object" + obj)
            target_frame="shoulders_left_link" if obj=="pringles" else "shoulders_right_link"
            x,y,z = transform_point(x,y,z,"realsense_link", target_frame)
            print("Coords wrt arm: "+str([x,y,z]))
            time.sleep(1.0)
            state="SM_MOVE_ARM"
    		
    		
        elif state=="SM_MOVE_ARM":
            #
            move_base(-3,0,1)
            #move_base(-2,0,0,1)
            if obj=="pringles":
                #move_left_arm(-0.3,0.2,-0.1,2.1,0.0,0.2,0.0)
    			#move_left_arm(-0.7,0,0,1.9,0.1,0.5,0)
                move_left_arm(-0.3,0.193,-0.1100,2.1460,0.001,0.1400,0)
                move_left_gripper(0.4)
            else:
                #move_right_arm(-0.3,-0.2,0.1,3.0,0.5,0.0,0.0)
    			#move_right_arm(-0.7,0.3,0.1,1.7,0.6,0.1,0.4)
                move_right_arm(-0.3,-0.2,-0.03,3.0,0.5,0.0,0.0)
                move_right_gripper(0.4)  
    			
            time.sleep(1.0)  
            move_base(3,0,1)
            state="SM_TAKE_OBJECT"
    		
        elif state=="SM_TAKE_OBJECT":
            print("Taking "+obj)
            if obj=="pringles":
                q=calculate_inverse_kinematics_left(x+0.1,y,z,0.5,-1.44,-0.67)
                move_left_arm(q[0], q[1], q[2], q[3], q[4], q[5], q[6])
                move_left_gripper(-0.4)
                move_left_arm(q[0], q[1], q[2], q[3]+0.3, q[4], q[5], q[6])#prepare for moving
            else:
                q=calculate_inverse_kinematics_right(x+0.1,y,z+0.1,-0.032,-1.525,0.2)
                move_right_arm(q[0], q[1], q[2], q[3], q[4], q[5], q[6])
                move_right_gripper(-0.4)
                move_right_arm(-0.4,0,0,3,1,0,0)#prepare
    			
            say("I have taken the "+obj)
            print("OK: "+obj+" taken")
            move_base(-3,0,1)
            time.sleep(2)
            state="SM_MOVE"
    	
        elif state=="SM_MOVE":
            if not goal_reached and not executing_task:
                print("delivering...")
                say("delivering")
                go_to_goal_pose(loc[0],loc[1])
                executing_task= True
            elif goal_reached:
                executing_task=False
    			
                state="SM_DELIVER"
    	
        elif state=="SM_DELIVER":
            if obj=="pringles":
                move_left_gripper(0.5)
                pringles_pos=loc
            else:
                move_right_gripper(0.5)
                drink_pos=loc
            state="SM_INIT"
    			
        else:
            print("ya termine")
            break
            loop.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    