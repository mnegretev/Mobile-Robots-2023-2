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

NAME = "BRITO SERRANO MIGUEL ANGEL"

#
# Global variable 'speech_recognized' contains the last recognized sentence
#
def callback_recognized_speech(msg):
    global recognized_speech, new_task, executing_task
    if executing_task:
        return
    new_task = True
    recognized_speech = msg.hypothesis[0]
    print("New command received: " + recognized_speech)

#
# Global variable 'goal_reached' is set True when the last sent navigation goal is reached
#
def callback_goal_reached(msg):
    global goal_reached
    goal_reached = msg.data
    print("Received goal reached: " + str(goal_reached))

def parse_command(cmd):
    global table
    obj = "pringles" if "PRINGLES" in cmd else "drink"
    loc = [8.0,8.5] if "TABLE" in cmd else [3.22, 9.72]
    table = True  if "TABLE" in cmd else False
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
    msg.volume  = 1.0
    msg.arg2    = "voice_kal_diphone"
    msg.arg = text
    pubSay.publish(msg)

#
# This function calls the service for calculating inverse kinematics for left arm (practice 08)
# and returns the calculated articular position.
#
def calculate_inverse_kinematics_left(x,y,z,roll, pitch, yaw):
    req_ik = InverseKinematicsRequest()
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
    rospy.init_node("final_exercise")
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
    print("Services are now available.")

    #
    # FINAL PROJECT 
    #
    new_task = False
    recognized_speech = ""
    executing_task = False
    state = "SM_INIT"
    while not rospy.is_shutdown():
        if state == "SM_INIT":
            print("Initializing final project...")
            print("Waiting for spoken command...")
            state = "SM_WAIT_FOR_COMMAND"
        elif state == "SM_WAIT_FOR_COMMAND":
            if new_task:
                new_task = False
                executing_task = True
                state = "SM_PARSING"
        elif state == "SM_PARSING":
            obj, loc = parse_command(recognized_speech)
            print("Requested object: " + obj)
            print("Requested location: " + str(loc))
            state = "SM_MOVE_HEAD"
        elif state == "SM_MOVE_HEAD":
            move_head(0,-1)
            if obj == "pringles":
                #state = "SM_TURN"
                state = "SM_MOVE_LEFT_ARM"
            else:
                state = "SM_MOVE_RIGHT_ARM"
        elif state == "SM_MOVE_LEFT_ARM":
            #move_left_arm(-1, 0,0,1.5, 0, 0.8, 0)
            move_left_arm(-1.202, 0.193,0,1.644, 0, 1.65, 0)
            x,y,z = find_object(obj)
            x,y,z = transform_point(x,y,z,"realsense_link", "shoulders_left_link")
            a1,a2,a3,a4,a5,a6,a7 = calculate_inverse_kinematics_left(x.item(),y.item(),z.item(),0,-1.50,0) 
            move_left_gripper(0.5)
            move_left_arm(a1,a2,a3,a4,a5,a6,a7)
            say("I am going to grab the pringles")   
            move_base(0.5, 0, 1.5)
            move_left_gripper(-0.5)
            state = "SM_MOVE_BACK"
        elif state == "SM_MOVE_RIGHT_ARM":
            x,y,z = find_object(obj)
            x,y,z = transform_point(x,y,z,"realsense_link", "shoulders_right_link")
            a1,a2,a3,a4,a5,a6,a7 = calculate_inverse_kinematics_right(x.item(),y.item(),z.item(),0,-1.5,0)
            move_right_arm(-0.931, -0.192, 0, 1.346, 0.82, 0.03,0)
            move_right_arm(-0.287, -0.192, 0, 1.346, 0.95, 0.22,0)
            move_right_gripper(0.5)
            move_right_arm(a1+0.5,a2,a3,a4,a5,a6,a7)
            say("I am going to grab the drink")
            move_base(0.5, 0, 0.5)
            move_right_gripper(-0.5)
            state = "SM_MOVE_BACK"
        elif state == "SM_MOVE_BACK":
            move_base(-0.5, 0, 5)
            goal_reached = False 
            state = "SM_MOVE_GOAL"
            if table:
               
            else:
                
        elif state == "SM_MOVE_GOAL":    
            go_to_goal_pose(loc[0],loc[1])
            state="SM_WAIT"
        elif state=="SM_WAIT":
            if goal_reached:
                goal_reached = False   
                state = "SM_TURN"
                #state = "End"
        elif state =="SM_TURN":
            if table :
                move_base(0, -1, 4)
            else:
                move_base(0, 1, 15)            
            if obj == "pringles":
                move_left_arm(0.990, 0.18,0,1.532, 0, -0.45, 0)
            else:
            	move_right_arm(0.990, 0.18,0,1.532, 0, -0.45, 0)    
            state ="SM_FORWARD" 
        elif state =="SM_FORWARD":
            if table :
           
                move_base(0.5, 0, 5)
            else:
           
                move_base(0.5, 0, 8)
            state="SM_TO_DROP_LEFT"
        elif state =="SM_TO_DROP_LEFT":
            if obj == "pringles":
                move_left_gripper(0.5)
            else:
            	move_right_gripper(0.5)
            print("Se realizo la entrega del objeto")	
            state="End"
        elif state =="End":
            None
        else:
            print("ERROR")
        loop.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    
