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

AUTOR = "Martinez Garcia Isaac"

def callback_recognized_speech(data):
    global recognized_speech, new_task, executing_task
    recognized_speech = data.hypothesis[0]
    print("New command received: " + recognized_speech)

def callback_goal_reached(data):
    global goal_reached
    goal_reached = data.data
    print("Received goal reached: " + str(goal_reached))

def parse_command(cmd):
    obj = "pringles" if "PRINGLES" in cmd else "drink"
    loc = [8.0,8.5] if "TABLE" in cmd else [3.22, 9.72]
    return obj, loc

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

def move_left_gripper(q):
    global pubLaGoalGrip
    pubLaGoalGrip.publish(q)
    time.sleep(1.0)

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

def move_right_gripper(q):
    global pubRaGoalGrip
    pubRaGoalGrip.publish(q)
    time.sleep(1.0)

def move_head(pan, tilt):
    global pubHdGoalPose
    msg = Float64MultiArray()
    msg.data.append(pan)
    msg.data.append(tilt)
    pubHdGoalPose.publish(msg)
    time.sleep(1.0)

def move_base(linear, angular, t):
    global pubCmdVel
    cmd = Twist()
    cmd.linear.x = linear
    cmd.angular.z = angular
    pubCmdVel.publish(cmd)
    time.sleep(t)
    pubCmdVel.publish(Twist())

def go_to_goal_pose(goal_x, goal_y):
    global pubGoalPose
    goal_pose = PoseStamped()
    goal_pose.pose.orientation.w = 1.0
    goal_pose.pose.position.x = goal_x
    goal_pose.pose.position.y = goal_y
    pubGoalPose.publish(goal_pose)

def say(text):
    global pubSay
    msg = SoundRequest()
    msg.sound   = -3
    msg.command = 1
    msg.volume  = 1.0
    msg.arg2    = "voice_kal_diphone"
    msg.arg = text
    pubSay.publish(msg)

def find_object(object_name):
    global srvFindObject
    try:
        resp = srvFindObject(object_name)
    except rospy.ServiceException as e:
        print("Service call failed: " + str(e))
        return
    return resp.x, resp.y, resp.z

def calculate_inverse_kinematics_left(x,y,z,roll, pitch, yaw):
    global srvIkLa
    try:
        resp = srvIkLa(x,y,z,roll, pitch, yaw)
    except rospy.ServiceException as e:
        print("Service call failed: " + str(e))
        return
    return resp.q1, resp.q2, resp.q3, resp.q4, resp.q5, resp.q6, resp.q7

def calculate_inverse_kinematics_right(x,y,z,roll, pitch, yaw):
    global srvIkRa
    try:
        resp = srvIkRa(x,y,z,roll, pitch, yaw)
    except rospy.ServiceException as e:
        print("Service call failed: " + str(e))
        return
    return resp.q1, resp.q2, resp.q3, resp.q4, resp.q5, resp.q6, resp.q7

def main():
    global srvIkLa, srvIkRa, srvFindObject, pubSay, pubGoalPose, pubLaGoalPose, pubLaGoalGrip, pubRaGoalPose, pubRaGoalGrip, pubHdGoalPose, pubCmdVel
    global recognized_speech, new_task, executing_task, goal_reached
    rospy.init_node("final_project", anonymous = True)
    rospy.Subscriber("/recognized_speech", RecognizedSpeech, callback_recognized_speech)
    rospy.Subscriber("/goal_reached", Bool, callback_goal_reached)
    srvIkLa = rospy.ServiceProxy("/ik_left_arm", CalculateIK)
    srvIkRa = rospy.ServiceProxy("/ik_right_arm", CalculateIK)
    srvFindObject = rospy.ServiceProxy("/find_object", FindObject)
    pubSay = rospy.Publisher("/sound_play", SoundRequest, queue_size = 10)
    pubGoalPose = rospy.Publisher("/goal_pose", PoseStamped, queue_size = 10)
    pubLaGoalPose = rospy.Publisher("/left_arm_goal_pose", Float64MultiArray, queue_size = 10)
    pubLaGoalGrip = rospy.Publisher("/left_arm_goal_grip", Float64, queue_size = 10)
    pubRaGoalPose = rospy.Publisher("/right_arm_goal_pose", Float64MultiArray, queue_size = 10)
    pubRaGoalGrip = rospy.Publisher("/right_arm_goal_grip", Float64, queue_size = 10)
    pubHdGoalPose = rospy.Publisher("/head_goal_pose", Float64MultiArray, queue_size = 10)
    pubCmdVel = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)
    print("FINAL PROJECT - " + AUTOR)
    recognized_speech = ""
    new_task = False
    executing_task = False
    goal_reached = False
    while not rospy.is_shutdown():
        if recognized_speech != "":
            if not executing_task:
                new_task = True
                executing_task = True
                print("New task: " + recognized_speech)
                object_name, location = parse_command(recognized_speech)
                recognized_speech = ""
                if object_name == "pringles":
                    say("I will get the pringles")
                else:
                    say("I will get the drink")
                # find object
                # calculate IK
                # grasp object
                # go to location
                # release object
                # say done
                new_task = False
                executing_task = False
                say("Task completed")
        rospy.spin()

if __name__ == "__main__":
    main()
