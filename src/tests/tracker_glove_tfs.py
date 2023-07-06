#!/usr/bin/env python

import math
import numpy as np
from std_msgs.msg import String
import tf 
import PyKDL 
import re
import rospy
from geometry_msgs.msg import Pose
import sys
from teaching_pkg.msg import *


rospy.init_node('tracker_tf_node', anonymous=True)
broad = tf.TransformBroadcaster()
hand_frame = PyKDL.Frame()
new_orientation = False
new_position = False
Rot_m = np.array([[1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0]])


def RtoQ(R):
    #x=y=z=w=0
    t = R[0][0] + R[1][1] + R[2][2]
    x_ok = 1+R[0][0]-R[1][1]-R[2][2]
    y_ok = 1-R[0][0]+R[1][1]-R[2][2]
    z_ok = 1-R[0][0]-R[1][1]+R[2][2]
    if t>0 and x_ok>0 and y_ok>0 and z_ok>0:
        r = math.sqrt(1+t)
        w = r/2
        x = math.copysign(math.sqrt(x_ok)/2, R[2][1]-R[1][2])
        y = math.copysign(math.sqrt(y_ok)/2, R[0][2]-R[2][0])
        z = math.copysign(math.sqrt(z_ok)/2, R[1][0]-R[0][1])
	print("pos")
    else:
        if R[0][0]>R[1][1] and R[0][0]>R[2][2]:
            r = math.sqrt(1+R[0][0]-R[1][1]-R[2][2])
            s = 1/(2*r)
            w = (R[2][1]-R[1][2])*s
            x = r/2
            y = (R[0][1]+R[1][0])*s
            z = (R[2][0]+R[0][2])*s
	    print("x")
        elif R[1][1]>R[0][0] and R[1][1]>R[2][2]:
            r = math.sqrt(1+R[1][1]-R[0][0]-R[2][2])
            print(r)
            s = 1/(2*r)
            print(s)
            w = (R[0][2]-R[2][0])*s
            y = r/2
            z = (R[1][2]+R[2][1])*s
            x = (R[0][1]+R[1][0])*s
	    print("y")
        else:
            r = math.sqrt(1+R[2][2]-R[0][0]-R[1][1])
            s = 1/(2*r)
            w = (R[1][0]-R[0][1])*s
            z = r/2
            x = (R[2][0]+R[0][2])*s
            y = (R[1][2]+R[2][1])*s
	    print("z")
    return [x,y,z,w]


def broadcastTransform(br, frame, frame_id, parent_frame, time):
	#Function for broadcasting transforms 
	time2 = rospy.get_rostime()
        br.sendTransform((frame.p.x(), frame.p.y(), frame.p.z()), 
            frame.M.GetQuaternion(), 
            time2, 
            frame_id, 
            parent_frame) 


def update_tfs():
    global hand_frame
    global new_position, new_orientation
    new_position = False 
    new_orientation = False
    tf_name = "right_hand_tracker_glove"
    parent_tf = "world"
    current_time = rospy.get_rostime()
    broadcastTransform(broad, hand_frame, tf_name, parent_tf, current_time)


####### Subscribers #########
def right_tracker_cb(info):
    global hand_frame
    global new_position, new_orientation
    x_tracker = info.x
    y_tracker = info.y
    z_tracker =  info.z
    hand_frame.p = PyKDL.Vector(x_tracker, y_tracker, z_tracker)
    new_position = True
    if new_orientation:
	update_tfs()

subsTrackerR = rospy.Subscriber('/Tracker/right_hand_movement_xyz', discrete_movement, right_tracker_cb)


def right_glove_cb(info):
    global hand_frame
    global new_position, new_orientation
    global Rot_m
    print(info)

    """
    Rot_m[0][0] = info.pitch.x
    Rot_m[1][0] = info.pitch.y
    Rot_m[2][0] = info.pitch.z
    Rot_m[0][1] = info.roll.x
    Rot_m[1][1] = info.roll.y
    Rot_m[2][1] = info.roll.z
    Rot_m[0][2] = info.yaw.x
    Rot_m[1][2] = info.yaw.y
    Rot_m[2][2] = info.yaw.z
    """
    Rot_m[0][0] = info.roll.x
    Rot_m[1][0] = info.roll.y
    Rot_m[2][0] = info.roll.z
    Rot_m[0][1] = info.pitch.x
    Rot_m[1][1] = info.pitch.y
    Rot_m[2][1] = info.pitch.z
    Rot_m[0][2] = info.yaw.x
    Rot_m[1][2] = info.yaw.y
    Rot_m[2][2] = info.yaw.z

    #print(Rot_m)
    quat = RtoQ(Rot_m)
    print(quat)
    hand_frame.M = PyKDL.Rotation.Quaternion(quat[0],quat[1],quat[2],quat[3])
    new_orientation = True
    if new_position:
	update_tfs()	

subsGloveR = rospy.Subscriber('/Glove/right_hand_orientation', rotation_matrix, right_glove_cb)


####### Quit #######
print('Tracker and glove tf node ready')
 # Keep this process running until Enter is pressed
print ("Press Enter to quit...")
try:
	sys.stdin.readline()
except KeyboardInterrupt:
	print("Tracker and glove tf node finalized")
	pass
