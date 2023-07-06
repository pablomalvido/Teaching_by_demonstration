#!/usr/bin/env python

#Import libraries
import rospy
import PyKDL 
import numpy as np
import math 
import tf 
import time
import os
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from std_msgs.msg import Float32
from teaching_pkg.msg import *
from os.path import abspath


####### Initialization #######
#ROS init
rospy.init_node('tracker_position_analysis')
tracker_position_xyz_pub = rospy.Publisher('/Tracker/right_hand_movement_xyz', discrete_movement, queue_size=1)
tracker_position_rpy_pub = rospy.Publisher('/Tracker/right_hand_movement_rpy', discrete_movement, queue_size=1)


#Global variables
deg_rot = 0
 
right_tracker_info = leap_hand() #not in use
initial_time = rospy.get_time() #not in use
period = 0.15 #not in use

status = {'x': '', 'y': '', 'z': '', 'pitch': '', 'roll': '', 'yaw': ''}

initial_count_xyz = 0; initial_count_rpy = 0

position_xyz = {'x': {'k-10': 0, 'k-20': 0, 'k-30': 0, 'k-2-1': 0, 'k-3-2': 0, 'status': '', 'stop': 0},
	'y': {'k-10': 0, 'k-20': 0, 'k-30': 0, 'k-2-1': 0, 'k-3-2': 0, 'status': '', 'stop': 0}, 
	'z': {'k-10': 0, 'k-20': 0, 'k-30': 0, 'k-2-1': 0, 'k-3-2': 0, 'status': '', 'stop': 0}}

position_rpy = {'pitch': {'k-10': 0, 'k-20': 0, 'k-30': 0, 'k-2-1': 0, 'k-3-2': 0, 'status': '', 'stop': 0},
	'roll': {'k-10': 0, 'k-20': 0, 'k-30': 0, 'k-2-1': 0, 'k-3-2': 0, 'status': '', 'stop': 0}, 
	'yaw': {'k-10': 0, 'k-20': 0, 'k-30': 0, 'k-2-1': 0, 'k-3-2': 0, 'status': '', 'stop': 0}}

glove_pose = {'pitch': {'x': 1, 'y': 0, 'z': 0},
	 'roll': {'x': 0, 'y': 1, 'z': 0}, 
	 'yaw': {'x': 0, 'y': 0, 'z': 1}}

x_tracker = 0; y_tracker = 0; z_tracker = 0
new_position = False; new_orientation = False

p_1 = np.array([0, 0, 0]); p_2 = np.array([0, 0, 0]); p_3 = np.array([0, 0, 0]); p_21 = np.array([0, 0, 0]); p_32 = np.array([0, 0, 0])
HTM_3F = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
HTM_2F = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]) 
HTM_1F = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
HTM_21 = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
HTM_32 = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])


####### Aiding functions #######
def transform_frame(R_original, R_transform_prev):
    R_result = np.matmul(R_transform_prev, R_original)
    return R_result


def inverse_HTM(M):
    px = -(M[0][0] * M[0][3] + M[1][0] * M[1][3] + M[2][0] * M[2][3])
    py = -(M[0][1] * M[0][3] + M[1][1] * M[1][3] + M[2][1] * M[2][3])
    pz = -(M[0][2] * M[0][3] + M[1][2] * M[1][3] + M[2][2] * M[2][3])
    HTM = np.array([[M[0][0], M[1][0], M[2][0], px],
                    [M[0][1], M[1][1], M[2][1], py],
                    [M[0][2], M[1][2], M[2][2], pz],
			        [0, 0, 0, 1]])
    return HTM


####### Data analysis functions #######
def analyze_movement(position_change):
    global status
    #20, 1.3, 1.5, 2.5
    eps1 = 0.04
    eps2 = eps1 * 1.2
    eps3 = eps1 * 1.4
    epsFast = eps1 * 2.5
    stop_detected = 3

    for axis in position_change:

            position_change[axis]['status'] = 'Stop'
            status_change = False
            
            if position_change[axis]['k-10'] >= 0:
                if position_change[axis]['k-10'] >= eps1:
                    if position_change[axis]['k-10'] >= epsFast:
                        position_change[axis]['status'] = 'Pos Fast'
                    else:
                        position_change[axis]['status'] = 'Pos'

                elif position_change[axis]['k-2-1'] >= 0:
                        if position_change[axis]['k-20'] >= eps2:
                            position_change[axis]['status'] = 'Pos'

                        elif position_change[axis]['k-3-2'] >= eps3:
                                if position_change[axis]['k-30']:
                                    position_change[axis]['status'] = 'Pos'

            else:
                if position_change[axis]['k-10'] <= -eps1:
                    if position_change[axis]['k-10'] <= -epsFast:
                        position_change[axis]['status'] = 'Neg Fast'
                    else:
                        position_change[axis]['status'] = 'Neg'

                elif position_change[axis]['k-2-1'] < 0:
                        if position_change[axis]['k-20'] <= -eps2:
                            position_change[axis]['status'] = 'Neg'

                        elif position_change[axis]['k-3-2'] < 0:
                                if position_change[axis]['k-30'] <= -eps3:
                                    position_change[axis]['status'] = 'Neg'

            if position_change[axis]['status'] == 'Stop':
                position_change[axis]['stop'] += 1
            
            else:
                position_change[axis]['stop'] = 0
                if status[axis] != position_change[axis]['status']:
                    status[axis] = position_change[axis]['status']
                    status_change = True

            if position_change[axis]['stop'] == stop_detected:
                status[axis] = "Stop"
                status_change = True

	    if axis == 'roll' or axis == 'pitch' or axis == 'yaw':
		#print("" + axis + ": " + str(position_change[axis]['k-10']))
            	if status_change:
                #if calibrated:
                    print("" + axis + ": " + status[axis])
		    print("" + axis + ": " + str(position_change[axis]['k-10']))
		    #pass

		


def analyze_movement_xyz():
    global x_tracker, y_tracker, z_tracker
    global position_xyz
    global p_1, p_2, p_3, p_21, p_32
    global initial_count_xyz
    #Adquire new value
    p_0 = np.array([x_tracker, y_tracker, z_tracker])
    #Calculate new info of this call
    p_10 = p_0 - p_1
    p_20 = p_0 - p_2
    p_30 = p_0 - p_3
    #Prepare the info needed to analyze
    for axis in position_xyz:
	if axis == 'x':
		row = 0
	elif axis == 'y':
		row = 1
	else:
		row = 2
	position_xyz[axis]['k-10'] = p_10[row]
	position_xyz[axis]['k-2-1'] = p_21[row]
	position_xyz[axis]['k-20'] = p_20[row]
	position_xyz[axis]['k-3-2'] = p_32[row]
	position_xyz[axis]['k-30'] = p_30[row]
    #Call the function that analyzes the movements
    if initial_count_xyz < 3:
        initial_count_xyz += 1
    else:
        analyze_movement(position_xyz)
    #Publish info
    tracker_msg = discrete_movement()
    tracker_msg.x_state = position_xyz['x']['status']
    tracker_msg.y_state = position_xyz['y']['status']
    tracker_msg.z_state = position_xyz['z']['status']
    tracker_msg.x = p_0[0]
    tracker_msg.y = p_0[1]
    tracker_msg.z = p_0[2]
    tracker_position_xyz_pub.publish(tracker_msg)
    #Update info for the next
    p_3 = p_2
    p_2 = p_1
    p_1 = p_0
    p_32 = p_21
    p_21 = p_10


def analyze_movement_rpy():
    global glove_pose
    global x_tracker, y_tracker, z_tracker
    global new_position, new_orientation
    global position_rpy
    global HTM_3F, HTM_2F, HTM_1F, HTM_21, HTM_32
    global initial_count_rpy
    new_position = False
    new_orientation = False
    #Adquire new value
    HTM_F0 = np.array([[glove_pose['pitch']['x'], glove_pose['roll']['x'], glove_pose['yaw']['x'], x_tracker],
                            [glove_pose['pitch']['y'], glove_pose['roll']['y'], glove_pose['yaw']['y'], y_tracker],
                            [glove_pose['pitch']['z'], glove_pose['roll']['z'], glove_pose['yaw']['z'], z_tracker],
			    [0, 0, 0, 1]])
    #print(str(HTM_F0))
    #Calculate new info of this call
    HTM_0F = inverse_HTM(HTM_F0)
    HTM_10 = transform_frame(R_transform_prev = HTM_1F, R_original = HTM_F0)
    HTM_20 = transform_frame(R_transform_prev = HTM_2F, R_original = HTM_F0)
    HTM_30 = transform_frame(R_transform_prev = HTM_3F, R_original = HTM_F0)
    #print(str(HTM_10))
    #Prepare the info needed to analyze
    for axis in position_rpy:
	if axis == 'pitch':
		row = 0
	elif axis == 'roll':
		row = 1
	else:
		row = 2
	position_rpy[axis]['k-10'] = HTM_10[row][3]
	position_rpy[axis]['k-2-1'] = HTM_21[row][3]
	position_rpy[axis]['k-20'] = HTM_20[row][3]
	position_rpy[axis]['k-3-2'] = HTM_32[row][3]
	position_rpy[axis]['k-30'] = HTM_30[row][3]
    #Call the function that analyzes the movements
    if initial_count_rpy < 3:
        initial_count_rpy += 1
    else:
        analyze_movement(position_rpy)
    #Publish info
    tracker_msg = discrete_movement()
    tracker_msg.x_state = position_rpy['pitch']['status']
    tracker_msg.y_state = position_rpy['roll']['status']
    tracker_msg.z_state = position_rpy['yaw']['status']
    tracker_msg.x = 0
    tracker_msg.y = 0
    tracker_msg.z = 0
    tracker_position_rpy_pub.publish(tracker_msg)
    #Update info for the next call
    HTM_3F = HTM_2F
    HTM_2F = HTM_1F
    HTM_1F = HTM_0F
    HTM_32 = HTM_21
    HTM_21 = HTM_10


####### Subscriptors #########
def right_tracker_cb(info):
    #global period
    #global initial_time
    #global global_recording
    global deg_rot
    global x_tracker, y_tracker, z_tracker
    global new_position, new_orientation
    position_VIVE = info.data
    x_tracker = float((position_VIVE.split(';'))[2]) #Horiz1 axis
    y_tracker = float((position_VIVE.split(';'))[0]) #Horiz2 axis
    z_tracker =  float((position_VIVE.split(';'))[1]) #Vert axis
    #print(str(z_tracker))
    if not deg_rot==0:
	x_tracker = x_tracker * math.cos(deg_rot) + y_tracker * math.sin(deg_rot)
	y_tracker = -x_tracker * math.sin(deg_rot) + y_tracker * math.cos(deg_rot)
    analyze_movement_xyz()
    new_position = True
    if new_orientation:
	analyze_movement_rpy()

subsTrackerR = rospy.Subscriber('Tracker/pose_string/right', String, right_tracker_cb)


def calibration_degree_cb(info):
    global deg_rot
    global initial_count_xyz, initial_count_rpy
    global status
    global position_xyz, position_rpy
    initial_count_xyz = 0
    initial_count_rpy = 0
    for axis in position_xyz:
        position_xyz[axis]['stop'] = 0
        status[axis] = ''
        position_xyz[axis]['status'] = ''
    for axis in position_rpy:
        position_rpy[axis]['stop'] = 0
        status[axis] = ''
        position_rpy[axis]['status'] = ''
    deg_rot = info.data

subsCalibrationDegree = rospy.Subscriber('/Tracker/right_hand_calibration_degree', Float32, calibration_degree_cb)


def right_glove_cb(info):
    global glove_pose
    global new_position, new_orientation
    glove_pose['pitch']['x'] = info.pitch.x
    glove_pose['pitch']['y'] = info.pitch.y
    glove_pose['pitch']['z'] = info.pitch.z
    glove_pose['roll']['x'] = info.roll.x
    glove_pose['roll']['y'] = info.roll.y
    glove_pose['roll']['z'] = info.roll.z
    glove_pose['yaw']['x'] = info.yaw.x
    glove_pose['yaw']['y'] = info.yaw.y
    glove_pose['yaw']['z'] = info.yaw.z
    new_orientation = True
    if new_position:
	analyze_movement_rpy()	

subsGloveR = rospy.Subscriber('/Glove/right_hand_orientation', rotation_matrix, right_glove_cb)


####### Quit #######
print('Tracker node ready')
 # Keep this process running until Enter is pressed
print ("Press Enter to quit...")
try:
	sys.stdin.readline()
except KeyboardInterrupt:
	print("Tracker node finalized")
	pass
