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


#ROS init
rospy.init_node('calibration')
R_glove_tracker_pub = rospy.Publisher('/Glove/R_glove_tracker', rotation_matrix, queue_size=1)
calibration_degree_pub = rospy.Publisher('/Tracker/right_hand_calibration_degree', Float32, queue_size=1)


#Global variables
deg_rot = 0 
right_tracker_info = leap_hand() #change type
initial_time = rospy.get_time()
period = 0.15 #Not in use
path = os.path.join(os.path.dirname(__file__), '../files/calibration.txt')
tracker_mov = discrete_movement()
glove_pose = {'pitch': {'x': 1, 'y': 0, 'z': 0}, 'roll': {'x': 0, 'y': 1, 'z': 0}, 'yaw': {'x': 0, 'y': 0, 'z': 1}}

#Last: load the previous axis from a txt
def load_calibration():
    global path
    deg_rot = 0
    R_message = rotation_matrix()
    try:
	f = open(path, "r")
	for line in f:
		line_id, line_data = line.split(':')
		if line_id == 'xy degree':
			deg_rot = float(line_data)
		if line_id == 'R glove_tracker pitch':
			R_message.pitch.x = float(line_data.split(',')[0])
			R_message.pitch.y = float(line_data.split(',')[1])
	       		R_message.pitch.z = float(line_data.split(',')[2])
		if line_id == 'R glove_tracker roll':
			R_message.roll.x = float(line_data.split(',')[0])
			R_message.roll.y = float(line_data.split(',')[1])
	       		R_message.roll.z = float(line_data.split(',')[2])
		if line_id == 'R glove_tracker yaw':
			R_message.yaw.x = float(line_data.split(',')[0])
			R_message.yaw.y = float(line_data.split(',')[1])
	       		R_message.yaw.z = float(line_data.split(',')[2])
    except:
    	print("Error loading the calibration file")
        return False, 0, None
    finally:
	f.close()
    print("Calibration loaded succesfully")
    print('- Tracker rotation degree: ' + str(deg_rot) + '\n- Rotation matrix glove: \n' + str(R_message))
    return True, deg_rot, R_message


def calibrate_axis(axis, finger, hand_axis):
    global tracker_mov
    global glove_pose
    hand_axis_calibration = {'x': 0, 'y': 0, 'z': 0}
    samples = 0

    print("Move your right hand up and down to start calibrating " + axis + " axis")
    while not ((tracker_mov.z_state == 'Pos') or (tracker_mov.z_state == 'Pos Fast')):
        time.sleep(0.1)
    while not ((tracker_mov.z_state == 'Neg') or (tracker_mov.z_state == 'Neg Fast')):
        time.sleep(0.1)

    print("Align your right hand " + finger + " with the + " + axis + " axis and move your hand in that direction")
    while ((tracker_mov.x_state == 'Stop') or (tracker_mov.y_state == 'Stop')):
        time.sleep(0.1)
    x_init = tracker_mov.x
    y_init = tracker_mov.y
    #hand_axis_calibration['x'] = glove_pose[hand_axis]['x']
    #hand_axis_calibration['y'] = glove_pose[hand_axis]['y']
    #hand_axis_calibration['z'] = glove_pose[hand_axis]['z']
    samples += 1
    while not ((tracker_mov.x_state == 'Stop') or (tracker_mov.y_state == 'Stop')):
        hand_axis_calibration['x'] += glove_pose[hand_axis]['x']
        hand_axis_calibration['y'] += glove_pose[hand_axis]['y']
        hand_axis_calibration['z'] += glove_pose[hand_axis]['z']
        samples += 1
        time.sleep(0.1)
    for axis in hand_axis_calibration:
        hand_axis_calibration[axis] = hand_axis_calibration[axis]/samples
    if (hand_axis_calibration['x']**2 + hand_axis_calibration['y']**2) > 1:
	return False, deg, hand_axis_calibration
    elif hand_axis_calibration['z']>=0:
	hand_axis_calibration['z'] = math.sqrt(1 - hand_axis_calibration['x']**2 - hand_axis_calibration['y']**2)
    else: 
        hand_axis_calibration['z'] = -(math.sqrt(1 - hand_axis_calibration['x']**2 - hand_axis_calibration['y']**2))
    x_diff = tracker_mov.x - x_init
    y_diff = tracker_mov.y - y_init
    deg = math.atan2(y_diff, x_diff)
    print(deg)
    print(axis + " axis calibration done")
    return True, deg, hand_axis_calibration


#New: record a new axis
def new_calibration():
    global path
    hand = {'pitch': {'x': 1, 'y': 0, 'z': 0}, 'roll': {'x': 0, 'y': 1, 'z': 0}, 'yaw': {'x': 0, 'y': 0, 'z': 1}}
    x_axis_success, deg_x, hand['pitch'] = calibrate_axis("X", "thumb finger", "pitch")
    y_axis_success, deg_y, hand['roll'] = calibrate_axis("Y", "middle finger", "roll")
    if not x_axis_success or not y_axis_success:
	print("Error 1, orientation of the hand was not correct")
	return False, 0, None

    print("Saving the configuration...")
    deg_diff = deg_y-deg_x #abs
    print(deg_diff)
    if deg_diff<0:
        deg_diff += math.pi*2
    if deg_diff<1.22 or deg_diff>1.92: #70 110
        print("Error 2, there must be 90 degress between +X and +Y axis")
        return False, 0, None #Calibration error, try again
    else:
        deg_correction = deg_diff - math.pi/2
        deg_rot = deg_x + deg_correction/2
        if deg_rot<0:
            deg_rot += math.pi*2

    if hand['pitch']['z'] > 0.4 or hand['roll']['z'] > 0.4:
        print("Error 3, orientation of the hand was not correct")
	print("pitch" + str(hand['pitch']['z']) + "roll " + str(hand['roll']['z']))
        return False, 0, None #Calibration error, try again
    else:
	P_ =  [hand['pitch']['x'], hand['pitch']['y'], hand['pitch']['z']]
	R_ =  [hand['roll']['x'], hand['roll']['y'], hand['roll']['z']]
	Y_ = np.cross(P_, R_) #Vectorial product
	print(str(P_))
	print(str(R_))
	print(str(Y_))
	hand['yaw']['x'] = Y_[0]
	hand['yaw']['y'] = Y_[1]
	hand['yaw']['z'] = Y_[2]
        #for axis in hand['pitch']:
            #hand['yaw'][axis] = math.sqrt(1 - hand['yaw'][axis]**2 - hand['roll'][axis]**2)
        #R_tracker_glove = R_tracker_hand * R_hand_glove = I * R_glove_hand(-1) = R_glove_hand(T)
        #R_glove_hand = np.array([[hand['pitch']['x'], hand['roll']['x'], hand['yaw']['x']],
        #                   [hand['pitch']['y'], hand['roll']['y'], hand['yaw']['y']],
        #                   [hand['pitch']['z'], hand['roll']['z'], hand['yaw']['z']]])

	#Traspose matrix
        R_message = rotation_matrix()
        R_message.pitch.x = hand['pitch']['x']
        R_message.pitch.y = hand['roll']['x']
        R_message.pitch.z = Y_[0]
        R_message.roll.x = hand['pitch']['y']
        R_message.roll.y = hand['roll']['y']
        R_message.roll.z = Y_[1]
        R_message.yaw.x = hand['pitch']['z']
        R_message.yaw.y = hand['roll']['z']
        R_message.yaw.z = Y_[2]

    #Saving calibration angle in txt file
    try:
        f = open(path, "w")
        f.write("xy degree:" + str(deg_rot) + "\n")
        f.write("R glove_tracker pitch:" + str(R_message.pitch.x) + ',' + str(R_message.pitch.y) + ',' + str(R_message.pitch.z) + "\n")
        f.write("R glove_tracker roll:" + str(R_message.roll.x) + ',' + str(R_message.roll.y) + ',' + str(R_message.roll.z) + "\n")
        f.write("R glove_tracker yaw:" + str(R_message.yaw.x) + ',' + str(R_message.yaw.y) + ',' + str(R_message.yaw.z) + "\n")
        f.close()
    except:
        print("Error 4 saving the calibration file")
        return False, deg_rot, R_message

    print("New calibration done succesfully")
    return True, deg_rot, R_message


#Ask for callibration: last, new, none
def user_calibration():
    global deg_rot
    calibration_success = False
    R_message = rotation_matrix()
    while not calibration_success:
        ok_option = False
        print("Select a calibration option: ")
        print("[0] Use existing calibration")
        print("[1] New calibration")
        print("[2] Default (No calibration)")
        while not ok_option:
            try:
                calibration_op = int(input())
            except: 
                calibration_op = 99
            if isinstance(calibration_op, int):
                if calibration_op in range(3):
                    ok_option = True
            if not ok_option:
                print("Wrong number, write it again")
            else:
                if calibration_op == 2:
                    print("Default calibration option selected")
                    deg_rot = 0
                    calibration_success = True
                else:
                    if calibration_op == 0:
                        print("Existing calibration option selected")
                        calibration_success, deg_rot, R_message = load_calibration()
                    else:
                        print("New calibration option selected")
                        calibration_success, deg_rot, R_message = new_calibration()
		    if calibration_success:
		        #Calibration publishers
		        deg_rot_message = Float32()
		        deg_rot_message.data = deg_rot
                        calibration_degree_pub.publish(deg_rot_message)
			print(R_message)
                        R_glove_tracker_pub.publish(R_message)


#Subscriptors
def right_tracker_cb(info):
    global tracker_mov
    tracker_mov = info

subsTrackerR = rospy.Subscriber('/Tracker/right_hand_movement_xyz', discrete_movement, right_tracker_cb)


def right_glove_cb(info):
    global glove_pose
    glove_pose['pitch']['x'] = info.pitch.x
    glove_pose['pitch']['y'] = info.pitch.y
    glove_pose['pitch']['z'] = info.pitch.z
    glove_pose['roll']['x'] = info.roll.x
    glove_pose['roll']['y'] = info.roll.y
    glove_pose['roll']['z'] = info.roll.z
    glove_pose['yaw']['x'] = info.yaw.x
    glove_pose['yaw']['y'] = info.yaw.y
    glove_pose['yaw']['z'] = info.yaw.z

subsGloveR = rospy.Subscriber('/Glove/right_hand_orientation', rotation_matrix, right_glove_cb)


if __name__ == "__main__":
    user_calibration()
    print("Tracker has been calibrated, you can start your demonstrations")

    # Keep this process running until Enter is pressed
    print ("Press Enter to quit...")
    try:
	sys.stdin.readline()
    except KeyboardInterrupt:
	pass
