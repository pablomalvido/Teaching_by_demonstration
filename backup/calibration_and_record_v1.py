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
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from teaching_pkg.msg import *
from os.path import abspath


#ROS init
rospy.init_node('calibration')
R_glove_tracker_pub = rospy.Publisher('/Glove/R_glove_tracker', rotation_matrix, queue_size=1)
calibration_degree_pub = rospy.Publisher('/Tracker/right_hand_calibration_degree', Float32, queue_size=1)
start_demo_pub = rospy.Publisher('/Teaching/start_demo', start_demo_msg, queue_size=1)
stop_demo_pub = rospy.Publisher('/Teaching/stop_demo', Bool, queue_size=1)
save_origin_pub = rospy.Publisher('/Teaching/save_origin', Bool, queue_size=1)


#Global variables
deg_rot = 0 
right_tracker_info = leap_hand() #change type
initial_time = rospy.get_time()
period = 0.15 #Not in use
path_cali = os.path.join(os.path.dirname(__file__), '../files/calibration.txt')
tracker_mov = discrete_movement()
glove_pose = {'roll': {'x': 1, 'y': 0, 'z': 0}, 'pitch': {'x': 0, 'y': 1, 'z': 0}, 'yaw': {'x': 0, 'y': 0, 'z': 1}}

#Last: load the previous axis from a txt
def load_calibration():
    global path_cali
    deg_rot = 0
    R_message = rotation_matrix()
    try:
	f = open(path_cali, "r")
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


def calibrate_axis(axis):
    global tracker_mov
    global glove_pose
    R_hand = {'roll': {'x': 1, 'y': 0, 'z': 0}, 'pitch': {'x': 0, 'y': 1, 'z': 0}, 'yaw': {'x': 0, 'y': 0, 'z': 1}}
    samples = 0

    print("Move your right hand up and down to start calibrating " + axis + " axis")
    while not ((tracker_mov.z_state == 'Pos') or (tracker_mov.z_state == 'Pos Fast')):
        time.sleep(0.1)
    while not ((tracker_mov.z_state == 'Neg') or (tracker_mov.z_state == 'Neg Fast')):
        time.sleep(0.1)

    print("Align your right hand axis with the world axis and move your hand in the " + axis + " direction")
    while ((tracker_mov.x_state == 'Stop') or (tracker_mov.y_state == 'Stop')):
        time.sleep(0.1)
    x_init = tracker_mov.x
    y_init = tracker_mov.y
    R_hand = glove_pose
    while not ((tracker_mov.x_state == 'Stop') and (tracker_mov.y_state == 'Stop')):
        time.sleep(0.1)
    x_diff = tracker_mov.x - x_init
    y_diff = tracker_mov.y - y_init
    deg = math.atan2(y_diff, x_diff)
    print(deg)
    print(axis + " axis calibration done")
    return True, deg, R_hand


#New: record a new axis
def new_calibration():
    global path_cali
    x_axis_success, deg_x, hand1 = calibrate_axis("X")
    y_axis_success, deg_y, hand2 = calibrate_axis("Y")
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

    #if hand1['pitch']['z'] > 0.35 or hand1['roll']['z'] > 0.35:
    #    print("Error 3, orientation of the hand was not correct")
	#print("pitch: " + str(hand1['pitch']['z']) + ", roll: " + str(hand1['roll']['z']))
        #return False, 0, None #Calibration error, try again
    if False:
	pass
    else:
	#Traspose matrix
        R_message = rotation_matrix()
        R_message.roll.x = hand1['roll']['x']
        R_message.roll.y = hand1['pitch']['x']
        R_message.roll.z = hand1['yaw']['x']
        R_message.pitch.x = hand1['roll']['y']
        R_message.pitch.y = hand1['pitch']['y']
        R_message.pitch.z = hand1['yaw']['y']
        R_message.yaw.x = hand1['roll']['z']
        R_message.yaw.y = hand1['pitch']['z']
        R_message.yaw.z = hand1['yaw']['z']

    #Saving calibration angle in txt file
    try:
        f = open(path_cali, "w")
        f.write("xy degree:" + str(deg_rot) + "\n")
	f.write("R glove_tracker roll:" + str(R_message.roll.x) + ',' + str(R_message.roll.y) + ',' + str(R_message.roll.z) + "\n")
        f.write("R glove_tracker pitch:" + str(R_message.pitch.x) + ',' + str(R_message.pitch.y) + ',' + str(R_message.pitch.z) + "\n")
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


def select_origin():
	print("Place your hand in the coordinates origin (0, 0, 0) and then:")
	print("[0] Save origin")
	print("[1] Skip")
	ok_option = False
	while not ok_option:
            try:
                origin_op = int(input())
            except: 
                origin_op = 99
            if isinstance(origin_op, int):
                if origin_op in range(2):
                    ok_option = True
            if not ok_option:
                print("Wrong number, write it again")
            else:
		if origin_op == 0:
			origin_msg = Bool()
			origin_msg.data = True
			save_origin_pub.publish(origin_msg)
		else:
			pass


def choose_directory(path, level):
	dir1 = os.listdir(path)

	dict_dir1 = {}
	i = 0
	print("Select the " + level + ":")
	for directory in dir1:
		if directory != "calibration.txt":
			i += 1
			dict_dir1[i] = directory
			print("[" + str(i) + "] " + directory)

	ok_option = False
	while not ok_option:
		try:
		        dir_op = int(input())
		except: 
		        dir_op = 99
		if isinstance(dir_op, int):
		        if dir_op in range(1, len(dict_dir1)+1):
		            ok_option = True
		if not ok_option:
		        print("Wrong number, write it again")
		else:
		        print("Selection: " + dict_dir1[dir_op])
			path = (path + "/" + dict_dir1[dir_op])
			return(path)


def new_demo():
    global tracker_mov

    path = "/home/remodel/catkin_ws/src/teaching_pkg/files"
    path = choose_directory(path, "object")
    path = choose_directory(path, "operation")

    start_message = start_demo_msg()
    if path[-8:] != "sequence":  #Demo operation
    	path = path + "/demos"
        dir1 = os.listdir(path)
        demos_map = map(int, dir1)
        demos_list = list(demos_map) 
        print("New demo: " + str(max(demos_list) + 1))
        path = path + "/" + str(max(demos_list) + 1)
	start_message.times = False
    else:  #Sequence (segmentation)
	dir1 = os.listdir(path)
        demos_map = map(lambda x: int(x.split("_")[0]), dir1) #To select only the number
        demos_list = list(demos_map) 
        print("New sequence: " + str(max(demos_list) + 1))
	print("Write a name for the sequence:")
	seq_name = ""
	try:
		seq_name = raw_input()
	except: 
		print("Failed saving the seq name")
        path = path + "/" + str(max(demos_list) + 1) + "_" + str(seq_name)
	start_message.times = True
    
    start_message.path = path
    os.mkdir(path)
    print(path)

    """
    ### Start with Tracker ###
    print("Move your right hand up and down to start recording a new demonstration")
    while not ((tracker_mov.z_state == 'Pos') or (tracker_mov.z_state == 'Pos Fast')):
        time.sleep(0.1)
    while not ((tracker_mov.z_state == 'Neg') or (tracker_mov.z_state == 'Neg Fast')):
        time.sleep(0.1)
    """
    #"""
    ### Start without Tracker ###
    print("Press enter to start recording a new the demo")
    try:
	sys.stdin.readline()
    except KeyboardInterrupt:
	pass
    #"""

    try:
	#path_general = os.path.join(os.path.dirname(__file__), '../files/demonstration.txt')
	path_general = path + '/demonstration.txt'
	#os.remove(path_general)
	#path_general_complete = os.path.join(os.path.dirname(__file__), '../files/demonstration_complete.txt')
	path_general_complete = path + '/demonstration_complete.txt'
	#os.remove(path_general_complete)
    except:
	print("demonstration.txt file doesn't exist")
    #start_message = String()
    #start_message.data = path
    start_demo_pub.publish(start_message)

    print("Recording... Press enter to stop the demo")
    try:
	sys.stdin.readline()
    except KeyboardInterrupt:
	pass
    stop_message = Bool()
    stop_message.data = True
    stop_demo_pub.publish(stop_message)
    print("Demonstration finished")

#Subscriptors
def right_tracker_cb(info):
    global tracker_mov
    tracker_mov = info

subsTrackerR = rospy.Subscriber('/Tracker/right_hand_movement_xyz', discrete_movement, right_tracker_cb)


def right_glove_cb(info):
    global glove_pose
    glove_pose['roll']['x'] = info.roll.x
    glove_pose['roll']['y'] = info.roll.y
    glove_pose['roll']['z'] = info.roll.z
    glove_pose['pitch']['x'] = info.pitch.x
    glove_pose['pitch']['y'] = info.pitch.y
    glove_pose['pitch']['z'] = info.pitch.z
    glove_pose['yaw']['x'] = info.yaw.x
    glove_pose['yaw']['y'] = info.yaw.y
    glove_pose['yaw']['z'] = info.yaw.z

subsGloveR = rospy.Subscriber('/Glove/right_hand_orientation', rotation_matrix, right_glove_cb)


if __name__ == "__main__":
    user_calibration()
    select_origin()
    print("Tracker has been calibrated, you can start your demonstrations")
    new_demo()

    ok_option = False
    while not ok_option:
	print("New demo?:")
    	print("[0] Yes")
    	print("[1] No")
	try:
	        new_op = int(input())
	except: 
	        new_op = 99
	if isinstance(new_op, int):
	        if new_op in range(2):
	            ok_option = True
	if not ok_option:
	        print("Wrong number, write it again")
	elif new_op == 0:
		new_demo()
		ok_option = False #To ask again after the demo

    # Keep this process running until Enter is pressed
    print ("Press Enter to quit...")
    try:
	sys.stdin.readline()
    except KeyboardInterrupt:
	pass