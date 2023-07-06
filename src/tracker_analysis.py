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
from teaching_pkg.msg import *
from os.path import abspath


#ROS init
rospy.init_node('tracker_analysis')
R_glove_tracker_pub = rospy.Publisher('/Glove/R_glove_tracker', rotation_matrix, queue_size=1)


#Global variables
deg_rot = 0 
calibrated = False
right_tracker_info = leap_hand() #change type
initial_time = rospy.get_time()
period = 0.15
status = {'x': '', 'y': '', 'z': ''}
stop_count = {'x': 0, 'y': 0, 'z': 0}
initial_count = {'x': 0, 'y': 0, 'z': 0}
position = {'x': {'k': 0, 'k-1': 0, 'k-2': 0, 'k-3': 0, 'status': ''}, 'y': {'k': 0, 'k-1': 0, 'k-2': 0, 'k-3': 0, 'status': ''}, 'z': {'k': 0, 'k-1': 0, 'k-2': 0, 'k-3': 0, 'status': ''}}
glove_pose = {'pitch': {'x': 1, 'y': 0, 'z': 0}, 'roll': {'x': 0, 'y': 1, 'z': 0}, 'yaw': {'x': 0, 'y': 0, 'z': 1}}
path = os.path.join(os.path.dirname(__file__), '../files/calibration.txt')


#Movements discretizing
def analyze_movement(pose_right):
    global position
    global status
    global stop_count
    global initial_count
    global calibrated
    global deg_rot

    #10, 1.3, 1.5, 2.5
    eps1 = 20
    eps2 = eps1 * 1.3
    eps3 = eps1 * 1.5
    epsFast = eps1 * 2.5
    stop_detected = 3

    if not pose_right.detected:
        for axis in position:
            initial_count[axis] = 0
            stop_count[axis] = 0
            status[axis] = ''
            position[axis]['status'] = ''
        print("Hand is not detected")
	
    else:
        if calibrated:
            position['x']['k'] = pose_right.pose_hand.position.x * math.cos(deg_rot) + pose_right.pose_hand.position.y * math.sin(deg_rot)
            position['y']['k'] = -pose_right.pose_hand.position.x * math.sin(deg_rot) + pose_right.pose_hand.position.y * math.cos(deg_rot)
        else:
            position['x']['k'] = pose_right.pose_hand.position.x
            position['y']['k'] = pose_right.pose_hand.position.y
        position['z']['k'] = pose_right.pose_hand.position.z

        for axis in position:

            if initial_count[axis] < 3:
                position[axis]['k-3'] = position[axis]['k-2']
                position[axis]['k-2'] = position[axis]['k-1']
                position[axis]['k-1'] = position[axis]['k']
                initial_count[axis] += 1
                break

            position[axis]['status'] = 'Stop'
            status_change = False
            
            if position[axis]['k'] >= position[axis]['k-1']:
                if position[axis]['k'] - position[axis]['k-1'] >= eps1:
                    if position[axis]['k'] - position[axis]['k-1'] >= epsFast:
                        position[axis]['status'] = 'Pos Fast'
                    else:
                        position[axis]['status'] = 'Pos'

                elif position[axis]['k-1'] > position[axis]['k-2']:
                        if position[axis]['k'] - position[axis]['k-2'] >= eps2:
                            position[axis]['status'] = 'Pos'

                        elif position[axis]['k-2'] > position[axis]['k-3']:
                                if position[axis]['k'] - position[axis]['k-3'] >= eps3:
                                    position[axis]['status'] = 'Pos'

            else:
                if position[axis]['k-1'] - position[axis]['k'] >= eps1:
                    if position[axis]['k-1'] - position[axis]['k'] >= epsFast:
                        position[axis]['status'] = 'Neg Fast'
                    else:
                        position[axis]['status'] = 'Neg'

                elif position[axis]['k-1'] < position[axis]['k-2']:
                        if position[axis]['k-2'] - position[axis]['k'] >= eps2:
                            position[axis]['status'] = 'Neg'

                        elif position[axis]['k-2'] < position[axis]['k-3']:
                                if position[axis]['k-3'] - position[axis]['k'] >= eps3:
                                    position[axis]['status'] = 'Neg'

            if position[axis]['status'] == 'Stop':
                stop_count[axis] += 1
            
            else:
                stop_count[axis] = 0
                if status[axis] != position[axis]['status']:
                    status[axis] = position[axis]['status']
                    status_change = True

            if stop_count[axis] == stop_detected:
                status[axis] = "Stop"
                status_change = True

            if status_change:
            #if axis == 'x':
                if calibrated:
                    print("" + axis + ": " + status[axis])

            position[axis]['k-3'] = position[axis]['k-2']
            position[axis]['k-2'] = position[axis]['k-1']
            position[axis]['k-1'] = position[axis]['k']


#Last: load the previous axis from a txt
def load_calibration():
    global path
    deg_rot = 0
    R_message = rotation_matrix()
    try:
        f = open(path, "r")
        deg_rot = float((split.f[0](':'))[1])
        pitch = (split.f[1](':'))[1]
        roll = (split.f[2](':'))[1]
        yaw = (split.f[3](':'))[1]
        R_message.pitch.x = float(split.pitch(',')[0])
        R_message.pitch.y = float(split.pitch(',')[1])
        R_message.pitch.z = float(split.pitch(',')[2])
        R_message.roll.x = float(split.roll(',')[0])
        R_message.roll.y = float(split.roll(',')[1])
        R_message.roll.z = float(split.roll(',')[2])
        R_message.yaw.x = float(split.yaw(',')[0])
        R_message.yaw.y = float(split.yaw(',')[1])
        R_message.yaw.z = float(split.yaw(',')[2])
        f.close()
    except:
        print("Error loading the calibration file")
        return False, 0
    print("Calibration loaded succesfully")
    return True, deg_rot, R_message


def calibrate_axis(axis, finger, hand_axis):
    global position
    global glove_pose
    hand_axis_calibration = {'x': 0, 'y': 0, 'z': 0}
    samples = 0

    print("Move your right hand up and down to start calibrating " + axis + " axis")
    while not ((position['z']['status'] == 'Pos') or (position['z']['status'] == 'Pos Fast')):
        time.sleep(0.1)
    while not ((position['z']['status'] == 'Neg') or (position['z']['status'] == 'Neg Fast')):
        time.sleep(0.1)

    print("Align your right hand " + finger + " with the + " + axis + " axis and move your hand in that direction")
    while ((position['x']['status'] == 'Stop') or (position['y']['status'] == 'Stop')):
        time.sleep(0.1)
    x_init = position['x']['k-1']
    y_init = position['y']['k-1']
    while not ((position['x']['status'] == 'Stop') or (position['y']['status'] == 'Stop')):
        hand_axis_calibration['x'] += glove_pose[hand_axis]['x']
        hand_axis_calibration['y'] += glove_pose[hand_axis]['y']
        hand_axis_calibration['z'] += glove_pose[hand_axis]['z']
        samples += 1
        time.sleep(0.1)
    for axis in hand_axis_calibration:
        hand_axis_calibration[axis] = hand_axis_calibration[axis]/samples
    hand_axis_calibration['z'] = math.sqrt(1 - hand_axis_calibration['x']**2 + hand_axis_calibration['y']**2)
    x_diff = position['x']['k-1'] - x_init
    y_diff = position['y']['k-1'] - y_init
    deg = math.atan(y_diff, x_diff)
    print(axis + " axis calibration done")
    return deg, hand_axis_calibration


#New: record a new axis
def new_calibration():
    global path
    hand = {'pitch': {'x': 1, 'y': 0, 'z': 0}, 'roll': {'x': 0, 'y': 1, 'z': 0}, 'yaw': {'x': 0, 'y': 0, 'z': 1}}
    deg_x, hand['pitch'] = calibrate_axis("X", "thumb finger", "pitch")
    deg_y, hand['roll'] = calibrate_axis("Y", "middle finger", "roll")

    print("Saving the configuration...")
    deg_diff = deg_y-deg_x
    if deg_diff<0:
        deg_diff += math.pi*2
    if deg_diff<70 or deg_diff>110:
        print("Error, there must be 90 degress between +X and +Y axis")
        return False, 0 #Calibration error, try again
    else:
        deg_correction = deg_diff - math.pi/2
        deg_rot = deg_x + deg_correction/2
        if deg_rot<0:
            deg_rot += math.pi*2

    if hand['pitch']['z'] > 0.5 or hand['roll']['z'] > 0.5:
        print("Error, orientation of the hand was not correct")
        return False, 0 #Calibration error, try again
    else:
        for axis in hand['pitch']:
            hand['yaw'][axis] = math.sqrt(1 - hand['yaw'][axis]**2 + hand['roll'][axis]**2)
        #R_tracker_glove = R_tracker_hand * R_hand_glove = I * R_glove_hand(-1) = R_glove_hand(T)
        #R_glove_hand = np.array([[hand['pitch']['x'], hand['roll']['x'], hand['yaw']['x']],
        #                   [hand['pitch']['y'], hand['roll']['y'], hand['yaw']['y']],
        #                   [hand['pitch']['z'], hand['roll']['z'], hand['yaw']['z']]])

        R_message = rotation_matrix()
        R_message.pitch.x = hand['pitch']['x']
        R_message.pitch.y = hand['roll']['x']
        R_message.pitch.z = hand['yaw']['x']
        R_message.roll.x = hand['pitch']['y']
        R_message.roll.y = hand['roll']['y']
        R_message.roll.z = hand['yaw']['y']
        R_message.yaw.x = hand['pitch']['z']
        R_message.yaw.y = hand['roll']['z']
        R_message.yaw.z = hand['yaw']['z']

    #Saving calibration angle in txt file
    try:
        f = open(path, "w")
        f.write("xy degree:" + str(deg_rot) + "\n")
        f.write("R glove_tracker pitch:" + str(hand['pitch']['x']) + ',' + str(hand['pitch']['y']) + ',' + str(hand['pitch']['z']) + "\n")
        f.write("R glove_tracker roll:" + str(hand['roll']['x']) + ',' + str(hand['roll']['y']) + ',' + str(hand['roll']['z']) + "\n")
        f.write("R glove_tracker yaw:" + str(hand['yaw']['x']) + ',' + str(hand['yaw']['y']) + ',' + str(hand['yaw']['z']) + "\n")
        f.close()
    except:
        print("Error saving the calibration file")
        return False, 0

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
        print("[2] Default")
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
                    #publish deg_rot
                    R_glove_tracker_pub.publish(R_message) #Define subscriber in the glove script


def user_reference_frame():
    print("Select a reference frame: ")
    print("[0] World reference frame")
    print("[1] Hand reference frame")
    while not ok_option:
        try:
            calibration_op = int(input())
        except: 
            calibration_op = 99
        if isinstance(calibration_op, int):
            if calibration_op in range(2):
                ok_option = True
        if not ok_option:
            print("Wrong number, write it again")
        else:
            #ToDo
            pass


#Subscriptors
def right_tracker_cb(info):
    #global period
    #global initial_time
    #global global_recording
    position_string = info.data
    position_VIVE = (split.position_string(':'))[1]
    right_tracker_info = leap_hand()
    right_tracker_info.detected = True
    right_tracker_info.pose_hand.position.x = float((split.position_VIVE(';'))[0])
    right_tracker_info.pose_hand.position.y = float((split.position_VIVE(';'))[1])
    right_tracker_info.pose_hand.position.z =  float((split.position_VIVE(';'))[2])
    #if ((rospy.get_time()-initial_time) > period):
    #    initial_time = rospy.get_time()
    #    analyze_movement(right_tracker_info
    analyze_movement(right_tracker_info)

subsTrackerR = rospy.Subscriber('Tracker/pose_right', String, right_tracker_cb) #Change topic and msg for the VIVE Tracker

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

subsGloveR = rospy.Subscriber('/Glove/right_hand_orientation', rotation_matrix, right_tracker_cb)


if __name__ == "__main__":
    global calibrated
    global initial_count
    global stop_count
    global status
    global position
    user_calibration()
    #user_reference_frame()
    for axis in position:
        initial_count[axis] = 0
        stop_count[axis] = 0
        status[axis] = ''
        position[axis]['status'] = ''
    calibrated = True
    print("Tracker has been calibrated, you can start your demonstrations")

    # Keep this process running until Enter is pressed
    print "Press Enter to quit..."
    try:
	sys.stdin.readline()
    except KeyboardInterrupt:
	pass
