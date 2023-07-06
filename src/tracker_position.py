#!/usr/bin/env python

#Import libraries
import rospy
import PyKDL 
import numpy as np
import math 
import tf 
import time
import os
import copy
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from std_msgs.msg import Float32
from teaching_pkg.msg import *
from os.path import abspath


####### Initialization #######
#ROS init
rospy.init_node('tracker_position_analysis')
br = tf.TransformBroadcaster()
tracker_position_xyz_pub = rospy.Publisher('/Tracker/right_hand_movement_xyz', discrete_movement, queue_size=1)
tracker_position_rpy_pub = rospy.Publisher('/Tracker/right_hand_movement_rpy', discrete_movement, queue_size=1)
coord_pub = rospy.Publisher('/Teaching/record_coordinates', Float32, queue_size=1)


#Global variables
record_demo = False
write_times = False
calibrate_tool = False
path_tool_cali = ""
demo_time = 0
path_xyz = os.path.join(os.path.dirname(__file__), '../files/zz_other/example/hand_xyz.txt')
path_rpy = os.path.join(os.path.dirname(__file__), '../files/zz_other/example/hand_rpy.txt')
path_x = os.path.join(os.path.dirname(__file__), '../files/zz_other/example/x.txt')
path_y = os.path.join(os.path.dirname(__file__), '../files/zz_other/example/y.txt')
path_z = os.path.join(os.path.dirname(__file__), '../files/zz_other/example/z.txt')
path_R = os.path.join(os.path.dirname(__file__), '../files/zz_other/example/R.txt')
path_P = os.path.join(os.path.dirname(__file__), '../files/zz_other/example/P.txt')
path_Y = os.path.join(os.path.dirname(__file__), '../files/zz_other/example/Y.txt')
path_general = os.path.join(os.path.dirname(__file__), '../files/zz_other/example/demonstration.txt')
path_general_complete = os.path.join(os.path.dirname(__file__), '../files/zz_other/example/demonstration_complete.txt')
path_coord = os.path.join(os.path.dirname(__file__), '../files/zz_other/example/coordinates.txt')
f_xyz = open(path_xyz, "a")
f_rpy = open(path_rpy, "a")
f_x = open(path_x, "a")
f_y = open(path_y, "a")
f_z = open(path_z, "a")
f_R = open(path_R, "a")
f_P = open(path_P, "a")
f_Y = open(path_Y, "a")
f_general = open(path_general, "a")
f_general_complete = open(path_general_complete, "a")
f_coord = open(path_coord, "a")

hand_frame = PyKDL.Frame()

change_xyz = False
change_rpy = False
prev_movement = False
update_general = False
z_message = ""
xy_mov_prev = ""
long_displacement_detected = 0.3

contact_info = "NC"

deg_rot = 0
 
right_tracker_info = leap_hand() #not in use
initial_time = rospy.get_time() #not in use
period = 0.15 #not in use

status = {'x': '', 'y': '', 'z': '', 'pitch': '', 'roll': '', 'yaw': ''}
change_value = {'x': 0, 'y': 0, 'z': 0, 'pitch': 0, 'roll': 0, 'yaw': 0}

initial_count_xyz = 0; initial_count_rpy = 0

position_xyz = {'x': {'k-10': 0, 'k-20': 0, 'k-30': 0, 'k-2-1': 0, 'k-3-2': 0, 'status': '', 'stop': 0},
	'y': {'k-10': 0, 'k-20': 0, 'k-30': 0, 'k-2-1': 0, 'k-3-2': 0, 'status': '', 'stop': 0}, 
	'z': {'k-10': 0, 'k-20': 0, 'k-30': 0, 'k-2-1': 0, 'k-3-2': 0, 'status': '', 'stop': 0}}

position_rpy = {'pitch': {'k-10': 0, 'k-20': 0, 'k-30': 0, 'k-2-1': 0, 'k-3-2': 0, 'status': '', 'stop': 0},
	'roll': {'k-10': 0, 'k-20': 0, 'k-30': 0, 'k-2-1': 0, 'k-3-2': 0, 'status': '', 'stop': 0}, 
	'yaw': {'k-10': 0, 'k-20': 0, 'k-30': 0, 'k-2-1': 0, 'k-3-2': 0, 'status': '', 'stop': 0}}

glove_pose = {'roll': {'x': 1, 'y': 0, 'z': 0},
	 'pitch': {'x': 0, 'y': 1, 'z': 0}, 
	 'yaw': {'x': 0, 'y': 0, 'z': 1}}

x_tracker = 0; y_tracker = 0; z_tracker = 0
x_tracker_scaled = 0; y_tracker_scaled = 0; z_tracker_scaled = 0
x_ori = 0; y_ori = 0; z_ori = 0 #Origin
x_hand = 0; y_hand = 0; z_hand = 0
axis_scale = {'x': 1, 'y': 1, 'z': 1}
new_position = False; new_orientation = False

p_1 = np.array([0, 0, 0]); p_2 = np.array([0, 0, 0]); p_3 = np.array([0, 0, 0]); p_21 = np.array([0, 0, 0]); p_32 = np.array([0, 0, 0])
HTM_3F = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
HTM_2F = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]) 
HTM_1F = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
HTM_21 = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
HTM_32 = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])

eps1 = {'x': 0.05, 'y': 0.035, 'z': 0.05, 'pitch': 0.045, 'roll': 0.045, 'yaw': 0.045}
eps2 = {'x': 0, 'y': 0, 'z': 0, 'pitch': 0, 'roll': 0, 'yaw': 0}
eps3 = {'x': 0, 'y': 0, 'z': 0, 'pitch': 0, 'roll': 0, 'yaw': 0}
epsFast = {'x': 0, 'y': 0, 'z': 0, 'pitch': 0, 'roll': 0, 'yaw': 0}
for axis in eps1:
    eps2[axis] = eps1[axis] * 1.2
    eps3[axis] = eps1[axis] * 1.35
    epsFast[axis] = eps1[axis] * 3


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


#Function for broadcasting transforms
def broadcastTransform(br, frame, frame_id, parent_frame, time=rospy.get_rostime()): 
    br.sendTransform((frame.p.x(), frame.p.y(), frame.p.z()), 
        frame.M.GetQuaternion(), 
        time, 
        frame_id, 
        parent_frame) 


def R_to_quat(R):
	qw = (math.sqrt(1 + R['roll']['x'] + R['pitch']['y'] + R['yaw']['z']))/2
	qx = (R['pitch']['z'] - R['yaw']['y'])/( 4 * qw)
	qy = (R['yaw']['x'] - R['roll']['z'])/( 4 * qw)
	qz = (R['roll']['y'] - R['pitch']['x'])/( 4 * qw)
	return [qx, qy, qz, qw]


####### Data analysis functions #######
def analyze_movement(position_change):
    global status
    global change_value
    #20, 1.3, 1.5, 2.5 #0.04 1.2 1.4 2.5
    global eps1, eps2, eps3, epsFast
    global record_demo
    global change_xyz
    global change_rpy
    global prev_movement
    global update_general
    global long_displacement_detected
    stop_detected = 3

    movement = False
    change_xyz = False
    change_rpy = False

    for axis in position_change:

            position_change[axis]['status'] = 'Stop'
            status_change = False
            direction_change = False

            if position_change[axis]['k-10'] >= 0:
                if position_change[axis]['k-10'] >= eps1[axis]:
		    change_value[axis] += position_change[axis]['k-10']
                    if position_change[axis]['k-10'] >= epsFast[axis]:
                        position_change[axis]['status'] = 'Pos: Fast'
                    else:
                        position_change[axis]['status'] = 'Pos'

                elif position_change[axis]['k-2-1'] >= 0:
                        if position_change[axis]['k-20'] >= eps2[axis]:
                            position_change[axis]['status'] = 'Pos'
			    change_value[axis] += position_change[axis]['k-10']

                        elif position_change[axis]['k-3-2'] >= 0:
                                if position_change[axis]['k-30'] >= eps3[axis]:
                                    position_change[axis]['status'] = 'Pos'
				    change_value[axis] += position_change[axis]['k-10']

            else:
                if position_change[axis]['k-10'] <= -eps1[axis]:
		    change_value[axis] += position_change[axis]['k-10']
                    if position_change[axis]['k-10'] <= -epsFast[axis]:
                        position_change[axis]['status'] = 'Neg: Fast'
                    else:
                        position_change[axis]['status'] = 'Neg'

                elif position_change[axis]['k-2-1'] < 0:
                        if position_change[axis]['k-20'] <= -eps2[axis]:
                            position_change[axis]['status'] = 'Neg'
			    change_value[axis] += position_change[axis]['k-10']

                        elif position_change[axis]['k-3-2'] < 0:
                                if position_change[axis]['k-30'] <= -eps3[axis]:
                                    position_change[axis]['status'] = 'Neg'
				    change_value[axis] += position_change[axis]['k-10']

            if position_change[axis]['status'] == 'Stop':
                position_change[axis]['stop'] += 1
            
            else:
                position_change[axis]['stop'] = 0
                if status[axis] != position_change[axis]['status']:
		    if ((status[axis][:3] != position_change[axis]['status'][:3]) and (status[axis] != "Stop")):		        
			direction_change = True
                    status[axis] = position_change[axis]['status']
                    status_change = True

            if position_change[axis]['stop'] == stop_detected:
                status[axis] = "Stop"
                status_change = True
		direction_change = True

	    #if axis == 'roll' or axis == 'pitch' or axis == 'yaw':
	    #if axis == 'x' or axis == 'y' or axis == 'z':
	    if True:
		#print("" + axis + ": " + str(position_change[axis]['k-10']))
		if direction_change:
		    ##################
		    """
		    print("" + axis + ": " + str(change_value[axis]) + " displacement")  #Uncomment to visualize
		    if record_demo:
			if axis == 'x' or axis == 'y' or axis == 'z':
				write_demo("xyz", "" + axis + ": " + str(change_value[axis]) + " displacement")
			if axis == 'roll' or axis == 'pitch' or axis == 'yaw':
				write_demo("rpy", "" + axis + ": " + str(change_value[axis]) + " displacement")
		    """
		    ##################
		    ##################
		    if ((change_value[axis] >= long_displacement_detected) and record_demo):
			if axis == 'x_dist' or axis == 'y_dist' or axis == 'z_dist':
				write_demo(axis, " long")
		    ##################
		    change_value[axis] = 0
            	if status_change:
                #if calibrated:
                    #print("" + axis + ": " + status[axis])  #Uncomment to visualize
		    if record_demo:
			#Record the files of the movements in the indivual axis
			write_demo(axis, "\n" + status[axis])
		    #print("" + axis + ": " + str(position_change[axis]['k-10']))
		    #pass

    #Consider movement only when x, y or z is moving (for the global)
    if ((status['x'] != "Stop") or (status['y'] != "Stop") or (status['z'] != "Stop")):
	movement = True

    if record_demo:
	    #Writes in the general file when there is a movement update (considering just movement or stop)
	    if ((movement is not prev_movement) or update_general):
		if movement:
			write_demo("general", "movement")
			#print("MOV")
		else:
			write_demo("general", "stop")
			#print("STOP")
		update_general = False

	    #Writes in the xyz or rpy files if there have been movement changes in any of these axis
	    if change_xyz:
		write_demo("xyz_EOL", "")
	    if change_rpy:
		write_demo("rpy_EOL", "")

    prev_movement = movement

		

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
    HTM_F0 = np.array([[glove_pose['roll']['x'], glove_pose['pitch']['x'], glove_pose['yaw']['x'], x_tracker],
                            [glove_pose['roll']['y'], glove_pose['pitch']['y'], glove_pose['yaw']['y'], y_tracker],
                            [glove_pose['roll']['z'], glove_pose['pitch']['z'], glove_pose['yaw']['z'], z_tracker],
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
	if axis == 'roll':
		row = 0
	elif axis == 'pitch':
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
    tracker_msg.x_state = position_rpy['roll']['status']
    tracker_msg.y_state = position_rpy['pitch']['status']
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


def write_demo(type_message, message):
    global f_xyz
    global f_rpy
    global f_x
    global f_y
    global f_z
    global f_R
    global f_P
    global f_Y
    global f_general
    global path_general
    global f_general_complete
    global path_general_complete
    global change_xyz
    global change_rpy
    global status
    global contact_info
    global z_message
    global xy_mov_prev
    global write_times
    global demo_time

    current_time = 0
    if write_times:
	current_time = rospy.get_time() - demo_time
	coord_message = Float32()
    	coord_message.data = current_time
	coord_pub.publish(coord_message) #Signal to write the coordinates.txt

    if type_message == "x" or type_message == "all" or type_message == "x_dist":
	if type_message == "all":
    		f_x.write(str(status["x"]))
	else:
		f_x.write(message)
	change_xyz = True
    if type_message == "y" or type_message == "all" or type_message == "y_dist":
    	if type_message == "all":
    		f_y.write(str(status["y"]))
	else:
		f_y.write(message)
	change_xyz = True
    if type_message == "z" or type_message == "all" or type_message == "z_dist":
    	if type_message == "all":
		if write_times:
    			f_z.write(str(current_time) + ":" + str(status["z"]))
		else:
			f_z.write(str(status["z"]))
	else:
		if write_times:
			f_z.write("\n" + str(current_time) + ":" + str(status["z"]))
		else:
			f_z.write(message)	
	change_xyz = True
    if type_message == "roll" or type_message == "all":
    	f_R.write(str(status["roll"]) + "\n")
	change_rpy = True
    if type_message == "pitch" or type_message == "all":
    	f_P.write(str(status["pitch"]) + "\n")
	change_rpy = True
    if type_message == "yaw" or type_message == "all":
	if write_times:
		f_Y.write(str(current_time) + ":" + str(status["yaw"]) + "\n")
	else:
    		f_Y.write(str(status["yaw"]) + "\n")
	change_rpy = True

    if type_message == "xyz_EOL" or type_message == "all":
	if write_times:
    		f_xyz.write(str(current_time) + ":" + str(status["x"]) + "," + str(status["y"]) + "," + str(status["z"]) + "\n")
	else:
		f_xyz.write(str(status["x"]) + "," + str(status["y"]) + "," + str(status["z"]) + "\n")

    if type_message == "rpy_EOL" or type_message == "all":
	if write_times:
    		f_rpy.write(str(current_time) + ":" + str(status["roll"]) + "," + str(status["pitch"]) + "," + str(status["yaw"]) + "\n")
	else:
		f_rpy.write(str(status["roll"]) + "," + str(status["pitch"]) + "," + str(status["yaw"]) + "\n")

    if type_message == "general":
	f_general = open(path_general, "a")
	if write_times:
    		message = str(current_time) + ":" + message
	f_general.write(message + "," + contact_info + "\n")
	f_general.close()

    if type_message == "z":
	if status['z'][:2] != z_message:
		z_message = status['z'][:2]
		f_general_complete = open(path_general_complete, "a")
		if write_times:
			f_general_complete.write(str(current_time) + ":")
		f_general_complete.write("Z " + z_message + "," + contact_info + "\n")
		f_general_complete.close()

    if type_message == "x" or type_message == "y":
	xy_mov = ""
	if status["x"] == "Stop" and status["y"] == "Stop":
		xy_mov = "XY Stop"
	else:
		xy_mov = "XY Movement"
	if xy_mov != xy_mov_prev:
		xy_mov_prev = xy_mov
		f_general_complete = open(path_general_complete, "a")
		if write_times:
			f_general_complete.write(str(current_time) + ":")
		f_general_complete.write(xy_mov + "," + contact_info + "\n")
		f_general_complete.close()


####### Subscriptors #########
def right_tracker_cb(info):
    #global period
    #global initial_time
    #global global_recording
    global deg_rot
    global x_tracker, y_tracker, z_tracker
    global x_tracker_scaled, y_tracker_scaled, z_tracker_scaled
    global x_hand, y_hand, z_hand
    global new_position, new_orientation
    global x_ori, y_ori, z_ori
    global br
    global glove_pose
    global axis_scale
    global calibrate_tool
    global path_tool_cali
    global hand_frame

    position_VIVE = info.data
    x_tracker = float((position_VIVE.split(';'))[2]) #Horiz1 axis
    y_tracker = float((position_VIVE.split(';'))[0]) #Horiz2 axis
    z_tracker =  float((position_VIVE.split(';'))[1]) #Vert axis
    #print("raw xyz: " + str(x_tracker)+", "+str(y_tracker)+", "+str(z_tracker))
    #print(str(z_tracker))
    x_tracker_copy = copy.deepcopy(x_tracker)
    y_tracker_copy = copy.deepcopy(y_tracker)
    if not deg_rot==0:
	x_tracker = x_tracker_copy * math.cos(deg_rot) + y_tracker_copy * math.sin(deg_rot)
	y_tracker = -x_tracker_copy * math.sin(deg_rot) + y_tracker_copy * math.cos(deg_rot)
    #print("aligned xyz: " + str(x_tracker)+", "+str(y_tracker)+", "+str(z_tracker))
    analyze_movement_xyz()
    #This scaled values should be used for everything (including the position analysis)
    x_tracker_scaled = x_tracker - x_ori
    y_tracker_scaled = y_tracker - y_ori
    z_tracker_scaled = z_tracker - z_ori
    #print("tared xyz: " + str(x_tracker_scaled)+", "+str(y_tracker_scaled)+", "+str(z_tracker_scaled))

    x_tracker_scaled *= axis_scale['x']
    y_tracker_scaled *= axis_scale['y']
    z_tracker_scaled *= axis_scale['z']
    #print("scaled xyz: " + str(x_tracker_scaled)+", "+str(y_tracker_scaled)+", "+str(z_tracker_scaled))


    #Broadcast tfs
    current_time = rospy.get_rostime()
    #wrist
    wrist_frame = PyKDL.Frame()
    wrist_frame.p = PyKDL.Vector(x_tracker_scaled, y_tracker_scaled, z_tracker_scaled)
    #rot_x = []
    #rot = PyKDL.Rotation()
    quat = R_to_quat(glove_pose)
    wrist_frame.M = PyKDL.Rotation.Quaternion(quat[0], quat[1], quat[2], quat[3])
    #wrist_frame.M = PyKDL.Rotation.Quaternion(0,0,0,1)
    #print(wrist_frame)
    name = 'right_wrist'
    broadcastTransform(br, wrist_frame, name, 'world', time=current_time)
    #hand
    WH_frame = PyKDL.Frame()
    WH_frame.p = PyKDL.Vector(0.1, 0, -0.05)
    WH_frame.M = PyKDL.Rotation.Quaternion(0,0,0,1)
    hand_frame = wrist_frame * WH_frame
    name = 'right_hand'
    broadcastTransform(br, hand_frame, name, 'world', time=current_time)
    x_hand = hand_frame.p.x()
    y_hand = hand_frame.p.y()
    z_hand = hand_frame.p.z()
    if calibrate_tool:
	hand_world_frame = hand_frame.Inverse()
	f = open(path_tool_cali, "w")
        f.write("x:" + str(hand_world_frame.p.x()) + "\n")
        f.write("y:" + str(hand_world_frame.p.y()) + "\n")
        f.write("z:" + str(hand_world_frame.p.z()) + "\n")
        f.close()
	calibrate_tool = False

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
    glove_pose['roll']['x'] = info.roll.x
    glove_pose['roll']['y'] = info.roll.y
    glove_pose['roll']['z'] = info.roll.z
    glove_pose['pitch']['x'] = info.pitch.x
    glove_pose['pitch']['y'] = info.pitch.y
    glove_pose['pitch']['z'] = info.pitch.z
    glove_pose['yaw']['x'] = info.yaw.x
    glove_pose['yaw']['y'] = info.yaw.y
    glove_pose['yaw']['z'] = info.yaw.z
    new_orientation = True
    if new_position:
	analyze_movement_rpy()	

subsGloveR = rospy.Subscriber('/Glove/right_hand_orientation', rotation_matrix, right_glove_cb)


def start_cb(info):
    global record_demo
    global f_xyz
    global f_rpy
    global f_x
    global f_y
    global f_z
    global f_R
    global f_P
    global f_Y
    global path_xyz
    global path_rpy
    global path_x
    global path_y
    global path_z
    global path_R
    global path_P
    global path_Y
    global path_general
    global path_general_complete
    global path_coord
    global update_general
    global write_times
    global demo_time
    path = info.path
    write_times = info.times
    demo_time = rospy.get_time()
    record_demo = True
    path_xyz = path + '/hand_xyz.txt'
    path_rpy = path + '/hand_rpy.txt'
    path_x = path + '/x.txt'
    path_y = path + '/y.txt'
    path_z = path + '/z.txt'
    path_R = path + '/R.txt'
    path_P = path + '/P.txt'
    path_Y = path + '/Y.txt'
    path_general = path + '/demonstration.txt'
    path_general_complete = path + '/demonstration_complete.txt'
    path_coord = path + '/coordinates.txt'
    f_xyz = open(path_xyz, "w")
    f_rpy = open(path_rpy, "w")
    f_x = open(path_x, "w")
    f_y = open(path_y, "w")
    f_z = open(path_z, "w")
    f_R = open(path_R, "w")
    f_P = open(path_P, "w")
    f_Y = open(path_Y, "w")
    write_demo("all", "")
    update_general = True

subsStart = rospy.Subscriber('/Teaching/start_demo', start_demo_msg, start_cb)


def stop_cb(info):
    global record_demo
    global f_xyz
    global f_rpy
    global f_x
    global f_y
    global f_z
    global f_R
    global f_P
    global f_Y
    record_demo = False
    f_xyz.close()
    f_rpy.close()
    f_x.close()
    f_y.close()
    f_z.close()
    f_R.close()
    f_P.close()
    f_Y.close()

subsStop = rospy.Subscriber('/Teaching/stop_demo', Bool, stop_cb)


def contact_cb(info):
    global contact_info
    if info.data:
    	contact_info = "C"
    else:
	contact_info = "NC"

subsContact = rospy.Subscriber('/Glove/contact', Bool, contact_cb)


def coord_cb(info):
    global x_hand, y_hand, z_hand
    global hand_frame
    global path_coord
    current_time = info.data
    Rx_hand, Ry_hand, Rz_hand, Rw_hand = hand_frame.M.GetQuaternion()
    f_coord = open(path_coord, "a")
    f_coord.write(str(current_time) + ":" + str(x_hand) + "," + str(y_hand) + "," + str(z_hand) + "," + str(Rx_hand) + "," + str(Ry_hand) + "," + str(Rz_hand) + "," + str(Rw_hand) + "\n")
    f_coord.close()

subsCoord = rospy.Subscriber('/Teaching/record_coordinates', Float32, coord_cb)


def origin_cb(info):
    global x_tracker, y_tracker, z_tracker
    global x_ori, y_ori, z_ori
    x_ori = x_tracker
    y_ori = y_tracker
    z_ori = z_tracker

subsOrigin = rospy.Subscriber('/Teaching/save_origin', Bool, origin_cb)


def axis_scale_cb(info):
    global x_tracker_scaled, y_tracker_scaled, z_tracker_scaled
    global axis_scale
    if info.axis == 'x':
	    axis_scale['x'] = abs(float(info.scale)/(x_tracker_scaled*100))
    elif info.axis == 'y':
	    axis_scale['y'] = abs(float(info.scale)/(y_tracker_scaled*100))
    elif info.axis == 'z':
	    axis_scale['z'] = abs(float(info.scale)/(z_tracker_scaled*100))

subsOrigin = rospy.Subscriber('/Teaching/axis_scale', axis_scale_msg, axis_scale_cb)


def cali_tool_cb(info):
    global calibrate_tool
    global path_tool_cali
    calibrate_tool = True
    path_tool_cali = info.data

subsOrigin = rospy.Subscriber('/Teaching/save_calibration_tool', String, cali_tool_cb)


####### Quit #######
print('Tracker node ready')
 # Keep this process running until Enter is pressed
print ("Press Enter to quit...")
try:
	sys.stdin.readline()
except KeyboardInterrupt:
	print("Tracker node finalized")
	pass
