#!/usr/bin/env python
from std_msgs.msg import String
import tf 
import PyKDL 
import re
import rospy
import math
import numpy as np
import copy
from teaching_pkg.msg import *


#ROS init
rospy.init_node('glove_orientation_analysis')
glove_ori_pub = rospy.Publisher('/Glove/right_hand_orientation', rotation_matrix, queue_size=1)


#Global variables
initial_time = rospy.get_time() #Not in use
period = 0.15 #0.2 #Not in use
initial_time_print = rospy.get_time() #Not in use
period_print = 1.0 #Not in use
z_alignment = {'roll': 0, 'pitch': 0, 'yaw': 0}
z_threshold = 0.7 #0.8
stop_count_ori = 0
stop_detection_ori = 3
prev_dir =''
detected_dir =''
R_tracker_glove = np.array([[1.0, 0.0, 0.0],
                           [0.0, 1.0, 0.0],
                           [0.0, 0.0, 1.0]])
transform = False
rotation_limits = {'roll': 0.5, 'pitch': 0.3, 'yaw': 0.3} #0.1 0.05 0.05
rotation = {'roll': 0, 'pitch': 0, 'yaw': 0}
stop_count_rot = 0
stop_detection_rot = 3
prev_rot=''
detected_rot=''
frame_tf = PyKDL.Frame()
frame_tf_old1 = PyKDL.Frame()
frame_tf_old2 = PyKDL.Frame()


def quaternion_rotation_matrix(q0,q1,q2,q3):
    """
    Covert a quaternion into a full three-dimensional rotation matrix.
 
    Input
    :param Q: A 4 element array representing the quaternion (q0,q1,q2,q3) 
 
    Output
    :return: A 3x3 element matrix representing the full 3D rotation matrix. 
             This rotation matrix converts a point in the local reference 
             frame to a point in the global reference frame.
    """
    # Extract the values from Q
    #q0 = Q[0]
    #q1 = Q[1]
    #q2 = Q[2]
    #q3 = Q[3]
     
    # First row of the rotation matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)
     
    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)
     
    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1
     
    # 3x3 rotation matrix
    rot_matrix = np.array([[r00, r01, r02],
                           [r10, r11, r12],
                           [r20, r21, r22]])
                            
    return rot_matrix


def Rx(theta):
  return np.matrix([[ 1, 0           , 0           ],
                   [ 0, math.cos(theta),-math.sin(theta)],
                   [ 0, math.sin(theta), math.cos(theta)]])
  
def Ry(theta):
  return np.matrix([[ math.cos(theta), 0, math.sin(theta)],
                   [ 0           , 1, 0           ],
                   [-math.sin(theta), 0, math.cos(theta)]])
  
def Rz(theta):
  return np.matrix([[ math.cos(theta), -math.sin(theta), 0 ],
                   [ math.sin(theta), math.cos(theta) , 0 ],
                   [ 0           , 0            , 1 ]])


def euler_from_quaternion(x,y,z,w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians


def eul2rot(theta) :
    R = np.array([[np.cos(theta[1])*np.cos(theta[2]), np.sin(theta[0])*np.sin(theta[1])*np.cos(theta[2]) - np.sin(theta[2])*np.cos(theta[0]), np.sin(theta[1])*np.cos(theta[0])*np.cos(theta[2]) + np.sin(theta[0])*np.sin(theta[2])],
                  [np.sin(theta[2])*np.cos(theta[1]), np.sin(theta[0])*np.sin(theta[1])*np.sin(theta[2]) + np.cos(theta[0])*np.cos(theta[2]), np.sin(theta[1])*np.sin(theta[2])*np.cos(theta[0]) - np.sin(theta[0])*np.cos(theta[2])],
                  [-np.sin(theta[1]), np.sin(theta[0])*np.cos(theta[1]), np.cos(theta[0])*np.cos(theta[1])]])
    return R


def transform_frame(R_original, R_transform_prev):
    #R_tracker_hand = R_tracker_glove * R_glove_hand
    R_result = np.matmul(R_transform_prev, R_original)
    return R_result


def z_alignment_analysis(pitch_z, roll_z, yaw_z):
	global z_alignment
	global z_threshold
	global prev_dir
	global stop_count_ori
	global stop_detection_ori
	global detected_dir

	#print("Z analysis")

	z_alignment['pitch'] = pitch_z
	z_alignment['roll'] = roll_z
	z_alignment['yaw'] = yaw_z

	z_max = 0
        for axis in z_alignment:
                if abs(z_alignment[axis]) > z_threshold:
                    if abs(z_alignment[axis]) > abs(z_max):
                        z_max = z_alignment[axis]
                        max_axis = axis
                        max_positive = z_max > 0
        if z_max != 0:
                stop_count_ori = 0
                if max_positive:
                    detected_dir = '+' + str(max_axis) + ' orientation'
                else:
                    detected_dir = '-' + str(max_axis) + ' orientation'
                if detected_dir != prev_dir:
                    #print(detected_dir)
                    prev_dir = detected_dir
        else:
                stop_count_ori += 1
                
        if stop_count_ori == stop_detection_ori:
                #print('Random orientation')
                prev_dir = 'Random orientation'


def rotation_analysis():
	global frame_tf
	global frame_tf_old1
	global frame_tf_old2
	global rotation
	global rotation_limits
	global prev_rot
	global stop_count_rot
	global stop_detection_rot
	global detected_rot

	#print("Rot analysis")

	relative_tf = frame_tf_old1.Inverse() * frame_tf 
	relative_quat = relative_tf.M.GetQuaternion()
	rotation['pitch'], rotation['yaw'] , rotation['roll'] = euler_from_quaternion(float(relative_quat[0]), float(relative_quat[1]), float(relative_quat[2]), float(relative_quat[3]))
	#print("Roll:" + str(yaw) + ", Pitch:" + str(roll) + ", Yaw:" + str(pitch))
	
	max_axis_value = 0
	for axis in rotation:
		if abs(rotation[axis]) > rotation_limits[axis]:
			if abs(rotation[axis]) > abs(max_axis_value):
				max_axis_value = rotation[axis]
				max_axis = axis
				max_positive = max_axis_value > 0
	if max_axis_value != 0:
		stop_count_rot = 0
		if max_positive:
			detected_rot = '+' + str(max_axis) + 'Rotation'
		else:
			detected_rot = '-' + str(max_axis) + 'Rotation'
		if detected_rot != prev_rot:
			#print(detected_rot)
			prev_rot = detected_rot
	else:
		stop_count_rot += 1
	
	if stop_count_rot == stop_detection_rot:
		#print('No rotation')
		prev_rot = 'No rotation'

	frame_tf_old2 = frame_tf_old1
	frame_tf_old1 = frame_tf


def analyze_orientation(_roll, _pitch, _yaw):
	global transform
	global frame_tf
	global R_tracker_glove

	R_matrix = eul2rot([_roll, _pitch, _yaw])
        #R_matrix = quaternion_rotation_matrix(float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3]))
        #### Correction of the matrix arrangement (P R Y) ####
        #R_matrix_corrected = np.array([[R_matrix[2][0], R_matrix[0][0], R_matrix[1][0]],
        #                    [R_matrix[2][2], R_matrix[0][2], R_matrix[1][2]],
        #                    [R_matrix[2][1], R_matrix[0][1], R_matrix[1][1]]])
	#print(R_matrix_corrected)
        if transform:
            R_matrix = transform_frame(R_transform_prev = R_tracker_glove, R_original = R_matrix_corrected)
        print(R_matrix)

        #Publish the rotation matrix of the hand
        ###
        R_matrix_msg = rotation_matrix()

        R_matrix_msg.pitch.x = R_matrix_corrected[0][0]
        R_matrix_msg.pitch.y = R_matrix_corrected[0][1]
        R_matrix_msg.pitch.z = R_matrix_corrected[0][2]
        R_matrix_msg.roll.x = R_matrix_corrected[1][0]
        R_matrix_msg.roll.y = R_matrix_corrected[1][1]
        R_matrix_msg.roll.z = R_matrix_corrected[1][2]
        R_matrix_msg.yaw.x = R_matrix_corrected[2][0]
        R_matrix_msg.yaw.y = R_matrix_corrected[2][1]
        R_matrix_msg.yaw.z = R_matrix_corrected[2][2]

        glove_ori_pub.publish(R_matrix_msg)
        ###

        #Calculates the hand alignment with the Z axis
	z_alignment_analysis(R_matrix_corrected[0][2], R_matrix_corrected[1][2], R_matrix_corrected[2][2])
	#Calculates the hand rotation
	rotation_analysis()


#Subscribers
def right_tracker_cb(info):
    global deg_rot
    global x_tracker, y_tracker, z_tracker
    global new_position, new_orientation
    orientation_VIVE = info.data
    pitch_tracker = float((orientation_VIVE.split(';'))[5]) #pitch?? x
    roll_tracker = float((orientation_VIVE.split(';'))[3]) #roll?? y
    yaw_tracker =  float((orientation_VIVE.split(';'))[4]) #yaw?? z
    analyze_orientation(roll_tracker, pitch_tracker, yaw_tracker)

subsTrackerR = rospy.Subscriber('Tracker/pose_string/right', String, right_tracker_cb)


def R_glove_tracker_cb(info):
    global R_tracker_glove
    global transform
    print(info.pitch.x)
    R_tracker_glove[0][0] = float(info.pitch.x)
    R_tracker_glove[0][1] = info.pitch.y
    R_tracker_glove[0][2] = info.pitch.z
    R_tracker_glove[1][0] = info.roll.x
    R_tracker_glove[1][1] = info.roll.y
    R_tracker_glove[1][2] = info.roll.z
    R_tracker_glove[2][0] = info.yaw.x
    R_tracker_glove[2][1] = info.yaw.y
    R_tracker_glove[2][2] = info.yaw.z
    print(str(R_tracker_glove))
    transform = True

subsGloveR = rospy.Subscriber('/Glove/R_glove_tracker', rotation_matrix, R_glove_tracker_cb)
		

print('Glove orientation node ready')
 # Keep this process running until Enter is pressed
print ("Press Enter to quit...")
try:
	sys.stdin.readline()
except KeyboardInterrupt:
	print("Glove orientation node finalized")
	pass
