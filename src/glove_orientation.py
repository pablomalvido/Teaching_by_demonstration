#!/usr/bin/env python
from std_msgs.msg import String
import tf 
import PyKDL 
import re
import rospy
import math
import numpy as np
import copy
import os
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from teaching_pkg.msg import *
from std_msgs.msg import String
from os.path import abspath


#ROS init
rospy.init_node('glove_orientation_analysis')
glove_ori_pub = rospy.Publisher('/Glove/right_hand_orientation', rotation_matrix, queue_size=1)
glove_Zalignment_pub = rospy.Publisher('/Glove/right_hand_Zalignment', String, queue_size=1)
coord_pub = rospy.Publisher('/Teaching/record_coordinates', Float32, queue_size=1)


#Global variables
record_demo = False #Restart
write_times = False
demo_time = 0
path_rot = os.path.join(os.path.dirname(__file__), '../files/zz_other/example/hand_orientation_Z.txt')
path_ori = os.path.join(os.path.dirname(__file__), '../files/zz_other/example/hand_rotation.txt')
path_general = os.path.join(os.path.dirname(__file__), '../files/zz_other/example/demonstration.txt')
path_general_complete = os.path.join(os.path.dirname(__file__), '../files/zz_other/example/demonstration_complete.txt')
f_rot = open(path_rot, "a")
f_ori = open(path_ori, "a")
f_general = open(path_general, "a")
f_general_complete = open(path_general_complete, "a")


contact_info = "NC"

first_ori = True #Restart
first_rot = True
same_ori_count = 0
long_ori_detected = 8

print_rot = True
print_ori = True
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
rotation_limits = {'roll': 0.5, 'pitch': 0.3, 'yaw': 0.25} #0.1 0.05 0.05
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


def transform_frame(R_original, R_transform_prev):
    #R_tracker_hand = R_tracker_glove * R_glove_hand
    R_result = np.matmul(R_transform_prev, R_original)
    return R_result


def z_alignment_analysis(roll_z, pitch_z, yaw_z):
	global z_alignment
	global z_threshold
	global prev_dir
	global stop_count_ori
	global stop_detection_ori
	global detected_dir
	global record_demo
	global first_ori
	global same_ori_count
	global long_ori_detected

	if print_ori:

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
			    same_ori_count = 0
		            #print(detected_dir)
			    #publish the orientation even if it is not recording so the other nodes can have this info from the start of the demo
			    Zalignment_message = String()
    			    Zalignment_message.data = detected_dir
			    glove_Zalignment_pub.publish(Zalignment_message)
			    if record_demo:
				if first_ori:
					write_demo("orientation", detected_dir)
					first_ori = False
				else:
					write_demo("orientation", "\n" + detected_dir)
		            prev_dir = detected_dir
			else:
			    same_ori_count += 1
			    if ((same_ori_count == long_ori_detected) and record_demo):
				write_demo("orientation", " long")
		else:
		        stop_count_ori += 1
		        
		if stop_count_ori == stop_detection_ori:
		        #print('Random orientation')
			#publish the orientation even if it is not recording so the other nodes can have this info from the start of the demo
			Zalignment_message = String()
    			Zalignment_message.data = "Random orientation"
			glove_Zalignment_pub.publish(Zalignment_message)
			if record_demo:
				if first_ori:
					write_demo("orientation", "Random orientation")
					first_ori = False
				else:
					write_demo("orientation", "\nRandom orientation")
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
	global record_demo

	if print_rot:

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
				if record_demo:
					write_demo("rotation", detected_rot)
					write_demo("general", "rotation")
				prev_rot = detected_rot
		else:
			stop_count_rot += 1
		
		if stop_count_rot == stop_detection_rot:
			#print('No rotation')
			if record_demo:
				write_demo("rotation", "No")
			prev_rot = 'No rotation'

		frame_tf_old2 = frame_tf_old1
		frame_tf_old1 = frame_tf
		

def analyze_orientation(ori_string):
	global transform
	global frame_tf
	global R_tracker_glove

        right_glove_ori = re.search('{(.*)}', ori_string).group(1)
        replace_elements = (" ","{","}","=","X","Y","Z","W","\n","\r","\t")
        for elem in replace_elements:
            right_glove_ori = right_glove_ori.replace(elem, "")
        quat = right_glove_ori.split(",")

	frame_tf = PyKDL.Frame()
	frame_tf.p = PyKDL.Vector(0,0,0)
	frame_tf.M = PyKDL.Rotation.Quaternion(float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3]))

	"""
        #R_matrix = quaternion_rotation_matrix(float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3]))
	R_matrix = quaternion_rotation_matrix(float(quat[1]), float(quat[2]), float(quat[3]), float(quat[0]))
        #### Correction of the matrix arrangement (P R Y) ####
	#R_matrix_corrected = R_matrix
        R_matrix_corrected = np.array([[R_matrix[2][2], R_matrix[2][0], R_matrix[2][1]],
                            [R_matrix[0][2], R_matrix[0][0], R_matrix[0][1]],
                            [R_matrix[1][2], R_matrix[1][0], R_matrix[1][1]]])
	#print(R_matrix_corrected)
	"""

	"""
	R_matrix_corrected = quaternion_rotation_matrix(float(quat[3]), float(quat[0]), float(quat[1]), float(quat[2]))
	Rx_90_neg = np.array([[1, 0, 0],
                            [0, 0, 1],
                            [0, -1, 0]])
	R_matrix_corrected = transform_frame(R_transform_prev = R_matrix_corrected, R_original = Rx_90_neg)
	Rz_90 = np.array([[0, -1, 0],
                            [1, 0, 0],
                            [0, 0, 1]])
	R_matrix_corrected = transform_frame(R_transform_prev = R_matrix_corrected, R_original = Rz_90)
	"""

	R_matrix = quaternion_rotation_matrix(float(quat[3]), float(quat[0]), float(quat[1]), float(quat[2]))
	R_matrix_corrected = np.array([[-R_matrix[0][2], -R_matrix[0][0], R_matrix[0][1]],
                            [-R_matrix[1][2], -R_matrix[1][0], R_matrix[1][1]],
                            [-R_matrix[2][2], -R_matrix[2][0], R_matrix[2][1]]])
	"""

	R_pov = np.array([[0, -1, 0],
                            [0, 0, 1],
                            [-1, 0, 0]])
	R_matrix_corrected = transform_frame(R_transform_prev = R_pov, R_original = R_matrix_corrected)
	"""
	

        if transform:
            R_matrix_corrected = transform_frame(R_transform_prev = R_tracker_glove, R_original = R_matrix_corrected)
	    #PROBLEM: opposite yaw rotation
	    #First option: change xy and yx signs
	    """
	    sign_xy =  R_matrix_corrected[0][1]/abs(R_matrix_corrected[0][1])
	    sign_yx =  R_matrix_corrected[1][0]/abs(R_matrix_corrected[1][0])
	    R_matrix_corrected[0][1] *= sign_yx
	    R_matrix_corrected[1][0] *= sign_xy
	    """
	    #Second option: Transpose
	    """
	    R_matrix_corrected_copy = R_matrix_corrected.copy()
	    R_matrix_corrected[0][1] = R_matrix_corrected_copy[1][0]
	    R_matrix_corrected[0][2] = R_matrix_corrected_copy[2][0]
	    R_matrix_corrected[1][0] = R_matrix_corrected_copy[0][1]
	    R_matrix_corrected[1][2] = R_matrix_corrected_copy[2][1]
	    R_matrix_corrected[2][0] = R_matrix_corrected_copy[0][2]
	    R_matrix_corrected[2][1] = R_matrix_corrected_copy[1][2]
	    """
	    #Third option:
	    """
	    yaw_correct_rot = -2 * math.atan2(R_matrix_corrected[1][0], R_matrix_corrected[0][0])
	    R_yaw_correct = np.array([[math.cos(yaw_correct_rot), -math.sin(yaw_correct_rot), 0],
                            [math.sin(yaw_correct_rot), math.cos(yaw_correct_rot), 0],
                            [0, 0, 1]])
	    R_matrix_corrected = transform_frame(R_transform_prev = R_matrix_corrected, R_original = R_yaw_correct)
	    """
	    
	    #Other
	    """
	    R_matrix_corrected_copy = R_matrix_corrected.copy()
	    R_matrix_corrected[0][0] = R_matrix_corrected_copy[0][2]
	    R_matrix_corrected[0][1] = R_matrix_corrected_copy[0][0]
	    R_matrix_corrected[0][2] = R_matrix_corrected_copy[0][1]
	    R_matrix_corrected[1][0] = R_matrix_corrected_copy[1][2]
	    R_matrix_corrected[1][1] = R_matrix_corrected_copy[1][0]
	    R_matrix_corrected[1][2] = R_matrix_corrected_copy[1][1]
	    R_matrix_corrected[2][0] = R_matrix_corrected_copy[2][2]
	    R_matrix_corrected[2][1] = R_matrix_corrected_copy[2][0]
	    R_matrix_corrected[2][2] = R_matrix_corrected_copy[2][1]
	    """
        print(R_matrix_corrected)

        #Publish the rotation matrix of the hand
        ###
        R_matrix_msg = rotation_matrix()

        R_matrix_msg.roll.x = R_matrix_corrected[0][0]
        R_matrix_msg.roll.y = R_matrix_corrected[1][0]
        R_matrix_msg.roll.z = R_matrix_corrected[2][0]
        R_matrix_msg.pitch.x = R_matrix_corrected[0][1]
        R_matrix_msg.pitch.y = R_matrix_corrected[1][1]
        R_matrix_msg.pitch.z = R_matrix_corrected[2][1]
        R_matrix_msg.yaw.x = R_matrix_corrected[0][2]
        R_matrix_msg.yaw.y = R_matrix_corrected[1][2]
        R_matrix_msg.yaw.z = R_matrix_corrected[2][2]

        glove_ori_pub.publish(R_matrix_msg)
        ###

        #Calculates the hand alignment with the Z axis
	z_alignment_analysis(R_matrix_corrected[0][2], R_matrix_corrected[1][2], R_matrix_corrected[2][2])
	#Calculates the hand rotation
	rotation_analysis()


def write_demo(type_message, message):
    global f_ori
    global f_rot
    global f_general
    global path_general
    global f_general_complete
    global path_general_complete
    global contact_info
    global prev_dir
    global write_times
    global demo_time
    global first_ori
    global first_rot

    current_time = 0
    if write_times:
	current_time = rospy.get_time() - demo_time
	if type_message != "orientation":
		message = str(current_time) + ":" + message
	coord_message = Float32()
    	coord_message.data = current_time
	coord_pub.publish(coord_message) #Signal to write the coordinates.txt

    if type_message == "rotation":
	first_rot = False
    	f_rot.write(message + "\n")
	if message[-2:] != "No":
		f_general_complete = open(path_general_complete, "a")
		f_general_complete.write(message + "," + contact_info + "\n")
		f_general_complete.close()
	else:
		f_general_complete = open(path_general_complete, "a")
		if write_times:
			f_general_complete.write(str(current_time) + ":Stop rot at " + str(prev_dir) + "," + contact_info + "\n")
		else:
			f_general_complete.write("Stop rot at " + str(prev_dir) + "," + contact_info + "\n")
		f_general_complete.close()
    if type_message == "orientation":
	if write_times:
		if message != " long":
			message = message.replace("\n","")
			if first_ori:
				message = str(current_time) + ":" + message
			else:
				message = "\n" + str(current_time) + ":" + message			
	f_ori.write(message)
    if type_message == "general":
        f_general = open(path_general, "a")
	f_general.write(message + "," + contact_info + "\n")
	f_general.close()


#Subscribers
def glove_ori_cb(info):
    #print(info.data)
    analyze_orientation(info.data)

subsGloveR = rospy.Subscriber('Glove/orientation_string/right', String, glove_ori_cb)


def R_glove_tracker_cb(info):
    global R_tracker_glove
    global transform
    #print(info.pitch.x)
    R_tracker_glove[0][0] = info.roll.x
    R_tracker_glove[0][1] = info.pitch.x
    R_tracker_glove[0][2] = info.yaw.x
    R_tracker_glove[1][0] = info.roll.y
    R_tracker_glove[1][1] = info.pitch.y
    R_tracker_glove[1][2] = info.yaw.y
    R_tracker_glove[2][0] = info.roll.z
    R_tracker_glove[2][1] = info.pitch.z
    R_tracker_glove[2][2] = info.yaw.z
    #print(str(R_tracker_glove))
    transform = True

subsGloveR = rospy.Subscriber('/Glove/R_glove_tracker', rotation_matrix, R_glove_tracker_cb) #Change topic and msg for the correct values
		

def start_cb(info):
    global record_demo
    global first_ori
    global f_ori
    global f_rot
    global path_ori
    global path_rot
    global path_general
    global path_general_complete
    global detected_dir
    global write_times
    global demo_time
    global same_ori_count
    global first_rot
    path = info.path
    write_times = info.times
    demo_time = rospy.get_time()
    path_ori = path + '/hand_orientation_Z.txt'
    path_rot = path + '/hand_rotation.txt'
    path_general = path + '/demonstration.txt'
    path_general_complete = path + '/demonstration_complete.txt'
    record_demo = True
    first_ori = True
    first_rot = True
    f_ori = open(path_ori, "w")
    f_rot = open(path_rot, "w")
    if write_times: #Only in sequences start with a No
    	write_demo("rotation", "No")
    same_ori_count = 0
    write_demo("orientation", detected_dir)
    first_ori = False

subsStart = rospy.Subscriber('/Teaching/start_demo', start_demo_msg, start_cb)


def stop_cb(info):
    global record_demo
    global f_ori
    global f_rot
    global path_ori
    global path_rot
    global first_rot
    if first_rot:
	write_demo("rotation", "No")
    record_demo = False
    f_ori.close()
    f_rot.close()

subsStop = rospy.Subscriber('/Teaching/stop_demo', Bool, stop_cb)


def contact_cb(info):
    global contact_info
    if info.data:
    	contact_info = "C"
    else:
	contact_info = "NC"

subsContact = rospy.Subscriber('/Glove/contact', Bool, contact_cb)



print('Glove orientation node ready')
 # Keep this process running until Enter is pressed
print ("Press Enter to quit...")
try:
	sys.stdin.readline()
except KeyboardInterrupt:
	print("Glove orientation node finalized")
	pass
