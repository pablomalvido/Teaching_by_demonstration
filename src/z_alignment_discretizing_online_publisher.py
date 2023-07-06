#!/usr/bin/env python
import socket
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
initial_time = rospy.get_time()
period = 0.15 #0.2
initial_time_print = rospy.get_time()
period_print = 1.0 #Not in use
z_alignment = {'roll': 0, 'pitch': 0, 'yaw': 0}
z_threshold = 0.8
stop_count = 0
stop_detection = 3
prev_dir =''
detected_dir =''
frame_tf = PyKDL.Frame()
frame_tf_old1 = PyKDL.Frame()
frame_tf_old2 = PyKDL.Frame()
R_tracker_glove = np.array([[1, 0, 0],
                           [0, 1, 0],
                           [0, 0, 1]])
transform = False


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


def transform_frame(R_transform_prev = None, R_original, R_transform_post = None):
    #R_tracker_hand = R_tracker_glove * R_glove_hand
    if R_transform_prev:
        R_result = np.matmul(R_transform_prev, R_original)
    if R_transform_post:
        R_result = np.matmul(R_original, R_transform_post)
    return R_result


#Subscribers
def R_glove_tracker_cb(info):
    global R_tracker_glove
    global transform
    R_tracker_glove[0][0] = info.pitch.x
    R_tracker_glove[0][1] = info.pitch.y
    R_tracker_glove[0][2] = info.pitch.z
    R_tracker_glove[1][0] = info.roll.x
    R_tracker_glove[1][1] = info.roll.y
    R_tracker_glove[1][2] = info.roll.z
    R_tracker_glove[2][0] = info.yaw.x
    R_tracker_glove[2][1] = info.yaw.y
    R_tracker_glove[2][2] = info.yaw.z
    transform = True

subsGloveR = rospy.Subscriber('/Glove/R_glove_tracker', rotation_matrix, R_glove_tracker_cb) #Change topic and msg for the correct values


#Socket
mi_socket = socket.socket()
mi_socket.bind(('192.168.43.201', 54000))
mi_socket.listen(5)

conexion1, addr1 = mi_socket.accept()
print ("Nueva conexion establecida 1")
print (addr1)

conexion2, addr2 = mi_socket.accept()
print ("Nueva conexion establecida 2")
print (addr2)

while True:
    peticion1 = conexion1.recv(1024)
    #print (peticion1)
    peticion2 = conexion2.recv(1024)
    #print (peticion2)

    if len(peticion1) == 0:
        break

    text1 = peticion1.split(":")
    text2 = peticion2.split(":")
    textR = ''
    textR_bool = False
    if text1[0] == 'RightOrientation':
        textR = text1[1]
        textR_bool = True
    if text2[0] == 'RightOrientation':
        textR = text2[1]
        textR_bool = True

    if textR_bool:
        if ((rospy.get_time()-initial_time) > period):
            initial_time = rospy.get_time()
            right_glove_ori = re.search('{(.*)}', textR).group(1)
            replace_elements = (" ","{","}","=","X","Y","Z","W","\n","\r","\t")
            for elem in replace_elements:
                right_glove_ori = right_glove_ori.replace(elem, "")
            quat = right_glove_ori.split(",")

            R_matrix = quaternion_rotation_matrix(float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3]))
            #### Correction of the matrix arrangement (P R Y) ####
            R_matrix_corrected = np.matrix([[R_matrix[2][0], R_matrix[0][0], R_matrix[1][0]],
                                [R_matrix[2][1], R_matrix[0][2], R_matrix[1][2]],
                                [R_matrix[2][2], R_matrix[0][1], R_matrix[1][1]]])
            if transform:
                R_matrix_corrected = transform_frame(R_transform_prev = R_tracker_glove, R_original = R_matrix_corrected)
            #print(R_matrix_corrected)

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

            glove_ori_pub.publish(R_matrix)
            ###

            #Calculates the hand alignment with the Z axis
            z_alignment['pitch'] = R_matrix[0][2]
            z_alignment['roll'] = R_matrix_corrected[1][2]
            z_alignment['yaw'] = R_matrix[2][2]

            z_max = 0
            for axis in z_alignment:
                if abs(z_alignment[axis]) > z_threshold:
                    if abs(z_alignment[axis]) > abs(z_max):
                        z_max = z_alignment[axis]
                        max_axis = axis
                        max_positive = z_max > 0
            if z_max != 0:
                stop_count = 0
                if max_positive:
                    detected_dir = '+' + str(max_axis) + ' orientation'
                else:
                    detected_dir = '-' + str(max_axis) + ' orientation'
                if detected_dir != prev_dir:
                    print(detected_dir)
                    prev_dir = detected_dir
            else:
                stop_count += 1
                
            if stop_count == stop_detection:
                print('Random orientation')
                prev_dir = 'Random orientation'
		

print('Finish')
conexion.close()
