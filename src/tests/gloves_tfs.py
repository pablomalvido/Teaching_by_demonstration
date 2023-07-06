#!/usr/bin/env python

import socket
from std_msgs.msg import String
import tf 
import PyKDL 
import re
import rospy
from geometry_msgs.msg import Pose

rospy.init_node('hands_tf_node_glove', anonymous=True)
orientation_time1 = 0
glove_ms = 150

broad = tf.TransformBroadcaster()

def broadcastTransform(br, frame, frame_id, parent_frame, time):
	#Function for broadcasting transforms 
	time2 = rospy.get_rostime()
        br.sendTransform((frame.p.x(), frame.p.y(), frame.p.z()), 
            frame.M.GetQuaternion(), 
            time2, 
            frame_id, 
            parent_frame) 

mi_socket = socket.socket()
#mi_socket.bind(('192.168.43.201', 54000))
mi_socket.bind(('10.0.0.5', 54000))
mi_socket.listen(5)

conexion1, addr1 = mi_socket.accept()
print ("Nueva conexion establecida 1")
print (addr1)

#conexion2, addr2 = mi_socket.accept()
#print ("Nueva conexion establecida 2")
#print (addr2)

while True:
    peticion1 = conexion1.recv(1024)
    print (peticion1)
    #peticion2 = conexion2.recv(1024)
    #print (peticion2)

    if len(peticion1) == 0:
        break

    text1 = peticion1.split(":")
    #text2 = peticion2.split(":")

    if text1[0] == 'RightOrientation':
	orientation_time2 = long(text1[1])
	if (orientation_time2 - orientation_time1) >= glove_ms:
		orientation_time1 = orientation_time2

		right_glove_ori = re.search('{(.*)}', text1[2]).group(1)
		replace_elements = (" ","{","}","=","X","Y","Z","W","\n","\r","\t")
		for elem in replace_elements:
			right_glove_ori = right_glove_ori.replace(elem, "")
		quat = right_glove_ori.split(",")
		frame_tf = PyKDL.Frame()
		frame_tf.p = PyKDL.Vector(0,1,0)
		frame_tf.M = PyKDL.Rotation.Quaternion(float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3]))
		print(""+quat[0]+","+quat[1]+","+quat[2]+","+quat[3])
		tf_name = "right_hand_glove"
		parent_tf = "world"
		current_time = rospy.get_rostime()
		broadcastTransform(broad, frame_tf, tf_name, parent_tf, current_time)

    """
    if text2[0] == 'LeftOrientation':
		left_glove_ori = re.search('{(.*)}', text2[1]).group(1)
		replace_elements = (" ","{","}","=","X","Y","Z","W","\n","\r","\t")
		for elem in replace_elements:
			left_glove_ori = left_glove_ori.replace(elem, "")
		quat = left_glove_ori.split(",")
		frame_tf = PyKDL.Frame()
		frame_tf.p = PyKDL.Vector(0,-1,0)
		frame_tf.M = PyKDL.Rotation.Quaternion(float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3]))
		tf_name = "left_hand_glove"
		parent_tf = "world"
		current_time = rospy.get_rostime()
		#broadcastTransform(broad, frame_tf, tf_name, parent_tf, current_time)

		left_pose = leap_hand()
		left_pose.detected = True
		left_pose.pose_hand.position.x = 0
		left_pose.pose_hand.position.y = 0
		left_pose.pose_hand.position.z = 0
		left_pose.pose_hand.orientation.x = float(quat[0])
		left_pose.pose_hand.orientation.y = float(quat[1])
		left_pose.pose_hand.orientation.z = float(quat[2])
		left_pose.pose_hand.orientation.w = float(quat[3])
		pub_oriLeft.publish(left_pose)
    """

print('Finish')
conexion.close()
