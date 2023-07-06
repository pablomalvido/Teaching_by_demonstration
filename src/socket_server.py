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

#ROS init
rospy.init_node('socket_server')
tracker_pub = rospy.Publisher('Tracker/pose_string/right', String, queue_size=1)
ori_glove_pub = rospy.Publisher('Glove/orientation_string/right', String, queue_size=1)
fingers_glove_pub = rospy.Publisher('Glove/fingers_string/right', String, queue_size=1)
tracker_info = String()
orientation_info = String()
fingers_info = String()
tracker_bool = False
orientation_bool = False
fingers_bool = False
orientation_time1 = 0
fingers_time1 = 0
glove_ms = 150
calibration_success = False
calibration_op = 0

#Number of clients
ok_option = False
print("How many devices do you want to connect with? ")
print("[0] One device")
print("[1] Glove and Tracker")
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

#Socket
my_socket = socket.socket()
my_socket.bind(('10.0.0.10', 54000))
my_socket.listen(5)

connection1, addr1 = my_socket.accept()
print ("Connection 1 established")
print (addr1)

#
if calibration_op == 1:
	connection2, addr2 = my_socket.accept()
	print ("Connection 2 established")
	print (addr2)
#

#connection3, addr3 = mi_socket.accept()
#print ("Connection 3 established")
#print (addr3)

while True:
    petition1 = connection1.recv(1024)
    #
    if calibration_op == 1:
    	petition2 = connection2.recv(1024)
    #
    #petition3 = connection3.recv(1024)

    if (calibration_op == 0 and len(petition1) == 0) or (calibration_op == 1 and (len(petition1) == 0 or len(petition2) == 0)):
    #
    #if len(petition1) == 0 or len(petition2) == 0:
    #
        #break
	print("Error")

    print(petition1)
    #
    if calibration_op == 1:
    	print(petition2)
    #
    #print(petition3)

    ########### for loop ###########
    text1 = petition1.split(":")
    #
    if calibration_op == 1:
    	text2 = petition2.split(":")
    #
    #text3 = petition3.split(":")

    #Check the message sent by each client
    if text1[0] == 'tracker_1':
	tracker_info.data = text1[1]
	tracker_bool = True

    elif text1[0] == 'RightOrientation':
	orientation_time2 = long(text1[1])
	if (orientation_time2 - orientation_time1) >= glove_ms:
		orientation_info.data = text1[2]
		orientation_bool = True
		orientation_time1 = orientation_time2

    elif text1[0] == 'RightFingers':
	fingers_time2 = long(text1[1])
	if (fingers_time2 - fingers_time1) >= glove_ms:
		fingers_info.data = text1[2]
		fingers_bool = True
		fingers_time1 = fingers_time2
    #
    #"""
    if calibration_op == 1:
	    if text2[0] == 'tracker_1':
		tracker_info.data = text2[1]
		tracker_bool = True

	    elif text2[0] == 'RightOrientation':
		orientation_time2 = long(text2[1])
		if (orientation_time2 - orientation_time1) >= glove_ms:
			orientation_info.data = text2[2]
			orientation_bool = True
			orientation_time1 = orientation_time2

	    elif text2[0] == 'RightFingers':
		fingers_time2 = long(text2[1])
		if (fingers_time2 - fingers_time1) >= glove_ms:
			fingers_info.data = text2[2]
			fingers_bool = True
			fingers_time1 = fingers_time2
    #"""
    #
    ################################

    #Publishers
    if tracker_bool:
	tracker_pub.publish(tracker_info)
	tracker_bool = False

    if orientation_bool:
	ori_glove_pub.publish(orientation_info)
	orientation_bool = False

    if fingers_bool:
	fingers_glove_pub.publish(fingers_info)
	fingers_bool = False


print('Finish')
connection1.close()
#
#
if calibration_op == 1:
	connection2.close()
#
