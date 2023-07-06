#!/usr/bin/env python
from std_msgs.msg import String
import tf 
import PyKDL 
import re
import rospy
import sys
import os
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_msgs.msg import Float32
from teaching_pkg.msg import *
from os.path import abspath


#ROS init
rospy.init_node('glove_fingers_analysis', anonymous=True)
contact_pub = rospy.Publisher('/Glove/contact', Bool, queue_size=1)
coord_pub = rospy.Publisher('/Teaching/record_coordinates', Float32, queue_size=1)

record_demo = False
write_times = False
demo_time = 0
path_bending = os.path.join(os.path.dirname(__file__), '../files/zz_other/example/finger_bending.txt')
path_pressure = os.path.join(os.path.dirname(__file__), '../files/zz_other/example/finger_pressure.txt')
path_fingers = os.path.join(os.path.dirname(__file__), '../files/zz_other/example/fingers.txt')
path_fingers2 = os.path.join(os.path.dirname(__file__), '../files/zz_other/example/fingers2.txt')
path_general = os.path.join(os.path.dirname(__file__), '../files/zz_other/example/demonstration.txt')
path_general_complete = os.path.join(os.path.dirname(__file__), '../files/zz_other/example/demonstration_complete.txt')
f_bending = open(path_bending, "a")
f_pressure = open(path_pressure, "a")
f_fingers = open(path_fingers, "a")
f_fingers2 = open(path_fingers2, "a")
f_general = open(path_general, "a")
f_general_complete = open(path_general_complete, "a")

change_bending = False
change_pressure = False
first_bending = True
first_pressure = True
publish_opening = True
publish_closing = True
publish_contact = True
publish_no_contact = True
initial_contact = True
cold_down = 0
orientation_hand = "Random orientation"

#original_limits = {'thumb': [500,110,-300], 'index': [2400,1200,600], 'middle': [1600,650,280], 'ring': [1000,500,0], 'pinky': [1500,800,150]} #Old gloves
#original_limits = {'thumb': [2000,1350,1000], 'index': [2300,1400,1000], 'middle': [2200,1400,1050], 'ring': [750,300,-300], 'pinky': [2400,1400,-300]}
original_limits = {'thumb': [2000,1300,-300], 'index': [2300,1200,-300], 'middle': [2200,1250,-300], 'ring': [750,300,-300], 'pinky': [2400,1400,-300]}
limits = original_limits
original_limits_pressure = {'thumb': [1,12000], 'index': [1,8000], 'middle': [1,8000], 'ring': [1,15000], 'pinky': [1,8000]}
limits_pressure = original_limits_pressure
fingers = {'thumb': {'bending':0, 'old_bending':0, 'status':0, 'presure':0, 'status_pressure':0}, 'index': {'bending':0, 'old_bending':0, 'status':0, 'presure':0, 'status_pressure':0}, 'middle': {'bending':0, 'old_bending':0, 'status':0, 'presure':0, 'status_pressure':0}, 'ring': {'bending':0, 'old_bending':0, 'status':0, 'presure':0, 'status_pressure':0}, 'pinky': {'bending':0, 'old_bending':0, 'status':0, 'presure':0, 'status_pressure':0}}


def analyze_fingers(fingers_string):
    global original_limits
    global limits
    global fingers
    global limits_pressure
    global original_limits_pressure
    global record_demo
    global change_bending
    global change_pressure
    global publish_opening
    global publish_closing
    global publish_contact
    global publish_no_contact
    global initial_contact
    global cold_down

    opening = False
    closing = False
    contact = False
    print_bending = True
    print_pressure = True

    right_glove_fingers = fingers_string.replace(" ", "")
    joint_values = right_glove_fingers.split(",")
    fingers['thumb']['bending'] = float(joint_values[0])
    fingers['index']['bending'] = float(joint_values[2])
    fingers['middle']['bending'] = float(joint_values[4])
    fingers['ring']['bending'] = float(joint_values[6])
    fingers['pinky']['bending'] = float(joint_values[8])
    fingers['thumb']['pressure'] = float(joint_values[1])
    fingers['index']['pressure'] = float(joint_values[3])
    fingers['middle']['pressure'] = float(joint_values[5])
    fingers['ring']['pressure'] = float(joint_values[7])
    fingers['pinky']['pressure'] = float(joint_values[9])

    change_bending = False
    change_pressure = False

    #if True:
    #  finger_name = 'pinky'
    #  print(fingers[finger_name]['bending'])	
    for finger_name in fingers:
	#print(fingers[finger_name]['bending'])
      #Calculates bending changes in each finger and writes in the demo files
      if print_bending:
    	if fingers[finger_name]['bending'] > limits[finger_name][0]:
	  if fingers[finger_name]['old_bending'] <= limits[finger_name][0]:
		opening = True
		print(finger_name + str(fingers[finger_name]['status']) + ' --> 0')
		if record_demo:
			write_demo("bending", finger_name + str(fingers[finger_name]['status']) + ' --> 0')
		fingers[finger_name]['status'] = 0
		limits = original_limits
		limits[finger_name][0] -= 100

	elif fingers[finger_name]['bending'] > limits[finger_name][1]:
	  if fingers[finger_name]['old_bending'] <= limits[finger_name][1]:
		opening = True
		print(finger_name + str(fingers[finger_name]['status']) + ' --> 1')
		if record_demo:
			write_demo("bending", finger_name + str(fingers[finger_name]['status']) + ' --> 1')
		fingers[finger_name]['status'] = 1
		limits = original_limits
		limits[finger_name][1] -= 100
	  elif fingers[finger_name]['old_bending'] > limits[finger_name][0]:
		closing = True
		print(finger_name + str(fingers[finger_name]['status']) + ' --> 1')
		if record_demo:
			write_demo("bending", finger_name + str(fingers[finger_name]['status']) + ' --> 1')
		fingers[finger_name]['status'] = 1
		limits = original_limits
		limits[finger_name][0] += 100

	elif fingers[finger_name]['bending'] > limits[finger_name][2]:
	  if fingers[finger_name]['old_bending'] <= limits[finger_name][2]:
		opening = True
		print(finger_name + str(fingers[finger_name]['status']) + ' --> 2')
		if record_demo:
			write_demo("bending", finger_name + str(fingers[finger_name]['status']) + ' --> 2')
		fingers[finger_name]['status'] = 2
		limits = original_limits
		limits[finger_name][2] -= 100
	  elif fingers[finger_name]['old_bending'] > limits[finger_name][1]:
		closing = True
		print(finger_name + str(fingers[finger_name]['status']) + ' --> 2')
		if record_demo:
			write_demo("bending", finger_name + str(fingers[finger_name]['status']) + ' --> 2')
		fingers[finger_name]['status'] = 2
		limits = original_limits
		limits[finger_name][1] += 100

	else:
	  if fingers[finger_name]['old_bending'] > limits[finger_name][2]:
		closing = True
		print(finger_name + str(fingers[finger_name]['status']) + ' --> 3')
		if record_demo:
			write_demo("bending", finger_name + str(fingers[finger_name]['status']) + ' --> 3')
		fingers[finger_name]['status'] = 3
		limits = original_limits
		limits[finger_name][2] += 100

	fingers[finger_name]['old_bending'] = fingers[finger_name]['bending']

      #Calculates pressure changes in each finger and writes in the demo files
      if print_pressure:

	if (fingers[finger_name]['pressure'] <= limits_pressure[finger_name][0]) and (fingers[finger_name]['status_pressure'] != 0):
		print(finger_name + ' pressure state' + str(fingers[finger_name]['status_pressure']) + ' --> 0 ' + str(fingers[finger_name]['pressure']))
		if record_demo:
			write_demo("pressure", finger_name + str(fingers[finger_name]['status_pressure']) + ' --> 0')
		fingers[finger_name]['status_pressure'] = 0

	elif (fingers[finger_name]['pressure'] > limits_pressure[finger_name][0]) and (fingers[finger_name]['pressure'] <= limits_pressure[finger_name][1]) and (fingers[finger_name]['status_pressure'] != 1):
		print(finger_name + ' pressure state' + str(fingers[finger_name]['status_pressure']) + ' --> 1: ' + str(fingers[finger_name]['pressure']))
		if record_demo:
			write_demo("pressure", finger_name + str(fingers[finger_name]['status_pressure']) + ' --> 1')
		fingers[finger_name]['status_pressure'] = 1
		limits_pressure = original_limits_pressure
		limits_pressure[finger_name][1] += 500

	elif (fingers[finger_name]['pressure'] > limits_pressure[finger_name][1]) and (fingers[finger_name]['status_pressure'] != 2):
		print(finger_name + ' pressure state' + str(fingers[finger_name]['status_pressure']) + ' --> 2: ' + str(fingers[finger_name]['pressure']))
		if record_demo:
			write_demo("pressure", finger_name+ str(fingers[finger_name]['status_pressure']) + ' --> 2')
		fingers[finger_name]['status_pressure'] = 2
		limits_pressure = original_limits_pressure
		limits_pressure[finger_name][1] -= 500

	if fingers[finger_name]['status_pressure'] > 0:
		contact = True

    if record_demo:
	    #Change of line when the info is updated
	    if change_bending:
		write_demo("bending_EOL", "")
	    if change_pressure:
		write_demo("pressure_EOL", "")
	    if change_bending or change_pressure:
		write_demo("fingers_EOL", "")

	    #Publish contact when the demo starts
	    if initial_contact:
		initial_contact = False
		if contact:
			publish_contact = False
			publish_no_contact = True
		else:
			publish_contact = True
			publish_no_contact = False
		contact_message = Bool()
    		contact_message.data = contact
		contact_pub.publish(contact_message)

	    #Publish in the general file when the hand is opening/closing and when it gets/loses in contact additionally publishes if there is contact or not
	    if opening and publish_opening:
		write_demo("general", "Opening")
		publish_closing = True
		publish_opening = False
		cold_down = 0
	    if closing and publish_closing:
		write_demo("general", "Closing")
		publish_opening = True
		publish_closing = False
		cold_down = 0
	    if cold_down >= 15:
		publish_opening = True
		publish_closing = True
	    cold_down += 1

	    if contact and publish_contact:
		write_demo("general", "Contact")
		publish_contact = False
		publish_no_contact = True
		contact_message = Bool()
    		contact_message.data = True
		contact_pub.publish(contact_message)
	    elif not contact and publish_no_contact:
		write_demo("general", "No contact")
		publish_contact = True
		publish_no_contact = False
		contact_message = Bool()
    		contact_message.data = False
		contact_pub.publish(contact_message)
    
    #Publish contact info before starting the demo so the other nodes know what is the state at the beginning
    else:
	contact_message = Bool()
    	contact_message.data = contact
	contact_pub.publish(contact_message)


def write_demo(type_message, message):
    global f_bending
    global f_pressure
    global f_fingers
    global f_fingers2
    global f_general
    global path_general
    global f_general_complete
    global path_general_complete
    global change_bending
    global change_pressure
    global fingers
    global orientation_hand
    global write_times
    global demo_time
    global first_bending
    global first_pressure

    current_time = 0
    if write_times:
	current_time = rospy.get_time() - demo_time
	coord_message = Float32()
    	coord_message.data = current_time
	coord_pub.publish(coord_message) #Signal to write the coordinates.txt

    if type_message == "bending":
	if write_times and first_bending: #For now just consider times for fingers2
		f_bending.write(str(current_time) + ":" + message + ",")
	else:
    		f_bending.write(message + ",")
	change_bending = True
	first_bending = False

    elif type_message == "bending_EOL":
	f_bending.write("\n")
	first_bending = True

    elif type_message == "pressure":
	if write_times and first_pressure: #For now just consider times for fingers2
		f_pressure.write(str(current_time) + ":" + message + ",")
	else:
		f_pressure.write(message + ",")
	change_pressure = True
	first_pressure = False

    elif type_message == "pressure_EOL":
	f_pressure.write("\n")
	first_pressure = True

    elif type_message == "fingers_EOL":
	fingers_line = str(fingers['thumb']['status']) + str(fingers['index']['status']) + str(fingers['middle']['status']) + str(fingers['ring']['status']) + str(fingers['pinky']['status']) + str(fingers['thumb']['status_pressure']) + str(fingers['index']['status_pressure']) + str(fingers['middle']['status_pressure']) + str(fingers['ring']['status_pressure']) + str(fingers['pinky']['status_pressure'])
	f_fingers.write(fingers_line + "\n")
	bending_temp = fingers_line[:5]
	pressure_temp = fingers_line[5:10]
	bent_fingers = 0
	touch_fingers = 0
	bent_fingers = 5 - int(bending_temp.count('0'))
	touch_fingers = 5 - int(pressure_temp.count('0'))
	bent_and_touch = 0
	for i in range(5):
		if bending_temp[i] != '0' and pressure_temp[i] != '0':
			bent_and_touch += 1
	closed_hand = bending_temp.count('2') > 0
	strong_touch = pressure_temp.count('2') > 0
	fingers2_line = "Bending" + str(bent_fingers) + ",Closed" + str(closed_hand) + ",Touch" + str(touch_fingers) + ",Strong" + str(strong_touch) + ",BentAndTouch" + str(bent_and_touch)
	if write_times: #For now just consider times for fingers2
		f_fingers2.write(str(current_time) + ":" + fingers2_line + "\n")
	else:
		f_fingers2.write(fingers2_line + "\n")
    if type_message == "general":
	f_general = open(path_general, "a")
	if write_times:
		message = str(current_time) + ":" + message
	f_general.write(message + "," + orientation_hand + "\n")
	f_general.close()
	f_general_complete = open(path_general_complete, "a")
	f_general_complete.write(message + "," + orientation_hand + "\n")
	f_general_complete.close()


#Subscribers
def glove_fingers_cb(info):
    analyze_fingers(info.data)

subsGloveR = rospy.Subscriber('Glove/fingers_string/right', String, glove_fingers_cb)


def start_cb(info):
    global record_demo
    global f_bending
    global f_pressure
    global f_fingers
    global f_fingers2
    global path_bending
    global path_pressure
    global path_fingers
    global path_fingers2
    global path_general
    global path_general_complete
    global initial_contact
    global write_times
    global demo_time
    global first_bending
    global first_pressure

    path = info.path
    write_times = info.times
    demo_time = rospy.get_time()
    first_bending = True
    first_pressure = True
    record_demo = True
    path_bending = path + '/finger_bending.txt'
    path_pressure = path + '/finger_pressure.txt'
    path_fingers = path + '/fingers.txt'
    path_fingers2 = path + '/fingers2.txt'
    path_general = path + '/demonstration.txt'
    path_general_complete = path + '/demonstration_complete.txt'
    f_bending = open(path_bending, "w")
    f_pressure = open(path_pressure, "w")
    f_fingers = open(path_fingers, "w")
    f_fingers2 = open(path_fingers2, "w")
    write_demo("fingers_EOL", "")
    initial_contact = True

subsStart = rospy.Subscriber('/Teaching/start_demo', start_demo_msg, start_cb)


def stop_cb(info):
    global record_demo
    global f_bending
    global f_pressure
    global f_fingers
    global f_fingers2
    record_demo = False
    f_bending.close()
    f_pressure.close()
    f_fingers.close()
    f_fingers2.close()

subsStop = rospy.Subscriber('/Teaching/stop_demo', Bool, stop_cb)


def ori_cb(info):
    global orientation_hand
    orientation_hand = info.data

subsOri = rospy.Subscriber('/Glove/right_hand_Zalignment', String, ori_cb)


print('Fingers node ready')
 # Keep this process running until Enter is pressed
print ("Press Enter to quit...")
try:
	sys.stdin.readline()
except KeyboardInterrupt:
	print("Fingers node finalized")
	pass
