#!/usr/bin/env python

import socket
from std_msgs.msg import String
import tf 
import PyKDL 
import re
import rospy
from geometry_msgs.msg import Pose
import sys

rospy.init_node('tracker_tf_node', anonymous=True)
broad = tf.TransformBroadcaster()


def broadcastTransform(br, frame, frame_id, parent_frame, time):
	#Function for broadcasting transforms 
	time2 = rospy.get_rostime()
        br.sendTransform((frame.p.x(), frame.p.y(), frame.p.z()), 
            frame.M.GetQuaternion(), 
            time2, 
            frame_id, 
            parent_frame) 


####### Subscriptors #########
def right_tracker_cb(info):
    position_VIVE = info.data
    x_tracker = float((position_VIVE.split(';'))[2]) #Horiz1 axis
    y_tracker = float((position_VIVE.split(';'))[0]) #Horiz2 axis
    z_tracker =  float((position_VIVE.split(';'))[1]) #Vert axis
    R_tracker = float((position_VIVE.split(';'))[3]) #Horiz1 axis (around x)
    P_tracker = float((position_VIVE.split(';'))[5]) #Horiz2 axis (around y)
    Y_tracker =  float((position_VIVE.split(';'))[4]) #Vert axis (around z)
    frame_tf = PyKDL.Frame()
    frame_tf.p = PyKDL.Vector(x_tracker, y_tracker, z_tracker)
    frame_tf.M = PyKDL.Rotation.RPY(R_tracker, P_tracker, Y_tracker)
    tf_name = "right_hand_tracker"
    parent_tf = "world"
    current_time = rospy.get_rostime()
    broadcastTransform(broad, frame_tf, tf_name, parent_tf, current_time)
    
subsTrackerR = rospy.Subscriber('Tracker/pose_string/right', String, right_tracker_cb)


####### Quit #######
print('Tracker tf node ready')
 # Keep this process running until Enter is pressed
print ("Press Enter to quit...")
try:
	sys.stdin.readline()
except KeyboardInterrupt:
	print("Tracker tf node finalized")
	pass
