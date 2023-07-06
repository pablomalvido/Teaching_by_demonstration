#!/usr/bin/env python
import rospy
import triad_openvr
import time
import sys
import os
from std_msgs.msg import String

rospy.init_node('tracker_adquisition') 
right_tracker = rospy.Publisher('Tracker/pose_right', String, queue_size=1)
left_tracker = rospy.Publisher('Tracker/pose_left', String, queue_size=1)

v = triad_openvr.triad_openvr()
v.print_discovered_objects()

if len(sys.argv) == 1:
    #interval = 1/250
    interval(0.15)
elif len(sys.argv) == 2:
    interval = 1/float(sys.argv[1])
else:
    print("Invalid number of arguments")
    interval = False
    
if interval:
    while not rospy.is_shutdown():
        start = time.time()
        txt = ""
        for device in v.object_names["Tracker"]:
            tracker_info = String()
            txt += str(device) + ":"
            for each in v.devices[device].get_pose_euler():
                txt += "%.4f" % each
                txt += " "
                tracker_info.data += "%.4f" % each
                tracker_info.data += ";"
            
            if str(device) == "tracker_1":
                right_tracker.publish(tracker_info)
            elif str(device) == "tracker_2":
                left_tracker.publish(tracker_info)

            txt += "\n"
        print("\r" + txt, end="")
        sleep_time = interval-(time.time()-start)
        if sleep_time>0:
            time.sleep(sleep_time)
