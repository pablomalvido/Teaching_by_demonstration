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
from std_msgs.msg import Float32
from teaching_pkg.msg import *
from os.path import abspath


#ROS init
rospy.init_node('create_model')
path = os.path.join(os.path.dirname(__file__), '../files/bottle/grasp/')
file_name = "/demonstration_complete.txt"
#file_name = "/hand_rotation.txt"
file_name = "/hand_xyz.txt"

xyz_dict = {}
xyz_line_accum = 0

demo_index = os.listdir(path)

for index in demo_index:
	path_index = path + str(index)
	path_xyz = path_index + file_name
	f = open(path_xyz, "r")
	for line in f:
		if not (line[:-1] in xyz_dict.keys()):
			xyz_dict[line[:-1]] = xyz_line_accum
			xyz_line_accum += 1

xyz_trans_abs = np.zeros((len(xyz_dict),len(xyz_dict)))
xyz_start_abs = np.zeros(len(xyz_dict))

for index in demo_index:
	path_index = path + str(index)
	path_xyz = path_index + file_name
	f = open(path_xyz, "r")
	
	first_line = True
	prev_index = 0
	for line in f:
		if first_line:
			xyz_start_abs[xyz_dict[line[:-1]]] += 1
			first_line = False
		else:
			xyz_trans_abs[xyz_dict[line[:-1]]][prev_index] += 1
		prev_index = xyz_dict[line[:-1]]
			

xyz_start_abs = xyz_start_abs/xyz_start_abs.sum()
#print(xyz_start_abs)

#print(xyz_trans_abs)
line_index = 0
for line in xyz_trans_abs:
	if line.sum() != 0:
		xyz_trans_abs[line_index] = line/line.sum()
	line_index += 1

#print(xyz_trans_abs)


"""
path_xyz = path + "3" + file_name
f = open(path_xyz, "r")
first_line = True
prev_index = 0
prob = 1
"""

prob = np.ones(len(demo_index))
n_index = 0
for index in demo_index:
	path_index = path + str(index)
	path_xyz = path_index + file_name
	f = open(path_xyz, "r")
	first_line = True
	prev_index = 0
	for line in f:
		if line[:-1] in xyz_dict.keys():
			if first_line:
				prob[n_index] = xyz_start_abs[xyz_dict[line[:-1]]]/max(xyz_start_abs)
				first_line = False
			else:
				prob[n_index] *= xyz_trans_abs[xyz_dict[line[:-1]]][prev_index]/max(xyz_trans_abs[xyz_dict[line[:-1]]])
		else:
			prob[n_index] = 0
			break
		prev_index = xyz_dict[line[:-1]]
	n_index += 1
print(prob)
