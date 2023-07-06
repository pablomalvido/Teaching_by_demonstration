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
path = os.path.join(os.path.dirname(__file__), '../files/bottle/grasp_top/') #build model
path2 = os.path.join(os.path.dirname(__file__), '../files/bottle/grasp/') #compare with model
#file_name = "/demonstration_complete.txt"
#file_name = "/hand_rotation.txt"
files = ["/hand_xyz.txt", "/hand_rotation.txt", "/demonstration_complete.txt", "/demonstration.txt", "/hand_orientation_Z.txt", "/z.txt"]
prob_dict = {}

print("model of: " + path)
print("compared with observation of: " + path2)

for file_name in files:
	xyz_dict = {}
	xyz_line_accum = 0

	demo_index = os.listdir(path)

	for index in demo_index:
	  if index != '2' and index!= 'models': #
		path_index = path + str(index)
		path_xyz = path_index + file_name
		f = open(path_xyz, "r")
		for line in f:
			line = line.replace("\n", "")
			line = line.replace(" ", "")
			if (not (line in xyz_dict.keys())) and line!="":
				xyz_dict[line] = xyz_line_accum
				xyz_line_accum += 1

	xyz_trans_abs = np.zeros((len(xyz_dict),len(xyz_dict)))
	xyz_start_abs = np.zeros(len(xyz_dict))

	for index in demo_index:
	  if index != '2' and index!= 'models': #
		path_index = path + str(index)
		path_xyz = path_index + file_name
		f = open(path_xyz, "r")
		
		first_line = True
		prev_index = 0
		for line in f:
			line = line.replace("\n", "")
			line = line.replace(" ", "")
			if line != "":
				if first_line:
					xyz_start_abs[xyz_dict[line]] += 1
					first_line = False
				else:
					xyz_trans_abs[xyz_dict[line]][prev_index] += 1
				prev_index = xyz_dict[line]
				

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

	demo_index2 = os.listdir(path2)
	prob = np.ones(len(demo_index2)-1)
	n_index = 0
	for index in demo_index2:
	  if index!= 'models': #
		path_index = path2 + str(index)
		path_xyz = path_index + file_name
		f = open(path_xyz, "r")
		first_line = True
		prev_index = 0
		for line in f:
			line = line.replace("\n", "")
			line = line.replace(" ", "")
			if line in xyz_dict.keys():
				if first_line:
					prob[n_index] = xyz_start_abs[xyz_dict[line]]/max(xyz_start_abs)
					first_line = False
				else:
					prob[n_index] *= xyz_trans_abs[xyz_dict[line]][prev_index]/max(xyz_trans_abs[xyz_dict[line]])
			elif line == "":
				continue
			else:
				prob[n_index] = 0
				break
			prev_index = xyz_dict[line]
		n_index += 1
	#print(prob)
	prob_dict[file_name] = prob

print(prob_dict)

avg_prob = np.zeros((len(prob_dict[files[0]])))
for file_name in files:
	demo = 0
	for i in prob_dict[file_name]:
		avg_prob[demo] += prob_dict[file_name][demo]
		demo +=1

avg_prob = avg_prob/len(prob_dict)
print(avg_prob)
