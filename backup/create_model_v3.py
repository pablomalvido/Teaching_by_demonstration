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
path = os.path.join(os.path.dirname(__file__), '../files/bottle/pour_water/') #build model of this operation
path_models = path + 'models'
#file_name = "/demonstration_complete.txt"
files = ["/hand_xyz.txt", "/hand_rotation.txt", "/demonstration_complete.txt", "/demonstration.txt", "/hand_orientation_Z.txt", "/z.txt"]
prob_dict = {}

for file_name in files:
	xyz_dict = {}
	xyz_line_accum = 0

	demo_index = os.listdir(path)

	for index in demo_index:
	  if index != 'models': 
		path_index = path + str(index)
		path_xyz = path_index + file_name
		f = open(path_xyz, "r")
		for line in f:
			line = line.replace("\n", "")
			line = line.replace(" ", "")
			if (not (line in xyz_dict.keys())) and line!="":
				xyz_dict[line] = xyz_line_accum
				xyz_line_accum += 1
		f.close()

	xyz_trans_abs = np.zeros((len(xyz_dict),len(xyz_dict)))
	xyz_start_abs = np.zeros(len(xyz_dict))

	for index in demo_index:
	  if index != 'models':
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
		f.close()
				

	xyz_start_abs = xyz_start_abs/xyz_start_abs.sum()
	#print(xyz_start_abs)

	#print(xyz_trans_abs)
	line_index = 0
	for line in xyz_trans_abs:
		if line.sum() != 0:
			xyz_trans_abs[line_index] = line/line.sum()
		line_index += 1

	#print(xyz_trans_abs)
	
	path_model_new = path_models + str(file_name)
	#os.remove(path_model_new)
	f_model = open(path_model_new, "w")
	f_model.write("Start:\n")
	for col in xyz_start_abs:
		f_model.write(str(col) + ",")
	f_model.write("\nTransition:\n")
	for row in xyz_trans_abs:
		for col in row:
			f_model.write(str(col) + ",")
		f_model.write("\n")
	f_model.write("Map:\n")
	for iden in xyz_dict:
		f_model.write(str(iden) + ":" + str(xyz_dict[iden]) + "\n")
	f_model.close()
