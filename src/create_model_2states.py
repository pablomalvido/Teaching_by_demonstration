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
rospy.init_node('create_model_2states')
object_name = str(rospy.get_param('~object', ""))
path_object = os.path.join(os.path.dirname(__file__), '../files/' + object_name + '/') #build model of this operation

#file_name = "/demonstration_complete.txt"
#files = ["/hand_xyz.txt", "/hand_rotation.txt", "/demonstration_complete.txt", "/demonstration.txt", "/hand_orientation_Z.txt", "/z.txt"]
files = ["/hand_xyz.txt", "/hand_rotation.txt", "/demonstration_complete.txt", "/demonstration.txt", "/hand_orientation_Z.txt", "/z.txt", "/finger_bending.txt", "/finger_pressure.txt"]


operations = os.listdir(path_object)
for operation in operations:
 if operation != 'sequence' and operation != "shake":
  path = path_object + operation + '/'
  path_demos = path + 'demos/'
  path_models = path + 'models/models2'
  prob_dict = {}
  for file_name in files:
	xyz_dict = {}
	xyz_line_accum = 0

	demo_index = os.listdir(path_demos)
	n_states = []

	#Creates the map dictionary
	for index in demo_index:
	  #if index[:6] != 'models' and index[:1] != '2': 
	  #if index[:6] != 'models':
		states_count = 0
		path_index = path_demos + str(index)
		path_xyz = path_index + file_name
		f = open(path_xyz, "r")
		for line in f:
			line = line.replace("\n", "")
			line = line.replace(" ", "")
			if line!="":
				states_count += 1
			if (not (line in xyz_dict.keys())) and line!="":
				xyz_dict[line] = xyz_line_accum
				xyz_line_accum += 1
		f.close()
		n_states.append(states_count)

	#Creates the start and transition matrixes
	xyz_trans_abs = np.zeros((len(xyz_dict),len(xyz_dict),len(xyz_dict)))
	xyz_start_abs = np.zeros((2, len(xyz_dict)))
	n_states_avg = sum(n_states)/len(n_states)

	for index in demo_index:
	  #if index[:6] != 'models' and index[:1] != '2':
	  #if index[:6] != 'models':
		path_index = path_demos + str(index)
		path_xyz = path_index + file_name
		f = open(path_xyz, "r")
		
		first_line = True
		second_line = False
		prev_index1 = 0
		prev_index2 = 0
		for line in f:
			line = line.replace("\n", "")
			line = line.replace(" ", "")
			if line != "":
				if first_line:
					xyz_start_abs[0][xyz_dict[line]] += 1
					first_line = False
					second_line = True
					prev_index2 = xyz_dict[line]
				elif second_line:
					xyz_start_abs[1][xyz_dict[line]] += 1
					second_line = False
					prev_index1 = xyz_dict[line]
				else:
					xyz_trans_abs[prev_index2][prev_index1][xyz_dict[line]] += 1
					prev_index2 = prev_index1
					prev_index1 = xyz_dict[line]
		f.close()
				

	line_index = 0
	for start_line in xyz_start_abs:
		if start_line.sum() != 0:
			xyz_start_abs[line_index] = start_line/start_line.sum()
		line_index += 1

	line_index1 = 0
	line_index2 = 0
	for trans_matrix in xyz_trans_abs:
		for trans_line in trans_matrix:
			if trans_line.sum() != 0:
				xyz_trans_abs[line_index2][line_index1] = trans_line/trans_line.sum()
			line_index1 += 1
		line_index2 += 1
		line_index1 = 0

	#print(xyz_trans_abs)


	#Write the calculated model in a file	
	path_model_new = path_models + str(file_name)
	f_model = open(path_model_new, "w")
	f_model.write("Start:\n")
	for row in xyz_start_abs:
		for col in row:
			f_model.write(str(col) + ",")
		f_model.write("\n")
	f_model.write("Transition:\n")
	for mat in xyz_trans_abs:
		for row in mat:
			for col in row:
				f_model.write(str(col) + ",")
			f_model.write("\n")
		f_model.write("-\n")
	f_model.write("Map:\n")
	for iden in xyz_dict:
		f_model.write(str(iden) + ":" + str(xyz_dict[iden]) + "\n")
	f_model.write("n_states:\n")
	f_model.write(str(n_states_avg) + "\n")
	f_model.close()
