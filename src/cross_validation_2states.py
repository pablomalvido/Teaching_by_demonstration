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
rospy.init_node('cross_validation_1state')
path = os.path.join(os.path.dirname(__file__), '../files/bottle/') #build model of this operation
#path_demos = path + operation + 'demos/'
#path_models = path + operation + 'models/models1'
#file_name = "/demonstration_complete.txt"
files = ["/hand_xyz.txt", "/hand_rotation.txt", "/demonstration_complete.txt", "/demonstration.txt", "/hand_orientation_Z.txt", "/z.txt"]
operations = ["grasp", "grasp_top", "pour_water", "place", "place_top"]


def define_model(op, exclude_obs = -1):
    model_dict = {}
    #For going through the files for each operation
    for file_name in files:
	op_dict = {}
	file_dict = {}
	file_line_accum = 0

	path_demos = path + op + "/demos/"
	demo_index = os.listdir(path_demos)

	for index in demo_index:
	  #Exclude the index that is gonna be compared
	  if index != exclude_obs:
		path_index = path_demos + str(index)
		path_file = path_index + file_name
		f = open(path_file, "r")
		for line in f:
			line = line.replace("\n", "")
			line = line.replace(" ", "")
			if (not (line in file_dict.keys())) and line!="":
				file_dict[line] = file_line_accum
				file_line_accum += 1
		f.close()

	#Creates the start and transition matrixes
	file_trans_abs = np.zeros((len(file_dict),len(file_dict),len(file_dict)))
	file_start_abs = np.zeros((2, len(file_dict)))

	for index in demo_index:
	  if index != exclude_obs:
		path_index = path_demos + str(index)
		path_file = path_index + file_name
		f = open(path_file, "r")
		
		first_line = True
		second_line = False
		prev_index1 = 0
		prev_index2 = 0
		for line in f:
			line = line.replace("\n", "")
			line = line.replace(" ", "")
			if line != "":
				if first_line:
					file_start_abs[0][file_dict[line]] += 1
					first_line = False
					second_line = True
					prev_index2 = file_dict[line]
				elif second_line:
					file_start_abs[1][file_dict[line]] += 1
					second_line = False
					prev_index1 = file_dict[line]
				else:
					file_trans_abs[prev_index2][prev_index1][file_dict[line]] += 1
					prev_index2 = prev_index1
					prev_index1 = file_dict[line]
		f.close()
	

	line_index = 0
	for start_line in file_start_abs:
		if start_line.sum() != 0:
			file_start_abs[line_index] = start_line/start_line.sum()
		line_index += 1

	#print(file_start_abs)

	line_index1 = 0
	line_index2 = 0
	for trans_matrix in file_trans_abs:
		for trans_line in trans_matrix:
			if trans_line.sum() != 0:
				file_trans_abs[line_index2][line_index1] = trans_line/trans_line.sum()
			line_index1 += 1
		line_index2 += 1
		line_index1 = 0

	#print(file_trans_abs)

	#Don't write, return the values and use them to compare
	op_dict = {'start': file_start_abs, 'trans': file_trans_abs, 'map': file_dict}
	model_dict[file_name[1:-4]] = op_dict

	"""
	path_model_new = path_models + str(file_name)
	#os.remove(path_model_new)
	f_model = open(path_model_new, "w")
	f_model.write("Start:\n")
	for col in file_start_abs:
		f_model.write(str(col) + ",")
	f_model.write("\nTransition:\n")
	for row in file_trans_abs:
		for col in row:
			f_model.write(str(col) + ",")
		f_model.write("\n")
	f_model.write("Map:\n")
	for iden in file_dict:
		f_model.write(str(iden) + ":" + str(file_dict[iden]) + "\n")
	f_model.close()
	"""

    return model_dict


def compare_model(op, obs, model_dict):
    prob = {}
    avg_prob = {}
    path_obs = path + op + "/demos/" + obs
    for operation in operations:
      if operation != "sequence": #Not needed now because operations is a predefined list
	operation_model = operation
	prob[operation] = {}
	if operation == op:
		operation_model = 'excluded'
	for model in files:
		path_obs_file = path_obs + str(model)
		f = open(path_obs_file, "r")
		first_line = True
		second_line = False
		prev_map_line2 = 0
		prev_map_line1 = 0
		prob[operation][model[1:-4]] = 1
		for line in f:
			line = line.replace("\n", "")
			line = line.replace(" ", "")
			if line in model_dict[operation_model][model[1:-4]]['map'].keys():
				map_line = model_dict[operation_model][model[1:-4]]['map'][line]
				if first_line:
					if model_dict[operation_model][model[1:-4]]['start'][0][map_line] != 0:
						prob[operation][model[1:-4]] = model_dict[operation_model][model[1:-4]]['start'][0][map_line]/max(model_dict[operation_model][model[1:-4]]['start'][0])
						first_line = False
						second_line = True
					else:
						prob[operation][model[1:-4]] = 0
						break
				elif second_line:
					if model_dict[operation_model][model[1:-4]]['start'][1][map_line] != 0:
						prob[operation][model[1:-4]] *= model_dict[operation_model][model[1:-4]]['start'][1][map_line]/max(model_dict[operation_model][model[1:-4]]['start'][1])
						second_line = False
					else:
						prob[operation][model[1:-4]] = 0
						break
				else:
					if model_dict[operation_model][model[1:-4]]['trans'][prev_map_line2][prev_map_line1][map_line] != 0:
						prob[operation][model[1:-4]] *= model_dict[operation_model][model[1:-4]]['trans'][prev_map_line2][prev_map_line1][map_line]/max(model_dict[operation_model][model[1:-4]]['trans'][prev_map_line2][prev_map_line1])
					else:
						prob[operation][model[1:-4]] = 0
						break
			elif line == "":
				continue
			else:
				prob[operation][model[1:-4]] = 0
				break
			prev_map_line2 = prev_map_line1
			prev_map_line1 = map_line

	#print(prob)
	avg_prob[operation] = 0
	for model in files:
		avg_prob[operation] += prob[operation][model[1:-4]]
	avg_prob[operation] = avg_prob[operation]/len(prob[operation])

    #print(model_dict) 
    #print(prob)
    #print(avg_prob)

    max_prob_op = ""
    max_prob = 0
    for operation in operations:
  	if operation != "sequence":
		if avg_prob[operation] > max_prob:
			max_prob_op = operation
			max_prob = avg_prob[operation]

    return max_prob_op, avg_prob


def print_confusion_matrix(confusion_matrix, decimal = False):
	#Print headers
	headers = "\t"
	for model_op in confusion_matrix:
		headers += "\t" + model_op + "   "
	print(headers)
	#Print table
	for model_op in confusion_matrix:
		table_row = ""
		table_row = model_op + "   "+ "\t"
		for obs_op in confusion_matrix[model_op]:
			if decimal:
				table_row += str(round(confusion_matrix[model_op][obs_op],4)) + "\t" + "\t"
			else:
				table_row += str(confusion_matrix[model_op][obs_op]) + "\t" + "\t"
		print(table_row)


#Create full models of all op and store them in a dict
model_dict = {}
for op in operations:
    if op != "sequence": #Not needed now because operations is a predefined list
	model_dict[op] = define_model(op)

confusion_matrix = {}
for op in operations:
	confusion_matrix[op] = {}
	for op2 in operations:
		confusion_matrix[op][op2] = 0

	path_demos = path + op + "/demos/"
	demo_index = os.listdir(path_demos)
	for obs in demo_index:
		#Create model of op without demo
		model_dict['excluded'] = define_model(op, exclude_obs = obs)
		#Compare op/demo with all the models and print results
		result_op, results = compare_model(op, obs, model_dict)
		#Fill confusion matrix
		confusion_matrix[op][result_op] += 1

#Convert confusion matrix in %
confusion_matrix_perc = {}
for model_op in confusion_matrix:
	confusion_matrix_perc[model_op] = {}
	line_sum = 0
	for obs_op in confusion_matrix[model_op]:
		line_sum += confusion_matrix[model_op][obs_op]
	for obs_op in confusion_matrix[model_op]:
		confusion_matrix_perc[model_op][obs_op] = float(confusion_matrix[model_op][obs_op])/float(line_sum)
	
#Print confusion matrix
print_confusion_matrix(confusion_matrix, False)
print("\n")
print_confusion_matrix(confusion_matrix_perc, True)


