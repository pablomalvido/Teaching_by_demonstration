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
rospy.init_node('cross_validation_all')
object_name = rospy.get_param('~object', "")
path = os.path.join(os.path.dirname(__file__), '../files/' + object_name + '/') #build model of this operation
#path_demos = path + operation + 'demos/'
#file_name = "/demonstration_complete.txt"
#files = ["/hand_xyz.txt", "/hand_rotation.txt", "/demonstration_complete.txt", "/demonstration.txt", "/hand_orientation_Z.txt", "/z.txt"]
files = ["/hand_xyz.txt", "/hand_rotation.txt", "/demonstration_complete.txt", "/demonstration.txt", "/hand_orientation_Z.txt", "/z.txt", "/finger_bending.txt", "/finger_pressure.txt"]
operations = os.listdir(path)
operations.remove("sequence")
print(operations)
#models_info = {"hand_xyz.txt": {"start": 2, "trans": 2, "weight": 1}, "hand_rotation.txt": {"start": 2, "trans": 2, "weight": 1}, "demonstration_complete.txt": {"start": 3, "trans": 2, "weight": 1}, "demonstration.txt": {"start": 3, "trans": 2, "weight": 1}, "hand_orientation_Z.txt": {"start": 1, "trans": 1, "weight": 3}, "z.txt": {"start": 2, "trans": 2, "weight": 1}}
models_info = {"hand_xyz.txt": {"start": 2, "trans": 2, "weight": 1}, "hand_rotation.txt": {"start": 2, "trans": 2, "weight": 1}, "demonstration_complete.txt": {"start": 3, "trans": 2, "weight": 1}, "demonstration.txt": {"start": 3, "trans": 2, "weight": 1}, "hand_orientation_Z.txt": {"start": 1, "trans": 1, "weight": 1}, "z.txt": {"start": 2, "trans": 2, "weight": 1}, "finger_bending.txt": {"start": 2, "trans": 2, "weight": 1}, "finger_pressure.txt": {"start": 2, "trans": 2, "weight": 1}}

if object_name =="bottle":
	files = ["/hand_xyz.txt", "/hand_rotation.txt", "/demonstration_complete.txt", "/demonstration.txt", "/hand_orientation_Z.txt", "/z.txt"]
	models_info = {"hand_xyz.txt": {"start": 2, "trans": 2, "weight": 1}, "hand_rotation.txt": {"start": 2, "trans": 2, "weight": 1}, "demonstration_complete.txt": {"start": 3, "trans": 2, "weight": 3}, "demonstration.txt": {"start": 3, "trans": 2, "weight": 1}, "hand_orientation_Z.txt": {"start": 1, "trans": 1, "weight": 1}, "z.txt": {"start": 2, "trans": 2, "weight": 1}}
	#models_info = {"hand_xyz.txt": {"start": 3, "trans": 3, "weight": 1}, "hand_rotation.txt": {"start": 3, "trans": 3, "weight": 1}, "demonstration_complete.txt": {"start": 3, "trans": 3, "weight": 1}, "demonstration.txt": {"start": 3, "trans": 3, "weight": 1}, "hand_orientation_Z.txt": {"start": 3, "trans": 3, "weight": 1}, "z.txt": {"start": 3, "trans": 3, "weight": 1}}


def define_models_weights():
	global object_name
	global models_info
	save_path = os.path.join(os.path.dirname(__file__), '../files/zz_other/models_weights/' + str(object_name) + ".txt")
	f_weights = open(save_path, "r")

	for line in f_weights:
		line = line.replace("\n", "")
		model = line.split(":")
		models_info[str(model[0])]['weight'] = float(model[1])
	f_weights.close()


def define_model_1(op, exclude_obs = -1):
    model_dict = {}
    #For going through the files for each operation
    for file_name in files:
	op_dict = {}
	file_dict = {}
	file_line_accum = 0

	path_demos = path + op + "/demos/"
	demo_index = os.listdir(path_demos)
	n_states = []

	for index in demo_index:
	  #Exclude the index that is gonna be compared
	  if index != exclude_obs:
		states_count = 0
		path_index = path_demos + str(index)
		path_file = path_index + file_name
		f = open(path_file, "r")
		for line in f:
			line = line.replace("\n", "")
			line = line.replace(" ", "")
			if line != "":
				states_count += 1
			if (not (line in file_dict.keys())) and line!="":
				file_dict[line] = file_line_accum
				file_line_accum += 1
		f.close()
		n_states.append(states_count)

	file_trans_abs = np.zeros((len(file_dict),len(file_dict)))
	file_start_abs = np.zeros(len(file_dict))
	n_states_avg = sum(n_states)/len(n_states)

	for index in demo_index:
	  if index != exclude_obs:
		path_index = path_demos + str(index)
		path_file = path_index + file_name
		f = open(path_file, "r")
		
		first_line = True
		prev_index = 0
		for line in f:
			line = line.replace("\n", "")
			line = line.replace(" ", "")
			if line != "":
				if first_line:
					file_start_abs[file_dict[line]] += 1
					first_line = False
				else:
					file_trans_abs[prev_index][file_dict[line]] += 1
				prev_index = file_dict[line]
		f.close()
				

	file_start_abs = file_start_abs/file_start_abs.sum()
	#print(file_start_abs)

	line_index = 0
	for line in file_trans_abs:
		if line.sum() != 0:
			file_trans_abs[line_index] = line/line.sum()
		line_index += 1

	#print(file_trans_abs)

	#Don't write, return the values and use them to compare
	op_dict = {'start': file_start_abs, 'trans': file_trans_abs, 'map': file_dict, 'n_states': n_states_avg}
	model_dict[file_name[1:-4]] = op_dict #The models for every txt file

    return model_dict



def define_model_2(op, exclude_obs = -1):
    model_dict = {}
    #For going through the files for each operation
    for file_name in files:
	op_dict = {}
	file_dict = {}
	file_line_accum = 0

	path_demos = path + op + "/demos/"
	demo_index = os.listdir(path_demos)
	n_states = []

	for index in demo_index:
	  #Exclude the index that is gonna be compared
	  if index != exclude_obs:
		states_count = 0
		path_index = path_demos + str(index)
		path_file = path_index + file_name
		f = open(path_file, "r")
		for line in f:
			line = line.replace("\n", "")
			line = line.replace(" ", "")
			if line!="":
				states_count += 1
			if (not (line in file_dict.keys())) and line!="":
				file_dict[line] = file_line_accum
				file_line_accum += 1
		f.close()
		n_states.append(states_count)

	#Creates the start and transition matrixes
	file_trans_abs = np.zeros((len(file_dict),len(file_dict),len(file_dict)))
	file_start_abs = np.zeros((2, len(file_dict)))
	n_states_avg = sum(n_states)/len(n_states)

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
	op_dict = {'start': file_start_abs, 'trans': file_trans_abs, 'map': file_dict, 'n_states': n_states_avg}
	model_dict[file_name[1:-4]] = op_dict

    return model_dict



def define_model_3(op, exclude_obs = -1):
    model_dict = {}
    #For going through the files for each operation
    for file_name in files:
	op_dict = {}
	file_dict = {}
	file_line_accum = 0

	path_demos = path + op + "/demos/"
	demo_index = os.listdir(path_demos)
	n_states = []

	for index in demo_index:
	  #Exclude the index that is gonna be compared
	  if index != exclude_obs:
		states_count = 0
		path_index = path_demos + str(index)
		path_file = path_index + file_name
		f = open(path_file, "r")
		for line in f:
			line = line.replace("\n", "")
			line = line.replace(" ", "")
			if line!="":
				states_count += 1
			if (not (line in file_dict.keys())) and line!="":
				file_dict[line] = file_line_accum
				file_line_accum += 1
		f.close()
		n_states.append(states_count)

	#Creates the start and transition matrixes
	file_trans_abs = np.zeros((len(file_dict),len(file_dict),len(file_dict),len(file_dict)))
	file_start_abs = np.zeros((3, len(file_dict)))
	n_states_avg = float(sum(n_states))/float(len(n_states))

	for index in demo_index:
	  if index != exclude_obs:
		path_index = path_demos + str(index)
		path_file = path_index + file_name
		f = open(path_file, "r")
		
		first_line = True
		second_line = False
		third_line = False
		prev_index1 = 0
		prev_index2 = 0
		prev_index3 = 0
		for line in f:
			line = line.replace("\n", "")
			line = line.replace(" ", "")
			if line != "":
				if first_line:
					file_start_abs[0][file_dict[line]] += 1
					first_line = False
					second_line = True
					prev_index3 = file_dict[line]
				elif second_line:
					file_start_abs[1][file_dict[line]] += 1
					second_line = False
					third_line = True
					prev_index2 = file_dict[line]
				elif third_line:
					file_start_abs[2][file_dict[line]] += 1
					third_line = False
					prev_index1 = file_dict[line]
				else:
					file_trans_abs[prev_index3][prev_index2][prev_index1][file_dict[line]] += 1
					prev_index3 = prev_index2
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
	line_index3 = 0
	for trans_matrix3 in file_trans_abs:
	    	for trans_matrix2 in trans_matrix3:
			for trans_line in trans_matrix2:
				if trans_line.sum() != 0:
					file_trans_abs[line_index3][line_index2][line_index1] = trans_line/trans_line.sum()
					line_index1 += 1
			line_index2 += 1
			line_index1 = 0
		line_index3 += 1
		line_index2 = 0
		line_index1 = 0

	#print(file_trans_abs)

	#Don't write, return the values and use them to compare
	op_dict = {'start': file_start_abs, 'trans': file_trans_abs, 'map': file_dict, 'n_states': n_states_avg}
	model_dict[file_name[1:-4]] = op_dict

    return model_dict



def compare_model(op, obs, model_dict):
    global models_info
    prob = {}
    avg_prob = {}
    start_model_n = 0
    trans_model_n = 0
    path_obs = path + op + "/demos/" + obs
    for operation in operations:
      if operation != "sequence" and operation != "shake": #Not needed now because operations is a predefined list
	operation_model = operation
	prob[operation] = {}
	if operation == op:
		operation_model = 'excluded'
	for model in files:
		model_name = model[1:-4]
		path_obs_file = path_obs + str(model)
		f = open(path_obs_file, "r")
		first_line = True
		second_line = False
		third_line = False
		prev_map_line3 = 999
		prev_map_line2 = 999
		prev_map_line1 = 999
		start_model_n = min(int(model_dict[3][operation_model][model_name]['n_states']), int(models_info[model[1:]]['start']))
		trans_model_n = min(int(model_dict[3][operation_model][model_name]['n_states'])-1, int(models_info[model[1:]]['trans']))
		#prob_model = 1
		prob[operation][model_name] = 1

		prob_accum = 1
		for line in f:
			line = line.replace("\n", "")
			line = line.replace(" ", "")
			if line == "":
				continue

			prob_line = 0
			
			#Found map in that operation
			if line in model_dict[3][operation_model][model_name]['map'].keys():
				map_line = model_dict[3][operation_model][model_name]['map'][line]			

				#First line prob
				if first_line:
					if model_dict[3][operation_model][model_name]['start'][0][map_line] > 0:
						prob_line = model_dict[3][operation_model][model_name]['start'][0][map_line]/max(model_dict[3][operation_model][model_name]['start'][0])
						first_line = False
						second_line = True
					
				#Second line prob
				elif second_line and start_model_n > 1:
					if model_dict[3][operation_model][model_name]['start'][1][map_line] > 0:
						prob_line = model_dict[3][operation_model][model_name]['start'][1][map_line]/max(model_dict[3][operation_model][model_name]['start'][1])
						second_line = False
						third_line = True
					
				#Third line prob
				elif third_line and start_model_n > 2:
					if model_dict[3][operation_model][model_name]['start'][2][map_line] > 0:
						prob_line = model_dict[3][operation_model][model_name]['start'][2][map_line]/max(model_dict[3][operation_model][model_name]['start'][2])
						third_line = False

				#Transition
				else:
				    if trans_model_n == 0:
					pass #Don't consider transitions, just the state itself (start prob)

				    elif trans_model_n == 1 and prev_map_line1 != 999:
					if model_dict[1][operation_model][model_name]['trans'][prev_map_line1][map_line] > 0:
						prob_line = model_dict[1][operation_model][model_name]['trans'][prev_map_line1][map_line]/max(model_dict[1][operation_model][model_name]['trans'][prev_map_line1])

				    elif trans_model_n == 2 and prev_map_line1 != 999 and prev_map_line2 != 999:
					if model_dict[2][operation_model][model_name]['trans'][prev_map_line2][prev_map_line1][map_line] > 0:
						prob_line = model_dict[2][operation_model][model_name]['trans'][prev_map_line2][prev_map_line1][map_line]/max(model_dict[2][operation_model][model_name]['trans'][prev_map_line2][prev_map_line1])

				    elif trans_model_n == 3 and prev_map_line1 != 999 and prev_map_line2 != 999 and prev_map_line3 != 999:
					if model_dict[3][operation_model][model_name]['trans'][prev_map_line3][prev_map_line2][prev_map_line1][map_line] > 0:
						prob_line = model_dict[3][operation_model][model_name]['trans'][prev_map_line3][prev_map_line2][prev_map_line1][map_line]/max(model_dict[3][operation_model][model_name]['trans'][prev_map_line3][prev_map_line2][prev_map_line1])

				#Update cycle
				prev_map_line3 = prev_map_line2
				prev_map_line2 = prev_map_line1
				prev_map_line1 = map_line

			#Line not found
			else:
				prev_map_line3 = prev_map_line2
				prev_map_line2 = prev_map_line1
				prev_map_line1 = 999
				if second_line:
					third_line = True
					second_line = False
				if first_line:
					second_line = True
					first_line = False #If it doesn't enter in the first line
				else:
					third_line = False #If it doesn't enter in the second line

			#Update probabilities
			prob_accum *= prob_line
			if prob_accum == 0:
				break

		prob[operation][model_name] = prob_accum

	#Weighted average of all the models
	avg_prob[operation] = 0
	weight_sum = 0
	for model in files:
		model_name = model[1:-4]
		avg_prob[operation] += prob[operation][model_name] * models_info[model[1:]]['weight']
		weight_sum += models_info[model[1:]]['weight']
	avg_prob[operation] = avg_prob[operation]/weight_sum

    #print(model_dict) 
    #print(prob)
    #print(avg_prob)

    max_prob_op = ""
    max_prob = -1
    for operation in operations:
  	if operation != "sequence" and operation != "shake":
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



def save_confusion_matrix(confusion_matrix):
	object_name = path.split("/")[-2:-1][0]
	remove_item = len(object_name)+1
	path_new = path[:-remove_item] + "/zz_other/confusion_matrix_weights/" + str(object_name) + ".txt"
	f_weights = open(path_new, "w")

	#Names operations
	models_line = ""
	for model_op in confusion_matrix:
		models_line += str(model_op) + ","
	f_weights.write(models_line[:-1] + "\n")
	
	#Weights
	for model_op in confusion_matrix:
		weights_line = ""
		for obs_op in confusion_matrix[model_op]:
			weights_line += str(confusion_matrix[model_op][obs_op]) + ","
		f_weights.write(weights_line[:-1] + "\n")
	f_weights.close()



#Create full models of all op and store them in a dict
#define_models_weights()
model_dict = {1:{}, 2:{}, 3:{}}
for op in operations:
    if op != "sequence": #Not needed now because operations is a predefined list
	model_dict[1][op] = define_model_1(op)
	model_dict[2][op] = define_model_2(op)
	model_dict[3][op] = define_model_3(op)

confusion_matrix = {}
prob_confusion_matrix = {}
for op in operations:
	confusion_matrix[op] = {}
	prob_confusion_matrix[op] = {}
	for op2 in operations:
		confusion_matrix[op][op2] = 0
		prob_confusion_matrix[op][op2] = 0

	path_demos = path + op + "/demos/"
	demo_index = os.listdir(path_demos)
	for obs in demo_index:
		#Create model of op without demo
		model_dict[1]['excluded'] = define_model_1(op, exclude_obs = obs) #The model of the operation of the observed demo is calculated again excluding that observation
		model_dict[2]['excluded'] = define_model_2(op, exclude_obs = obs)
		model_dict[3]['excluded'] = define_model_3(op, exclude_obs = obs)
		#Compare op/demo with all the models and print results
		result_op, results = compare_model(op, obs, model_dict)
		#Fill confusion matrix
		confusion_matrix[op][result_op] += 1
		for prob_op in results:
			prob_confusion_matrix[op][prob_op] += results[prob_op]

#Convert confusion matrix in %
confusion_matrix_perc = {}
prob_confusion_matrix_perc = {}
for model_op in confusion_matrix:
	confusion_matrix_perc[model_op] = {}
	prob_confusion_matrix_perc[model_op] = {}
	line_sum = 0
	prob_line_sum = 0
	for obs_op in confusion_matrix[model_op]:
		line_sum += confusion_matrix[model_op][obs_op]
		prob_line_sum += prob_confusion_matrix[model_op][obs_op]
	for obs_op in confusion_matrix[model_op]:
		confusion_matrix_perc[model_op][obs_op] = float(confusion_matrix[model_op][obs_op])/float(line_sum)
		prob_confusion_matrix_perc[model_op][obs_op] = float(prob_confusion_matrix[model_op][obs_op])/float(prob_line_sum)
	
#Print confusion matrix
print_confusion_matrix(confusion_matrix, False)
print("\n")
save_confusion_matrix(confusion_matrix_perc)
print_confusion_matrix(confusion_matrix_perc, True)
print("\n")
print_confusion_matrix(prob_confusion_matrix_perc, True)


