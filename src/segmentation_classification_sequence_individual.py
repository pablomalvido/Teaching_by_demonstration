#!/usr/bin/env python

#Import libraries
import rospy
import PyKDL 
import numpy as np
import math 
import time
import os
import copy
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from std_msgs.msg import Float32
from teaching_pkg.msg import *
from os.path import abspath
from PyQt5 import QtWidgets, QtCore
from pyqtgraph import PlotWidget, plot
import pyqtgraph as pg
from elvez_pkg.msg import *
from elvez_pkg.srv import *


#ROS init
rospy.init_node('segment_classify')


#Variables
object_name = rospy.get_param('~object', "")
demo_name = rospy.get_param('~demo_name', "")
path_object = os.path.join(os.path.dirname(__file__), '../files/' + object_name + '/') #compare with the models of this object
#seq_name = "5_pick_place_top" #8
seq_name = "26_grasp_pourwater_place" #3
#seq_name = "17_grasp_top_"
#file_names = ["hand_xyz.txt", "hand_rotation.txt", "demonstration_complete.txt", "demonstration.txt", "hand_orientation_Z.txt", "z.txt"]
file_names = ["hand_xyz.txt", "hand_rotation.txt", "demonstration_complete.txt", "demonstration.txt", "hand_orientation_Z.txt", "z.txt", "finger_bending.txt", "finger_pressure.txt"]
#file_names = ["hand_rotation.txt", "demonstration_complete.txt", "demonstration.txt", "hand_orientation_Z.txt"]
#file_names = ["demonstration_complete.txt", "demonstration.txt", "hand_orientation_Z.txt"]
#models_info = {"hand_xyz.txt": {"start": 2, "trans": 2, "weight": 0.5}, "hand_rotation.txt": {"start": 2, "trans": 2, "weight": 1}, "demonstration_complete.txt": {"start": 3, "trans": 2, "weight": 8}, "demonstration.txt": {"start": 3, "trans": 2, "weight": 2}, "hand_orientation_Z.txt": {"start": 1, "trans": 1, "weight": 2.5}, "z.txt": {"start": 2, "trans": 2, "weight": 0.8}}
#models_info = {"hand_xyz.txt": {"start": 2, "trans": 2, "weight": 1}, "hand_rotation.txt": {"start": 2, "trans": 2, "weight": 1}, "demonstration_complete.txt": {"start": 3, "trans": 2, "weight": 1}, "demonstration.txt": {"start": 3, "trans": 2, "weight": 1}, "hand_orientation_Z.txt": {"start": 1, "trans": 1, "weight": 3}, "z.txt": {"start": 2, "trans": 2, "weight": 1}}
models_info = {"hand_xyz.txt": {"start": 2, "trans": 2, "weight": 1}, "hand_rotation.txt": {"start": 2, "trans": 2, "weight": 1}, "demonstration_complete.txt": {"start": 3, "trans": 2, "weight": 1}, "demonstration.txt": {"start": 3, "trans": 2, "weight": 1}, "hand_orientation_Z.txt": {"start": 1, "trans": 1, "weight": 1}, "z.txt": {"start": 2, "trans": 2, "weight": 1}, "finger_bending.txt": {"start": 2, "trans": 2, "weight": 1}, "finger_pressure.txt": {"start": 2, "trans": 2, "weight": 1}}
order = 10 #8

if object_name =="bottle":
	file_names = ["hand_xyz.txt", "hand_rotation.txt", "demonstration_complete.txt", "demonstration.txt", "hand_orientation_Z.txt", "z.txt"]
	models_info = {"hand_xyz.txt": {"start": 2, "trans": 2, "weight": 1}, "hand_rotation.txt": {"start": 2, "trans": 2, "weight": 1}, "demonstration_complete.txt": {"start": 3, "trans": 2, "weight": 3}, "demonstration.txt": {"start": 3, "trans": 2, "weight": 1}, "hand_orientation_Z.txt": {"start": 1, "trans": 1, "weight": 1}, "z.txt": {"start": 2, "trans": 2, "weight": 1}}

perfect_segm = 0
recognized_op_100 = 0
recognized_op_50 = 0
recognized_op_1 = 0
wrong_op = 0
total_op = 0
process_detection = {}
process_classified_correctly = 0
loc_perfect_segm = 0
recognized_loc_100 = 0
recognized_loc_1 = 0
process_and_loc_classified_correctly = 0
total_location_seqs = 0

end = False
trans = False
seq_analysis = {}
complete_seq_analysis = {}
seq_analysis_process = []
conf_matrix = {}



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



def obtain_models_1(path, files_models):
  """
  Obtain the model considering prev state
  """
  model_dict = {}
  operations = os.listdir(path)
  for operation in operations:
    if operation != "sequence" and operation != "shake":
	model_dict[operation] = {}
	path_operation = path + operation + "/models/models1/"
	models = os.listdir(path_operation)
	for model in models:
		path_model = path_operation + model
		model_dict[operation][model[:-4]] = {}
		f = open(path_model, "r")
		prev_line = ""
		model_dict[operation][model[:-4]]['start'] = []
		model_dict[operation][model[:-4]]['trans'] = []
		model_dict[operation][model[:-4]]['map'] = {}
		#Read the models
		for line in f:
			line = line.replace("\n", "")
			if line == "Start:" or line == "Transition:" or line == "Map:" or line == "-" or line == "n_states:":
				prev_line = line
				continue
			elif line == "":
				break
			if prev_line == "Start:":
				start_elem = line.split(",")
				start_elem = map(float, start_elem[:-1])
				model_dict[operation][model[:-4]]['start'] = start_elem
			elif prev_line == "Transition:":
				trans_elem = line.split(",")
				trans_elem = map(float, trans_elem[:-1])
				model_dict[operation][model[:-4]]['trans'].append(trans_elem)
			elif prev_line == "Map:":
				""""Change to solve the :Fast problem in z.txt. Solve it properly in the future. Solve also the initial orientation lines"""
				key_elem_list = line.split(':')[:-1]
				aux_text = ""
				for elem in key_elem_list:
				    aux_text += elem
				key_elem = aux_text
				value_elem = int(line.split(':')[-1])
				model_dict[operation][model[:-4]]['map'][key_elem] = value_elem
			elif prev_line == "n_states:":
				n_states = float(line) #Round to the lowest
				n_states = int(n_states)
				model_dict[operation][model[:-4]]['n_states'] = n_states
		f.close()
  return model_dict



def obtain_models_2(path, files_models):
  """
  Obtain the model considering the 2 prev states
  """
  model_dict = {}
  operations = os.listdir(path)
  for operation in operations:
    if operation != "sequence" and operation != "shake":
	model_dict[operation] = {}
	path_operation = path + operation + "/models/models2/"
	models = os.listdir(path_operation)
	for model in models:
		path_model = path_operation + model
		model_dict[operation][model[:-4]] = {}
		f = open(path_model, "r")
		prev_line = ""
		second = 0
		model_dict[operation][model[:-4]]['start'] = []
		model_dict[operation][model[:-4]]['trans'] = []
		model_dict[operation][model[:-4]]['map'] = {}
		#Read the models
		for line in f:
			line = line.replace("\n", "")
			if line == "Start:" or line == "Transition:" or line == "Map:" or line == "-" or line == "n_states:":
				prev_line = line
				if line == "Transition:":
					model_dict[operation][model[:-4]]['trans'].append([])
					second = 0
				elif line == "-":
					model_dict[operation][model[:-4]]['trans'].append([])
					second += 1
				continue
			elif line == "":
				break
			if prev_line == "Start:":
				start_elem = line.split(",")
				start_elem = map(float, start_elem[:-1])
				model_dict[operation][model[:-4]]['start'].append(start_elem)
			elif prev_line == "Transition:" or prev_line == "-":
				trans_elem = line.split(",")
				trans_elem = map(float, trans_elem[:-1])
				model_dict[operation][model[:-4]]['trans'][second].append(trans_elem)
			elif prev_line == "Map:":
				""""Change to solve the :Fast problem in z.txt. Solve it properly in the future. Solve also the initial orientation lines"""
				key_elem_list = line.split(':')[:-1]
				aux_text = ""
				for elem in key_elem_list:
				    aux_text += elem
				key_elem = aux_text
				value_elem = int(line.split(':')[-1])
				model_dict[operation][model[:-4]]['map'][key_elem] = value_elem
			elif prev_line == "n_states:":
				n_states = float(line) #Round to the lowest
				n_states = int(n_states)
				model_dict[operation][model[:-4]]['n_states'] = n_states
		f.close()
  return model_dict



def obtain_models_3(path, files_models):
  """
  Obtain the model considering the 3 prev states
  """
  model_dict = {}
  operations = os.listdir(path)
  for operation in operations:
    if operation != "sequence" and operation != "shake":
	model_dict[operation] = {}
	path_operation = path + operation + "/models/models3/"
	models = os.listdir(path_operation)
	for model in models:
		path_model = path_operation + model
		model_dict[operation][model[:-4]] = {}
		f = open(path_model, "r")
		prev_line = ""
		second = 0
		third = 0
		model_dict[operation][model[:-4]]['start'] = []
		model_dict[operation][model[:-4]]['trans'] = []
		model_dict[operation][model[:-4]]['map'] = {}
		model_dict[operation][model[:-4]]['n_states'] = 0
		#Read the models
		for line in f:
			line = line.replace("\n", "")
			if line == "Start:" or line == "Transition:" or line == "Map:" or line == "-" or line == "--" or line == "n_states:":
				prev_line = line
				if line == "Transition:":
					model_dict[operation][model[:-4]]['trans'].append([])
					model_dict[operation][model[:-4]]['trans'][third].append([])
					third = 0
					second = 0
				elif line == "--":
					model_dict[operation][model[:-4]]['trans'].append([])
					third += 1
					model_dict[operation][model[:-4]]['trans'][third].append([])
					second = 0
				elif line == "-":
					model_dict[operation][model[:-4]]['trans'][third].append([])
					second += 1
				continue
			elif line == "":
				break

			if prev_line == "Start:":
				start_elem = line.split(",")
				start_elem = map(float, start_elem[:-1])
				model_dict[operation][model[:-4]]['start'].append(start_elem)
			elif prev_line == "Transition:" or prev_line == "-" or prev_line == "--":
				trans_elem = line.split(",")
				trans_elem = map(float, trans_elem[:-1])
				model_dict[operation][model[:-4]]['trans'][third][second].append(trans_elem)
			elif prev_line == "Map:":
				key_elem_list = line.split(':')[:-1]
				aux_text = ""
				for elem in key_elem_list:
				    aux_text += elem
				key_elem = aux_text
				value_elem = int(line.split(':')[-1])
				model_dict[operation][model[:-4]]['map'][key_elem] = value_elem
			elif prev_line == "n_states:":
				n_states = float(line) #Round to the lowest
				n_states = int(n_states)
				model_dict[operation][model[:-4]]['n_states'] = n_states
		f.close()
  return model_dict



def read_confusion_matrix(path):
	global conf_matrix

	object_name = path.split("/")[-2:-1][0]
	remove_item = len(object_name) + 1
	path_new = path[:-remove_item] + "/zz_other/confusion_matrix_weights/" + str(object_name) + ".txt"
	f = open(path_new, "r")
	
	first_line = True
	operation_names = []
	index_row = 0
	index_col = 0
	for line in f:
		line = line.replace("\n", "")
		line = line.replace("_", "")
		if first_line:
			first_line = False
			operation_names = line.split(",")
			for op in operation_names:
				conf_matrix[op] = {}
		else:
			weights = line.split(",")
			for w in weights:
				conf_matrix[operation_names[index_row]][operation_names[index_col]] = float(w)
				index_col += 1
			index_col = 0
			index_row += 1
	f.close()
					



def prob_seq_calc(models_1, models_2, models_3, path, files_models, models_info, seq_name):
  """
  Obtain the probabilities
  """
  #Initialization
  path_seq = path + "sequence/" + seq_name + "/"
  prob = {} #accumulated. For each op and txt
  prob_1 = {} #Probability of being the first state of an operation
  prob_2 = {} #Probability of each transition. For each op and txt
  times = {} #For each txt
  start_model_n = {}
  trans_model_n = {}
  count = {}
  first_line = {}
  second_line = {}
  third_line = {}
  prev_map_line1 = {}
  prev_map_line2 = {}
  prev_map_line3 = {}

  operations = os.listdir(path)
  for operation in operations:
	  if operation != "sequence" and operation != "shake":
		prob[operation] = {}
		prob_1[operation] = {}
		prob_2[operation] = {}
		start_model_n[operation] = {}
		trans_model_n[operation] = {}
		count[operation] = {}
		for txt_model in files_models:
			prob[operation][txt_model[:-4]] = []
			prob[operation][txt_model[:-4]].append(1) #Initial prob
			prob_1[operation][txt_model[:-4]] = []
			prob_1[operation][txt_model[:-4]].append(0) #Initial prob
			prob_2[operation][txt_model[:-4]] = []
			prob_2[operation][txt_model[:-4]].append(0) #Initial prob
			start_model_n[operation][txt_model[:-4]] = min(int(models_3[operation][txt_model[:-4]]['n_states']), int(models_info[txt_model]['start']))
			trans_model_n[operation][txt_model[:-4]] = min(int(models_3[operation][txt_model[:-4]]['n_states'])-1, int(models_info[txt_model]['trans']))
			count[operation][txt_model[:-4]] = 0

  for txt_model in files_models:
    path_seq_model = path_seq + txt_model
    f = open(path_seq_model, "r")
    times[txt_model[:-4]] = []
    times[txt_model[:-4]].append(0) #Initial time
    for operation in operations: #Initilization
	first_line[operation] = True
	second_line[operation] = False
	third_line[operation] = False
	prev_map_line3[operation] = 999
	prev_map_line2[operation] = 999
	prev_map_line1[operation] = 999

    for line in f:
	line = line.replace("\n", "")
	line = line.replace(" ", "")
	if line == "":
		continue
	times[txt_model[:-4]].append(float(line.split(":")[0]))
	line2 = line.split(":")[0]
	line = line.split(":")[1:]
	aux_text = ""
	for elem in line:
	    aux_text += elem
	line = aux_text

	for operation in operations:
		if operation != "sequence" and operation != "shake":
			prob_1_value = 0
			prob_2_value = 0
			prob_accum = 0
				
			#Found map in that operation
			if(line in models_3[operation][txt_model[:-4]]['map'].keys()):
				map_line = models_3[operation][txt_model[:-4]]['map'][line]
				#Determine prob_1. Prob of starting an operation			
				if start_model_n[operation][txt_model[:-4]] == 1:
					prob_1_value = models_3[operation][txt_model[:-4]]['start'][0][map_line]/max(models_3[operation][txt_model[:-4]]['start'][0])
				elif start_model_n[operation][txt_model[:-4]] == 2:
					if prev_map_line1[operation] != 999:
						prob_1_value_0 = models_3[operation][txt_model[:-4]]['start'][0][prev_map_line1[operation]]/max(models_3[operation][txt_model[:-4]]['start'][0])
					else:
						prob_1_value_0 = 0
					prob_1_value_1 = models_3[operation][txt_model[:-4]]['start'][1][map_line]/max(models_3[operation][txt_model[:-4]]['start'][1])
					prob_1_value = 0.65 * prob_1_value_0 + 0.35 * prob_1_value_1
				elif start_model_n[operation][txt_model[:-4]] == 3:
					if prev_map_line2[operation] != 999:
						prob_1_value_0 = models_3[operation][txt_model[:-4]]['start'][0][prev_map_line2[operation]]/max(models_3[operation][txt_model[:-4]]['start'][0])
					else:
						prob_1_value_0 = 0
					if prev_map_line1[operation] != 999:
						prob_1_value_1 = models_3[operation][txt_model[:-4]]['start'][1][prev_map_line1[operation]]/max(models_3[operation][txt_model[:-4]]['start'][1])
					else:
						prob_1_value_1 = 0
					prob_1_value_2 = models_3[operation][txt_model[:-4]]['start'][2][map_line]/max(models_3[operation][txt_model[:-4]]['start'][2])
					prob_1_value = 0.5 * prob_1_value_0 + 0.3 * prob_1_value_1 + 0.2 * prob_1_value_2

				#First line prob
				if first_line[operation]:
					if models_3[operation][txt_model[:-4]]['start'][0][map_line] > 0:
						prob_accum = models_3[operation][txt_model[:-4]]['start'][0][map_line]/max(models_3[operation][txt_model[:-4]]['start'][0])
						first_line[operation] = False
						second_line[operation] = True
						#prob_accum = 1 #Skip first line prob
						prob_2_value = models_3[operation][txt_model[:-4]]['start'][0][map_line]/max(models_3[operation][txt_model[:-4]]['start'][0])
					
				#Second line prob
				elif second_line[operation] and trans_model_n[operation][txt_model[:-4]] > 1:
					if models_3[operation][txt_model[:-4]]['start'][1][map_line] > 0:
						prob_accum = prob[operation][txt_model[:-4]][-1] * models_3[operation][txt_model[:-4]]['start'][1][map_line]/max(models_3[operation][txt_model[:-4]]['start'][1])
						second_line[operation] = False
						third_line[operation] = True
						#prob_accum = 1 #Skip first line prob
						prob_2_value = models_3[operation][txt_model[:-4]]['start'][1][map_line]/max(models_3[operation][txt_model[:-4]]['start'][1])
					
				#Third line prob
				elif third_line[operation] and trans_model_n[operation][txt_model[:-4]] > 2:
					if models_3[operation][txt_model[:-4]]['start'][2][map_line] > 0:
						prob_accum = prob[operation][txt_model[:-4]][-1] * models_3[operation][txt_model[:-4]]['start'][2][map_line]/max(models_3[operation][txt_model[:-4]]['start'][2])
						third_line[operation] = False
						#prob_accum = 1 #Skip first line prob
						prob_2_value = models_3[operation][txt_model[:-4]]['start'][2][map_line]/max(models_3[operation][txt_model[:-4]]['start'][2])
					
				#Transition prob
				else:
				    if trans_model_n[operation][txt_model[:-4]] == 0:
					prob_2_value = prob_1_value #Don't consider transitions, just the state itself
					prob_accum = prob[operation][txt_model[:-4]][-1] * prob_2_value
				    elif trans_model_n[operation][txt_model[:-4]] == 1 and prev_map_line1[operation] != 999:
					if models_1[operation][txt_model[:-4]]['trans'][prev_map_line1[operation]][map_line] > 0:
						prob_2_value = models_1[operation][txt_model[:-4]]['trans'][prev_map_line1[operation]][map_line]/max(models_1[operation][txt_model[:-4]]['trans'][prev_map_line1[operation]])
						prob_accum = prob[operation][txt_model[:-4]][-1] * prob_2_value
				    elif trans_model_n[operation][txt_model[:-4]] == 2 and prev_map_line1[operation] != 999 and prev_map_line2[operation] != 999:
					if models_2[operation][txt_model[:-4]]['trans'][prev_map_line2[operation]][prev_map_line1[operation]][map_line] > 0:
						prob_2_value = models_2[operation][txt_model[:-4]]['trans'][prev_map_line2[operation]][prev_map_line1[operation]][map_line]/max(models_2[operation][txt_model[:-4]]['trans'][prev_map_line2[operation]][prev_map_line1[operation]])
						prob_accum = prob[operation][txt_model[:-4]][-1] * prob_2_value
				    elif trans_model_n[operation][txt_model[:-4]] == 3 and prev_map_line1[operation] != 999 and prev_map_line2[operation] != 999 and prev_map_line3[operation] != 999:
					if models_3[operation][txt_model[:-4]]['trans'][prev_map_line3[operation]][prev_map_line2[operation]][prev_map_line1[operation]][map_line] > 0:
						prob_2_value = models_3[operation][txt_model[:-4]]['trans'][prev_map_line3[operation]][prev_map_line2[operation]][prev_map_line1[operation]][map_line]/max(models_3[operation][txt_model[:-4]]['trans'][prev_map_line3[operation]][prev_map_line2[operation]][prev_map_line1[operation]])
						prob_accum = prob[operation][txt_model[:-4]][-1] * prob_2_value
				
				#Update cycle
				prev_map_line3[operation] = prev_map_line2[operation]
				prev_map_line2[operation] = prev_map_line1[operation]
				prev_map_line1[operation] = map_line

			#No line. Skip
			elif line == "":
				continue

			#Line not found
			else:
				prev_map_line3[operation] = prev_map_line2[operation]
				prev_map_line2[operation] = prev_map_line1[operation]
				prev_map_line1[operation] = 999
				if second_line[operation]:
					third_line[operation] = True
					second_line[operation] = False
				if first_line[operation]:
					second_line[operation] = True
					first_line[operation] = False #If it doesn't enter in the first line
				else:
					third_line[operation] = False #If it doesn't enter in the second line

			#Update probabilities
			prob[operation][txt_model[:-4]].append(prob_accum)
			count[operation][txt_model[:-4]] += 1
			if start_model_n[operation][txt_model[:-4]] == 0:
				prob_1[operation][txt_model[:-4]].append(0)
			if start_model_n[operation][txt_model[:-4]] == 1:
				prob_1[operation][txt_model[:-4]].append(prob_1_value)
			elif start_model_n[operation][txt_model[:-4]] == 2 and count[operation][txt_model[:-4]] > 1:
				prob_1[operation][txt_model[:-4]].append(prob_1_value)
			elif start_model_n[operation][txt_model[:-4]] == 3 and count[operation][txt_model[:-4]] > 2:
				prob_1[operation][txt_model[:-4]].append(prob_1_value)
			prob_2[operation][txt_model[:-4]].append(prob_2_value)

  for txt_model in files_models:
	for operation in operations:
		if operation != "sequence" and operation != "shake":
			if start_model_n[operation][txt_model[:-4]] == 2:
				prob_1[operation][txt_model[:-4]].append(0)
			elif start_model_n[operation][txt_model[:-4]] == 3:
				prob_1[operation][txt_model[:-4]].append(0)
				prob_1[operation][txt_model[:-4]].append(0)
  return times, prob_1, prob_2



def prob_avg_calc(times, prob_start, prob_trans, path, files_models, models_info):
	"""
	Calculates the total probabilities considering all the models
	"""
	times_total = []
	models_weight_div = 0
	for txt_model in files_models:
		times_total += times[txt_model[:-4]]
		models_weight_div += models_info[txt_model]['weight']
	times_total = list(set(times_total)) #Remove duplicates
	#times_total.append(15)
	times_total.sort()

	prob_total_trans = {}
	prob_total_avg_trans = {}
	prob_total_start = {}
	prob_total_avg_start = {}
	operations = os.listdir(path)
	for operation in operations:
	  if operation != "sequence" and operation != "shake":
		prob_total_trans[operation] = [0]*len(times_total)
		prob_total_avg_trans[operation] = []
		prob_total_start[operation] = [0]*len(times_total)
		prob_total_avg_start[operation] = []

	index_total = 0
	for time_i in times_total:
		for txt_model in files_models:
			index_txt = 0
			for time_txt_i in times[txt_model[:-4]]:
				if time_txt_i >= time_i:
					break #Don't increase the index
				index_txt += 1
			if index_txt >= len(times[txt_model[:-4]]):
				index_txt -= 1
			#Once the index of the analyzed txt is determined, we add that value to all the operations
			for operation in operations:
			  if operation != "sequence" and operation != "shake":
				prob_total_trans[operation][index_total] += prob_trans[operation][txt_model[:-4]][index_txt] * models_info[txt_model]['weight'] #Add the prob of all models in that time instant
				prob_total_start[operation][index_total] += prob_start[operation][txt_model[:-4]][index_txt] * models_info[txt_model]['weight']
		index_total += 1

	for operation in operations:
	  if operation != "sequence" and operation != "shake":
		prob_total_avg_trans[operation] = [element / models_weight_div for element in prob_total_trans[operation]]
		prob_total_avg_start[operation] = [element / models_weight_div for element in prob_total_start[operation]]

	return times_total, prob_total_avg_start, prob_total_avg_trans



def segmentation_calc(times_total, prob_total_avg_start, prob_total_avg_trans, path, order, seq_name):
	"""
	Segmentation in the most probable sequence of operations
	"""
	global cad_components
	
	x_pop = {}
	y_pop_start = {}
	y_pop_trans = {}
	p_fit_start = {}
	p_fit_trans = {}
	max_y = {}
	operations = os.listdir(path)

	op_seq = seq_name.split("-")[0]
	cad_seq = ""
	if len(seq_name.split("-")) > 1:
		cad_seq = seq_name.split("-")[1]

	ok_order = False
	while not ok_order:
	  ok_order = True
	  for operation in operations:
		if operation != "sequence" and operation != "shake":
			#Regression curve
			max_y[operation] = []
			x_pop[operation] = []
			y_pop_start[operation] = []
			y_pop_trans[operation] = []
			p_fit_start[operation] = np.polyfit(times_total, prob_total_avg_start[operation], order)
			p_fit_trans[operation] = np.polyfit(times_total, prob_total_avg_trans[operation], order)
			length_x = 50
			step = max(times_total)/length_x
			for point in range(length_x + 1):
				current_x = float(point)*step
				x_pop[operation].append(current_x) #50 points from 0 to max_x
				current_y_start = 0
				current_y_trans = 0
				for n in range(order+1):
					current_y_start += p_fit_start[operation][n]*(current_x**(order-n))
					current_y_trans += p_fit_trans[operation][n]*(current_x**(order-n))
				y_pop_start[operation].append(current_y_start)
				y_pop_trans[operation].append(current_y_trans)
				if current_y_trans > 1.5 or current_y_start > 1.5 or current_y_trans < -0.1 or current_y_start < -0.1:
					if order > 1:
						ok_order = False
						order -= 1
						break
					else:
						print("Fit line error")

			index = 0
			for y in y_pop_start[operation]:
				if index != 0 and index != (len(y_pop_start[operation])-1):
					if y > y_pop_start[operation][index-1] and y >= y_pop_start[operation][index+1] and y > 0.2:
						max_y_dict = {'start_index': 0, 'avg_value': 0, 'time': 0}
						max_y_dict['start_index'] = index
						max_y[operation].append(max_y_dict)
				index += 1

	for operation in operations:
		if operation != "sequence" and operation != "shake":
			index = 0
			for elem in max_y[operation]:
				if elem['start_index'] < (len(y_pop_start[operation])-3):
					avg_op = (y_pop_start[operation][elem['start_index']] + y_pop_trans[operation][elem['start_index']] + y_pop_trans[operation][elem['start_index']+1] + y_pop_trans[operation][elem['start_index']+2])/4
					max_op = True
					for op2 in operations: #Comparison to see if it is the most probable operation
						if op2 != operation and op2 != "sequence" and op2 != "shake":
							avg_op2 = (y_pop_start[op2][elem['start_index']] + y_pop_trans[op2][elem['start_index']] + y_pop_trans[op2][elem['start_index']+1] + y_pop_trans[op2][elem['start_index']+2])/4
							if avg_op2 > avg_op:
								max_op = False
								break
					if max_op:
						max_y[operation][index]['avg_value'] = avg_op
						max_y[operation][index]['time'] = x_pop[operation][elem['start_index']]
				index += 1

	path_seq = path + "sequence/" + seq_name + "/"
	files_seq = os.listdir(path_seq)
	coord_list = []
	coord_bool = False
	if 'coordinates.txt' in files_seq:
		coord_bool = True
		path_coord = path_seq + 'coordinates.txt'
		f = open(path_coord, "r")
		for line in f:
			coord_dict = {}
			line = line.replace("\n", "")
			line = line.replace(" ", "")
			if line == "":
				continue
			xyz_coord = line.split(":")[1]							
			coord_dict['time'] = float(line.split(":")[0])
			coord_dict['x'] = float(xyz_coord.split(",")[0])
			coord_dict['y'] = float(xyz_coord.split(",")[1])
			coord_dict['z'] = float(xyz_coord.split(",")[2])
			if len(xyz_coord.split(",")) > 3 and False: #Never execute because of the False
				coord_dict['Rx'] = float(xyz_coord.split(",")[3])
				coord_dict['Ry'] = float(xyz_coord.split(",")[4])
				coord_dict['Rz'] = float(xyz_coord.split(",")[5])
				coord_dict['Rw'] = float(xyz_coord.split(",")[6])
			coord_list.append(coord_dict)

	operation_comp = "None"
	segmentation = []
	for operation in operations:
		if operation != "sequence" and operation != "shake":
			path_cali_tool = path + str(operation) + "/tool_calibration.txt"
			f = open(path_cali_tool, "r")
			for line in f:
				line = line.replace("\n", "")
				line = line.replace(" ", "")
				if line.split(":")[0] == 'x':
					tool_offset_x = float(line.split(":")[1])
				elif line.split(":")[0] == 'y':
					tool_offset_y = float(line.split(":")[1])
				elif line.split(":")[0] == 'z':
					tool_offset_z = float(line.split(":")[1])
			f.close()
			for elem in max_y[operation]:
				if elem['avg_value'] > 0:
					if coord_bool:
						coord_x = 0
						coord_y = 0
						coord_z = 0
						i = 0
						for coord_elem in coord_list:
							if coord_elem['time'] >= elem['time']:
								if i < 1: #Takes the coord of the future time steps, that is were the operatio is executed
									i+=1
									continue
								if len(coord_elem) == 4:
									#coord_x = coord_elem['x'] + tool_offset_x
									coord_x = coord_elem['x']
									#coord_y = coord_elem['y'] + tool_offset_y
									coord_y = coord_elem['y']
									coord_z = coord_elem['z']
								else:
									hand_frame_coord = PyKDL.Frame()
									hand_frame_coord.p = PyKDL.Vector(coord_elem['x'], coord_elem['y'], coord_elem['z'])
					    				hand_frame_coord.M = PyKDL.Rotation.Quaternion(coord_elem['Rx'], coord_elem['Ry'], coord_elem['Rz'], coord_elem['Rw'])
									hand_tool_frame = PyKDL.Frame()
									hand_tool_frame.p = PyKDL.Vector(tool_offset_x, tool_offset_y, tool_offset_z)
									tool_coord = hand_frame_coord * hand_tool_frame
									coord_x = tool_coord.p.x()
									coord_y = tool_coord.p.y()
									coord_z = tool_coord.p.z()
								min_dist_comp = 99999999
								operation_comp = "None"
								if cad_seq != "":
								    for component in cad_components:
									dist_comp = math.sqrt((cad_components[component]['center'][0] - coord_x)**2 + (cad_components[component]['center'][1] - coord_y)**2)
									#if seq_name[:2]=='28':
									#	print(str(component) + ": " + str(dist_comp))
									if dist_comp <= cad_components[component]['max_dist'] and dist_comp <= min_dist_comp:
										min_dist_comp = dist_comp
										operation_comp = component #Name of the component where the operation took place
								break
					else: 
						coord_x = 0
						coord_y = 0
						coord_z = 0
					segmentation.append({'operation': operation, 'start_index': elem['start_index'], 'avg_value': str(elem['avg_value']), 'time': str(elem['time']), 'x': coord_x, 'y': coord_y, 'z': coord_z, 'location': operation_comp})

	segmentation.sort(key=lambda x: x.get('start_index'))
	for elem in segmentation:
		#print(elem['time'] + ': ' + elem['operation'] + ": " + str(elem['x']) + ", " + str(elem['y']) + ", " + str(elem['z']) + ", location: " + str(elem['location']))
		#elem['time'] = float(elem['time'])*1.38
		#if elem['operation'] == 'tape':
		#	print("\t - cut, time: " +str(elem['time'])+"s, location: "+ str(elem['location']))
		#else:
		print("\t - " + elem['operation'] + ", time: " +str(elem['time'])+"s, location: "+ str(elem['location']))
		pass
	return segmentation, order



def recursive_seq_calc(seq, index, segm_len, max_sep):
    global end
    global trans

    seq_len = len(seq)
    #print(seq)
    if -index > seq_len-1:
        end = True
        
    elif seq[index]-seq[index-1] < max_sep and (seq[-1] < segm_len or (seq[index] < (segm_len+index+1) and trans)):
        if trans:
            seq[index] += 1
            for i in range(-index-1):
                seq[index+i+1] = seq[index+i] + 1
            trans = False
        else:
            for i in range(-index):
                seq[index+i] += 1
                
    else:
        recursive_seq_calc(seq, index-1, segm_len, max_sep)



def initial_seq_calc(index, seq_len, segm_len, max_sep):
    global end 
    global trans
    global seq_analysis_process

    seq = []
    for i in range(seq_len):
        seq.append(index + i)

    while end == False:
        seq_append = copy.deepcopy(seq)
        seq_analysis_process.append(seq_append)
	"""
        text = ""
        for elem in seq:
            text += str(elem)
        print(text)
	"""
        if seq_len == 1:
            end = True
        elif seq[-1]-seq[-2] < max_sep and seq[-1] < segm_len:
            seq[-1] += 1
            trans = True
        elif seq_len>2:  
            recursive_seq_calc(seq, -2, segm_len, max_sep)
        else:
            end = True



def classification_calc(segmentation, process_list):
	global end
	global trans
	global seq_analysis
	global seq_analysis_process
	global complete_seq_analysis
	global conf_matrix

	seq_analysis = {}
	complete_seq_analysis = {}
	max_sep = 3 #(0 _ _ 3)

	operations_segm = []
	locations_segm = []
	if segmentation != []:
		for elem in segmentation:
			operations_segm.append(elem['operation'].replace('_',''))
			locations_segm.append(elem['location'])

	#print(operations_segm)
	#print(process_list)
	segm_len = len(operations_segm)

	for process in process_list: #process_list is a dict with all the registered processes in the DB of processes and their segmentation (all the possible combinations in which it can be classified)
	    final_seqs = []
	    final_seqs_removed = []
	    seq_len = len(process_list[process])
	    final_seqs.append(process_list[process]) #Sequence of operations to compare our segmentation with
	    final_seqs_removed.append(-1) #No indexes removed (-1)

	    if seq_len > 2:
		remove_indexes = []
		for i in range(seq_len-2):
			remove_indexes.append(i+1)
		
		for i in remove_indexes:
			seq_copy = copy.deepcopy(process_list[process])
			del seq_copy[i]
			final_seqs.append(seq_copy) #All the seqs that we are gonna compare our segmentation with for that process. The original and removing intermediate indexes from the sequence
			final_seqs_removed.append(i)

	    max_prob_temp = 0
	    max_seq = 99
	    seq_index = -1
	    for seq_i in final_seqs: #Comparison with all the seqs that represent this process
		seq_index += 1
		seq_len_i = len(seq_i)	

		end = False
		trans = False

		seq_analysis_process = []

		n_starts = min(max_sep*2 - 1, seq_len_i*2 - 1)
		for i in range(n_starts): #All possible indexes to start the combinations to compare
		    end = False
		    trans = False
		    initial_seq_calc(i-int(n_starts/2), seq_len_i, segm_len, max_sep) #Returns all the combinations to compare in the seq_analysis_process global variable

		#seq_analysis[op] = {}
		#seq_analysis[op]['seq'] = seq_analysis_process

		
		for seq_check in seq_analysis_process: #Compare each combination of the current process sequence with the process segmented
			prob_temp = []
			for i in range(seq_len_i):
				#prob of operations_segm[seq_check[i]] being process_list[process][i]
				if seq_check[i] < 0: #Outside left, adds a 0
					prob_temp.append(0)
				elif seq_check[i] >= segm_len: #Outside right, adds a 0
					prob_temp.append(0)
				else:
					if operations_segm[seq_check[i]] == seq_i[i]: #Correct operation
						prob_temp.append(1)
					else: #Wrong operation (Confusion matrix prob, better than 0.1)
						#weight_wrong = max(conf_matrix[seq_i[i]][operations_segm[seq_check[i]]], 0.01) #[the one we did][the one that was recognized]
						weight_wrong = conf_matrix[seq_i[i]][operations_segm[seq_check[i]]]
						weight_correct = conf_matrix[seq_i[i]][seq_i[i]]
						#weight_wrong = weight_wrong/weight_correct
						weight_wrong = weight_wrong
						#weight_wrong = 0.1
						prob_temp.append(weight_wrong)
			for i in range(segm_len):
				if not i in seq_check: #Inside not considered
					prob_temp.append(0)

			if seq_len_i < seq_len: #Removing operation in the middle in the sequence
				prob_temp.append(0)

			prob_temp_avg = (sum(prob_temp)/float(len(prob_temp)))
			if prob_temp_avg > max_prob_temp:
				max_prob_temp = prob_temp_avg
				max_seq = seq_check
				if seq_len_i < seq_len: #One of the middle operations was removed
					copy_seq = copy.deepcopy(seq_check)
					copy_seq.insert(final_seqs_removed[seq_index], 99)
					index_temp = 0
					for elem_seq in copy_seq:
						if index_temp > final_seqs_removed[seq_index]:
							copy_seq[index_temp] += 1
						index_temp += 1
					max_seq = copy_seq

	    seq_analysis[process] = max_prob_temp
	    complete_seq_analysis[process] = {}
	    complete_seq_analysis[process]['prob'] = max_prob_temp
	    complete_seq_analysis[process]['seq'] = max_seq

	#print(complete_seq_analysis)
	max_process = max(seq_analysis, key = seq_analysis.get)
	if seq_analysis[max_process] <= 0.3:
		seq_analysis["None"] = seq_analysis[max_process]
		max_process = "None"

	cad_classified = []
	cad_text = ""
	if max_process != "None":
	  for index in complete_seq_analysis[max_process]['seq']:
		if index < 0 or index == 99 or index > (len(locations_segm)-1):
			cad_classified.append("Unkown")
			cad_text += "_Unkown"
		else:
			cad_classified.append(locations_segm[index])
			cad_text += "_" + str(locations_segm[index])

	return max_process, seq_analysis[max_process], cad_classified, cad_text[1:]
	


def summary_calc(segmentation, process_classified, seq_name, cad_classified):
	global perfect_segm
	global recognized_op_100
	global recognized_op_50
	global recognized_op_1
	global wrong_op
	global total_op
	global process_detection
	global process_classified_correctly
	global loc_perfect_segm
	global recognized_loc_100
	global recognized_loc_1
	global process_and_loc_classified_correctly
	global total_location_seqs
	recognized_op_count = 0
	recognized_loc_count = 0

	op_seq = seq_name.split("-")[0]
	operations_segm = []
	operations_seq = op_seq.split('_')[1:]

	cad_seq = ""
	locations_seq = []
	locations_segm = []
	location_included = False
	if len(seq_name.split("-")) > 1:
		cad_seq = seq_name.split("-")[1]
		locations_seq = cad_seq.split('_')
		location_included = True
		total_location_seqs += 1

	process_name = ""
	for elem in operations_seq:
		process_name += ("_" + elem)
	process_name = process_name[1:]

	if len(process_detection) > 0:
		if not(process_name in process_detection): #If it didn't exist yet
			process_detection[process_name] = {'perfect': 0, 'amount': 0, 'correct': 0}
	else:
		process_detection[process_name] = {'perfect': 0, 'amount': 0, 'correct': 0}
	process_detection[process_name]['amount'] += 1

	if process_name == process_classified:
		process_classified_correctly += 1
		process_detection[process_name]['correct'] += 1
		if location_included and locations_seq == cad_classified:
			process_and_loc_classified_correctly += 1	
	
	if segmentation != []:
		"""#To avoid repeated continuous operations
		for elem in segmentation:
			if len(operations_segm) > 0:
				if elem['operation'].replace('_','') != operations_segm[-1]:
					operations_segm.append(elem['operation'].replace('_',''))
			else:
				operations_segm.append(elem['operation'].replace('_',''))
		"""
		for elem in segmentation:
			operations_segm.append(elem['operation'].replace('_',''))
			if location_included:
				locations_segm.append(elem['location'])
		#print(operations_segm)

		if operations_seq == operations_segm:
			perfect_segm += 1
			process_detection[process_name]['perfect'] += 1

		for op in operations_seq:
			if op in operations_segm:
				recognized_op_count += 1

		if recognized_op_count == len(operations_seq):
			recognized_op_100 += 1

		if recognized_op_count >= (len(operations_seq))/2:
			recognized_op_50 += 1

		if recognized_op_count >= 1:
			recognized_op_1 += 1


		if location_included:
			if locations_seq == locations_segm:
				loc_perfect_segm += 1

			for loc in locations_seq:
				if loc in locations_segm:
					recognized_loc_count += 1

			if recognized_loc_count >= 1:
				recognized_loc_1 += 1

			if recognized_loc_count == len(locations_seq):
				recognized_loc_100 += 1


		index = 0
		for op in operations_seq:
			if len(operations_segm) > index:
				if op != operations_segm[index]:
					wrong_op += 1
			index += 1
		if len(operations_segm) > len(operations_seq):
			wrong_op += (len(operations_segm) - len(operations_seq))

		total_op += len(operations_seq)


def save_data_txt(x, y_start, y_trans, graph_name, path, order):
	path_graph = os.path.join(os.path.dirname(__file__), '../files/zz_other/graphs/graph_segmentation'+str(graph_name)+'.txt')
	f = open(path_graph, "w")
	f.write("times\n")
	f.write(str(x)[1:-1]+"\n")
	operations = os.listdir(path)

	f.write("start_prob\n")
	for operation in operations:
		if operation != "sequence" and operation != "shake":
			y = y_start[operation]
			f.write(str(operation)+"\n")
			f.write("probs\n")
			f.write(str(y)[1:-1]+"\n")
			#Regression curve
			p_fit = np.polyfit(x,y,order)
			x_pop=[]
			y_pop=[]
			length_x = 50
			step = max(x)/length_x
			for point in range(length_x + 1):
				current_x = float(point)*step
				x_pop.append(current_x) #50 points from 0 to max_x
				current_y = 0
				for n in range(order+1):
					current_y += p_fit[n]*(current_x**(order-n))
				y_pop.append(current_y)
			f.write("times_polyfit\n")
			f.write(str(x_pop)[1:-1]+"\n")
			f.write("prob_polyfit\n")
			f.write(str(y_pop)[1:-1]+"\n")

	f.write("trans_prob\n")
	for operation in operations:
		if operation != "sequence" and operation != "shake":
			y = y_trans[operation]
			f.write(str(operation)+"\n")
			f.write("probs\n")
			f.write(str(y)[1:-1]+"\n")
			#Regression curve
			p_fit = np.polyfit(x,y,order)
			x_pop=[]
			y_pop=[]
			length_x = 50
			step = max(x)/length_x
			for point in range(length_x + 1):
				current_x = float(point)*step
				x_pop.append(current_x) #50 points from 0 to max_x
				current_y = 0
				for n in range(order+1):
					current_y += p_fit[n]*(current_x**(order-n))
				y_pop.append(current_y)
			f.write("times_polyfit\n")
			f.write(str(x_pop)[1:-1]+"\n")
			f.write("prob_polyfit\n")
			f.write(str(y_pop)[1:-1]+"\n")

	f.close()


			
class MainWindow(QtWidgets.QMainWindow):
    """
    Class of the window with the graph for plotting the probabilities
    """

    def __init__(self, x_data, y_data, title, path, order, *args, **kwargs):
	"""
	Constructor. Graph initialization
	"""
	global tactile_data
        super(MainWindow, self).__init__(*args, **kwargs)
        self.graphWidget = pg.PlotWidget()
	#self.graphWidget.setLimits(yMin=0, yMax=3.2)
        self.setCentralWidget(self.graphWidget)
	self.graphWidget.setBackground('w')
	self.graphWidget.showGrid(x=False, y=False)
	#self.graphWidget.setXRange(0, 3.2, padding=0)
        #self.graphWidget.setYRange(0, 3.5, padding=0)
	self._legend = self.graphWidget.addLegend(offset=[500,10])
	self._legend.setLabelTextSize('12pt')
	self._legend.setLabelTextColor([0, 0, 0, 1.0])
	self.setWindowTitle(title)

	self.data_line = {}
	self.data_line_fit = {}
	self.horiz_line = {}
	self.horiz_line2 = {}
        self.pen = {}
        self.pen2 = {}
	self.pen_dash = {}
	x = x_data

	operations = os.listdir(path)
	colors_inc = 255*3/(len(operations)+1) #Should be -1 bc of 'sequence'
	color_op = []
	color_accum = 0
	for i in range(len(operations)+1):
	    color = [0,0,0]
	    color_accum += colors_inc
	    if color_accum > 255:
		color[0] = 255
		if color_accum > (255*2):
		    color[1] = 255
		    color[2] = color_accum - (255*2)
		else:
		    color[1] = color_accum - 255
		    color[2] = 0
	    else:
		color[0] = color_accum
		color[1] = 0
		color[2] = 0
	    color_op.append(color)

	#Just for the paper figure colors
	"""color_op = []
	color_op.append([126,122,246])
	color_op.append([249, 16, 16])
	color_op.append([116, 249, 25])
	color_op.append([9, 5, 132])
	color_op.append([132, 5, 5])
	color_op.append([5, 132, 14])"""

	offset = len(operations)-2 #Should be -1
	i = 0
	for operation in operations:
		if operation != "sequence" and operation != "shake":
			y = [element + offset for element in y_data[operation]]
			y_horiz = [offset] * len(y)
			y_horiz2 = [offset + 0.5] * len(y)
			offset -= 1
			self.pen[operation] = pg.mkPen(color=color_op[i], width=2)
			#self.pen2[operation] = pg.mkPen(color=color_op[i+3], width=2)
			self.pen_dash[operation] = pg.mkPen(color=color_op[i], style=QtCore.Qt.DashLine, width=1)
			self.data_line[operation] =  self.graphWidget.plot(x, y, pen=self.pen[operation], name=operation)
			self.horiz_line[operation] =  self.graphWidget.plot(x, y_horiz, pen=self.pen[operation])
			self.horiz_line2[operation] =  self.graphWidget.plot(x, y_horiz2, pen=self.pen_dash[operation])
			
			#Regression curve
			p_fit = np.polyfit(x,y,order)
			x_pop=[]
			y_pop=[]
			length_x = 50
			step = max(x)/length_x
			for point in range(length_x + 1):
				current_x = float(point)*step
				x_pop.append(current_x) #50 points from 0 to max_x
				current_y = 0
				for n in range(order+1):
					current_y += p_fit[n]*(current_x**(order-n))
				y_pop.append(current_y)
			#self.data_line_fit[operation] =  self.graphWidget.plot(x_pop, y_pop, pen=self.pen2[operation])
			self.data_line_fit[operation] =  self.graphWidget.plot(x_pop, y_pop, pen=self.pen[operation])
			i += 1


define_models_weights()
models_1 = obtain_models_1(path_object, file_names)
models_2 = obtain_models_2(path_object, file_names) 
models_3 = obtain_models_3(path_object, file_names)

cad_components = {}
rospy.wait_for_service('/ELVEZ_platform_handler/all_cad_components')
my_service = rospy.ServiceProxy('/ELVEZ_platform_handler/all_cad_components', all_cad_components)
all_cadReq = all_cad_componentsRequest()
all_cadResult = my_service(all_cadReq)
for cad_comp in all_cadResult.data:
	if cad_comp[0] is 'B':
		rospy.wait_for_service('/ELVEZ_platform_handler/tray_info')
		my_service = rospy.ServiceProxy('/ELVEZ_platform_handler/tray_info', tray_info)
		trayReq = tray_infoRequest()
		trayReq.box = cad_comp.split(',')[0]
		trayReq.tray = cad_comp.split(',')[2]
		trayResult = my_service(trayReq)
		center_xy = [trayResult.key_center_frame.position.x, trayResult.key_center_frame.position.y]
		max_dist = math.sqrt(trayResult.dimensions[0]**2 + trayResult.dimensions[1]**2)*1.25 #A 25% of margin
		cad_components[str(trayReq.box) + "." + str(trayReq.tray)] = {'center': center_xy, 'max_dist': max_dist}
		

path_seq = path_object + "sequence/"
sequences_list = os.listdir(path_seq)

if "seq_groups" in sequences_list:
	sequences_list.remove("seq_groups")

process_list = {}
for seq_name in sequences_list: #Get the names of all the different processes
	op_seq = seq_name.split("-")[0]
	cad_seq = ""
	if len(seq_name.split("-")) > 1:
		cad_seq = seq_name.split("-")[1]

	operations_seq = op_seq.split('_')[1:]
	process_name = ""
	for elem in operations_seq:
		process_name += ("_" + elem)
	process_name = process_name[1:]
	if len(process_list) > 0:
		if not(process_name in process_list):
			process_list[process_name] = operations_seq
	else:
		process_list[process_name] = operations_seq
#print(process_list)

read_confusion_matrix(path_object)

print("")
#for seq_name in sequences_list:
if True:
	#print(seq_name)
	print("SEGMENTATION:")
	seq_name = demo_name

	times, prob_start, prob_trans = prob_seq_calc(models_1, models_2, models_3, path_object, file_names, models_info, seq_name)

	times_total, prob_total_avg_start, prob_total_avg_trans = prob_avg_calc(times, prob_start, prob_trans, path_object, file_names, models_info)

	segmentation, corrected_order = segmentation_calc(times_total, prob_total_avg_start, prob_total_avg_trans, path_object, order, seq_name)

	process_classified, process_prob, cad_classified, cad_text = classification_calc(segmentation, process_list)

	summary_calc(segmentation, process_classified, seq_name, cad_classified)

	print("")
	#process_classified="grasp_cut_place"
	print("CLASSIFIED AS: " + process_classified + " (" + str(process_prob*100) + "%) in: " + cad_text)
	
	print("")

"""
perfect_segm_perc = float(perfect_segm)/(float(len(sequences_list)))*100
recognized_op_100_perc = float(recognized_op_100)/(float(len(sequences_list)))*100
recognized_op_50_perc = float(recognized_op_50)/(float(len(sequences_list)))*100
recognized_op_1_perc = float(recognized_op_1)/(float(len(sequences_list)))*100
wrong_op_perc = (float(wrong_op)/float(total_op))*100
process_classified_correctly_perc = float(process_classified_correctly)/(float(len(sequences_list)))*100
performance = (2.5 * perfect_segm_perc + 2 * recognized_op_100_perc + 1.5 * recognized_op_50_perc + recognized_op_1_perc - 2 * wrong_op_perc)/7
loc_perfect_segm_perc = float(loc_perfect_segm)/(float(total_location_seqs))*100
recognized_loc_100_perc = float(recognized_loc_100)/(float(total_location_seqs))*100
recognized_loc_1_perc = float(recognized_loc_1)/(float(total_location_seqs))*100
process_and_loc_classified_correctly_perc = float(process_and_loc_classified_correctly)/(float(total_location_seqs))*100

print("---------------------------------------------------------------------")
print("Processes segmentation performance:")
print("Sequences perfectly recognized: " + str(perfect_segm) + "/" + str(len(sequences_list)) + ", " + str(perfect_segm_perc) + "%")
print("Sequences with all operations recognized: " + str(recognized_op_100) + "/" + str(len(sequences_list)) + ", " + str(recognized_op_100_perc) + "%")
print("Sequences with at least 50% operations recognized: " + str(recognized_op_50) + "/" + str(len(sequences_list)) + ", " + str(recognized_op_50_perc) + "%")
print("Sequences with at least 1 operation recognized: " + str(recognized_op_1) + "/" + str(len(sequences_list)) + ", " + str(recognized_op_1_perc) + "%")
print("Wrong predictions: " + str(wrong_op) + "/" + str(total_op) + ", " + str(wrong_op_perc) + "%")
print("Performance: " + str(performance) + "%")
print("\n")

print("Locations recognition performance:")
print("Location sequences perfectly recognized: " + str(loc_perfect_segm) + "/" + str(total_location_seqs) + ", " + str(loc_perfect_segm_perc) + "%")
print("Sequences with all locations recognized: " + str(recognized_loc_100) + "/" + str(total_location_seqs) + ", " + str(recognized_loc_100_perc) + "%")
print("Sequences with at least 1 location recognized: " + str(recognized_loc_1) + "/" + str(total_location_seqs) + ", " + str(recognized_loc_1_perc) + "%")
print("\n")

print("Classification performance:")
print("Processes classified correctly without location: " + str(process_classified_correctly) + "/" + str(len(sequences_list)) + ", " + str(process_classified_correctly_perc) + "%")
print("Processes classified correctly with location: " + str(process_and_loc_classified_correctly) + "/" + str(total_location_seqs) + ", " + str(process_and_loc_classified_correctly_perc) + "%")
print("\n")

print("Processes classification performance:")
for process in process_detection:
	print(process + " perfectly segmented: " + str(process_detection[process]['perfect']) + "/" + str(process_detection[process]['amount']) + ", " + str(float(process_detection[process]['perfect'])/float(process_detection[process]['amount'])*100) + "% // correctly classified: " + str(process_detection[process]['correct']) + "/" + str(process_detection[process]['amount']) + ", " + str(float(process_detection[process]['correct'])/float(process_detection[process]['amount'])*100) + "%")
print("\n")
"""

#seq_name = "10_grasp_hammer_place-B1.1_B1.1_B1.1" #12,10,13,15,19,30
#seq_name = "47_grasp_pourwater_place-B1.1_B2.1_B2.1"
#seq_name="16_grasp_tape_place-B1.1_B1.1_B1.1"
#seq_name= "18_grasp_cut_place-B1.1_B2.1_B1.1" #18,1,2,1;23,1,3,4;17,1,1,1;7,1,2,1
#seq_name="33_grasptop_placetop-B1.1_B3.1" #32,1,3; 33,1,3; 34,3,2; 14,-,-
seq_name = demo_name

#times, prob_start, prob_trans = prob_seq_calc(models_1, models_2, models_3, path_object, file_names, models_info, seq_name)
#times_total, prob_total_avg_start, prob_total_avg_trans = prob_avg_calc(times, prob_start, prob_trans, path_object, file_names, models_info)
#segmentation, corrected_order = segmentation_calc(times_total, prob_total_avg_start, prob_total_avg_trans, path_object, order, seq_name)
#print(corrected_order)
#save_data_txt(x = times_total, y_start = prob_total_avg_start, y_trans = prob_total_avg_trans, graph_name = seq_name, path = path_object, order = corrected_order)
#app = QtWidgets.QApplication(sys.argv)
#w_start = MainWindow(x_data = times_total, y_data = prob_total_avg_start, title = seq_name + ": Starting probability", path = path_object, order = corrected_order)
#w_start.show()
#w_trans = MainWindow(x_data = times_total, y_data = prob_total_avg_trans, title = seq_name + ": Transition probability", path = path_object, order = corrected_order) #Initialization of the class of the graph window
#w_trans.show()
#sys.exit(app.exec_())

# Keep this process running until Enter is pressed
print ("Press Enter to quit...")
try:
	sys.stdin.readline()
except KeyboardInterrupt:
	pass
finally:
	pass
