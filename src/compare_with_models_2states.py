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
rospy.init_node('compare_model')
path = os.path.join(os.path.dirname(__file__), '../files/bottle/') #compare with the models of this object
path2 = os.path.join(os.path.dirname(__file__), '../files/bottle/grasp/demos/2/') #observation
model_dict = {}
prob = {}
avg_prob = {}


operations = os.listdir(path)
for operation in operations:
  #if operation == "grasp" or operation == "grasp_top" or operation == "pour_water":
  if operation == "grasp":
	model_dict[operation] = {}
	prob[operation] = {}
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
		#Create the models
		for line in f:
			line = line.replace("\n", "")
			if line == "Start:" or line == "Transition:" or line == "Map:" or line == "-":
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
				map_elem = line.split(":")
				key_elem = map_elem[0]
				value_elem = int(map_elem[1])
				model_dict[operation][model[:-4]]['map'][key_elem] = value_elem
		f.close()
		
		
		#Compare observation files with the models		
		path_obs_file = path2 + str(model)
		f = open(path_obs_file, "r")
		first_line = True
		second_line = False
		prev_map_line2 = 0
		prev_map_line1 = 0
		prob[operation][model[:-4]] = 1
		for line in f:
			line = line.replace("\n", "")
			line = line.replace(" ", "")
			if line in model_dict[operation][model[:-4]]['map'].keys():
				map_line = model_dict[operation][model[:-4]]['map'][line]
				if first_line:
					if model_dict[operation][model[:-4]]['start'][0][map_line] != 0:
						prob[operation][model[:-4]] = model_dict[operation][model[:-4]]['start'][0][map_line]/max(model_dict[operation][model[:-4]]['start'][0])
						first_line = False
						second_line = True
					else:
						prob[operation][model[:-4]] = 0
						break
				elif second_line:
					if model_dict[operation][model[:-4]]['start'][1][map_line] != 0:
						prob[operation][model[:-4]] *= model_dict[operation][model[:-4]]['start'][1][map_line]/max(model_dict[operation][model[:-4]]['start'][1])
						second_line = False
					else:
						prob[operation][model[:-4]] = 0
						break
				else:
					if model_dict[operation][model[:-4]]['trans'][prev_map_line2][prev_map_line1][map_line] != 0:
						prob[operation][model[:-4]] *= model_dict[operation][model[:-4]]['trans'][prev_map_line2][prev_map_line1][map_line]/max(model_dict[operation][model[:-4]]['trans'][prev_map_line2][prev_map_line1])
					else:
						prob[operation][model[:-4]] = 0
						break
			elif line == "":
				continue
			else:
				prob[operation][model[:-4]] = 0
				break
			prev_map_line2 = prev_map_line1
			prev_map_line1 = map_line

	print(prob)
	avg_prob[operation] = 0
	for model in models:
		avg_prob[operation] += prob[operation][model[:-4]]
	avg_prob[operation] = avg_prob[operation]/len(prob[operation])

#print(model_dict) 
#print(prob)
print(avg_prob)

max_prob_op = ""
max_prob = 0
for operation in operations:
  #if operation == "grasp" or operation == "grasp_top" or operation == "pour_water":
  if operation == "grasp":
	if avg_prob[operation] > max_prob:
		max_prob_op = operation
		max_prob = avg_prob[operation]

print("The observed operation is: " + max_prob_op + " with a " + str((max_prob * 100)) + "%")
