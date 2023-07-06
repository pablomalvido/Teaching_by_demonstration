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
path2 = os.path.join(os.path.dirname(__file__), '../files/bottle/pour_water/2/') #observation
model_dict = {}
prob = {}
avg_prob = {}


operations = os.listdir(path)
for operation in operations:
  if operation == "grasp" or operation == "grasp_top" or operation == "pour_water":
	model_dict[operation] = {}
	prob[operation] = {}
	path_operation = path + operation + "/models/"
	models = os.listdir(path_operation)
	for model in models:
		path_model = path_operation + model
		model_dict[operation][model[:-4]] = {}
		f = open(path_model, "r")
		prev_line = ""
		model_dict[operation][model[:-4]]['start'] = []
		model_dict[operation][model[:-4]]['trans'] = []
		model_dict[operation][model[:-4]]['map'] = {}
		#Create the models
		for line in f:
			line = line.replace("\n", "")
			if line == "Start:" or line == "Transition:" or line == "Map:":
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
				map_elem = line.split(":")
				key_elem = map_elem[0]
				value_elem = int(map_elem[1])
				model_dict[operation][model[:-4]]['map'][key_elem] = value_elem
		f.close()
		
		#Compare observation files with the models		
		path_obs_file = path2 + str(model)
		f = open(path_obs_file, "r")
		first_line = True
		prev_map_line = 0
		prob[operation][model[:-4]] = 1
		for line in f:
			line = line.replace("\n", "")
			line = line.replace(" ", "")
			if line in model_dict[operation][model[:-4]]['map'].keys():
				map_line = model_dict[operation][model[:-4]]['map'][line]
				if first_line:
					prob[operation][model[:-4]] = model_dict[operation][model[:-4]]['start'][map_line]/max(model_dict[operation][model[:-4]]['start'])
					first_line = False
				else:
					prob[operation][model[:-4]] *= model_dict[operation][model[:-4]]['trans'][map_line][prev_map_line]/max(model_dict[operation][model[:-4]]['trans'][map_line])
			elif line == "":
				continue
			else:
				prob[operation][model[:-4]] = 0
				break
			prev_map_line = map_line

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
  if operation == "grasp" or operation == "grasp_top" or operation == "pour_water":
	if avg_prob[operation] > max_prob:
		max_prob_op = operation
		max_prob = avg_prob[operation]

print("The observed operation is: " + max_prob_op + " with a " + str((max_prob * 100)) + "%")

		
