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
from PyQt5 import QtWidgets, QtCore
from pyqtgraph import PlotWidget, plot
import pyqtgraph as pg


#ROS init
rospy.init_node('compare_sequence')
path = os.path.join(os.path.dirname(__file__), '../files/bottle/') #compare with the models of this object
path_seq = os.path.join(os.path.dirname(__file__), '../files/bottle/sequence/')
#sequence = "3_pick_pour_place"
sequence = "5_pick_place_top"
path2 = path_seq + sequence + "/" #sequence
#files_models = ["hand_xyz.txt", "hand_rotation.txt", "demonstration_complete.txt", "demonstration.txt", "hand_orientation_Z.txt", "z.txt"]
files_models = ["hand_rotation.txt"]
#txt_model = "demonstration_complete.txt"
model_dict = {}
prob = {} #accumulated. For each op and txt
prob_1 = {} #Probability of being the first state of an operation
prob_2 = {} #instant. For each op and txt
avg_prob = {}
first_line = {}
second_line = {}
prev_map_line1 = {}
prev_map_line2 = {}
prob_1_1_value = {}
times = {} #For each txt


#Obtain the model
operations = os.listdir(path)
for operation in operations:
  if operation != "sequence" and operation != "shake":
	model_dict[operation] = {}
	###Initializations for the prob loop
	prob[operation] = {}
	prob_1[operation] = {}
	prob_2[operation] = {}
	###
	path_operation = path + operation + "/models/models2/"
	models = os.listdir(path_operation)
	for model in models:
		###Initializations for the prob loop
		prob[operation][model[:-4]] = []
		prob[operation][model[:-4]].append(1) #Initial prob
		prob_1[operation][model[:-4]] = []
		prob_1[operation][model[:-4]].append(0) #Initial prob
		prob_2[operation][model[:-4]] = []
		prob_2[operation][model[:-4]].append(0) #Initial prob
		###
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
				""""Change to solve the :Fast problem in z.txt. Solve it properly in the future. Solve also the initial orientation lines"""
				key_elem_list = line.split(':')[:-1]
				aux_text = ""
				for elem in key_elem_list:
				    aux_text += elem
				key_elem = aux_text
				value_elem = int(line.split(':')[-1])
				model_dict[operation][model[:-4]]['map'][key_elem] = value_elem
		f.close()

print(model_dict['place']['demonstration_complete']['start'])
print(model_dict['place']['demonstration_complete']['map'])
print(model_dict['place']['demonstration_complete']['start'][0][1])
print(model_dict['place']['demonstration_complete']['start'][1][2])

for txt_model in files_models:
    path_seq = path2 + txt_model
    f = open(path_seq, "r")
    times[txt_model[:-4]] = []
    times[txt_model[:-4]].append(0) #Initial time
    for operation in operations: #Initilization
	first_line[operation] = True
	second_line[operation] = False
	prev_map_line2[operation] = 0
	prev_map_line1[operation] = 0
	prob_1_1_value[operation] = 0

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

			#Found map in that operation
			if(line in model_dict[operation][txt_model[:-4]]['map'].keys()):
				map_line = model_dict[operation][txt_model[:-4]]['map'][line]
				#Determine prob_1. Prob of starting an operation
				if max(model_dict[operation][txt_model[:-4]]['start'][1]) > 0:
					prob_1_2_value = model_dict[operation][txt_model[:-4]]['start'][1][map_line]/max(model_dict[operation][txt_model[:-4]]['start'][1])
				else:
					prob_1_2_value = 0
				prob_1_value = prob_1_1_value[operation] * prob_1_2_value
				if operation == "place":
					print("Map: " + str(map_line))
					print("First: " + str(prob_1_1_value[operation]))
					print("Second: " + str(prob_1_2_value))
				prob_1_1_value[operation] = model_dict[operation][txt_model[:-4]]['start'][0][map_line]/max(model_dict[operation][txt_model[:-4]]['start'][0])

				#First line prob
				if first_line[operation]:
					if model_dict[operation][txt_model[:-4]]['start'][0][map_line] > 0:
						prob_accum = model_dict[operation][txt_model[:-4]]['start'][0][map_line]/max(model_dict[operation][txt_model[:-4]]['start'][0])
						first_line[operation] = False
						second_line[operation] = True
						#prob_accum = 1 #Skip first line prob
						prob_2_value = model_dict[operation][txt_model[:-4]]['start'][0][map_line]/max(model_dict[operation][txt_model[:-4]]['start'][0])
					else:
						prob_accum = 0

				#Second line prob
				elif second_line:
					if model_dict[operation][txt_model[:-4]]['start'][1][map_line] > 0:
						prob_accum = prob[operation][txt_model[:-4]][-1] * model_dict[operation][txt_model[:-4]]['start'][1][map_line]/max(model_dict[operation][txt_model[:-4]]['start'][1])
						second_line[operation] = False
						#prob_accum = 1 #Skip first line prob
						prob_2_value = model_dict[operation][txt_model[:-4]]['start'][1][map_line]/max(model_dict[operation][txt_model[:-4]]['start'][1])
					else:
						prob_accum = 0

				#Transition prob
				elif prev_map_line[operation] != 999:
					if model_dict[operation][txt_model[:-4]]['trans'][prev_map_line2[operation]][prev_map_line1[operation]][map_line] > 0:
						prob_accum = prob[operation][txt_model[:-4]][-1] * model_dict[operation][txt_model[:-4]]['trans'][prev_map_line2[operation]][prev_map_line1[operation]][map_line]/max(model_dict[operation][txt_model[:-4]]['trans'][prev_map_line2[operation]][prev_map_line1[operation]])
						prob_2_value = model_dict[operation][txt_model[:-4]]['trans'][prev_map_line2[operation]][prev_map_line1[operation]][map_line]/max(model_dict[operation][txt_model[:-4]]['trans'][prev_map_line2[operation]][prev_map_line1[operation]])
					else:
						prob_accum = 0
				else:
					prob_accum = 0
				prev_map_line2[operation] = prev_map_line1[operation]
				prev_map_line1[operation] = map_line

			#No line. Skip
			elif line == "":
				continue

			#Line not found
			else:
				prob_accum = 0
				prev_map_line2[operation] = prev_map_line1[operation]
				prev_map_line1[operation] = 999
				prob_1_1_value[operation] = 0
				if first_line[operation]:
					second_line[operation] = True
					first_line[operation] = False #If it doesn't enter in the first line
				else:
					second_line[operation] = False #If it doesn't enter in the second line

			#Update probabilities
			prob[operation][txt_model[:-4]].append(prob_accum)
			prob_1[operation][txt_model[:-4]].append(prob_1_value)
			prob_2[operation][txt_model[:-4]].append(prob_2_value)

print(prob_2)

#Calculates the total probabilities considering all the models
times_total = []
for txt_model in files_models:
	times_total += times[txt_model[:-4]]
times_total = list(set(times_total)) #Remove duplicates
times_total.sort()

prob_total = {}
prob_total_avg = {}
prob_total_1 = {}
prob_total_avg_1 = {}
for operation in operations:
  if operation != "sequence" and operation != "shake":
	prob_total[operation] = [0]*len(times_total)
	prob_total_avg[operation] = []
	prob_total_1[operation] = [0]*len(times_total)
	prob_total_avg_1[operation] = []

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
			prob_total[operation][index_total] += prob_2[operation][txt_model[:-4]][index_txt] #Add the prob of all models in that time instant
			prob_total_1[operation][index_total] += prob_1[operation][txt_model[:-4]][index_txt]
	index_total += 1

for operation in operations:
  if operation != "sequence" and operation != "shake":
	prob_total_avg[operation] = [element / len(files_models) for element in prob_total[operation]]
	prob_total_avg_1[operation] = [element / len(files_models) for element in prob_total_1[operation]]

print(prob_total_avg)		
			
class MainWindow(QtWidgets.QMainWindow):
    """
    Class of the window with the graph for plotting the probabilities
    """

    def __init__(self, x_data, y_data, title, *args, **kwargs):
	"""
	Constructor. Graph initialization
	"""
	global tactile_data
        super(MainWindow, self).__init__(*args, **kwargs)
        self.graphWidget = pg.PlotWidget()
        self.setCentralWidget(self.graphWidget)
	self.graphWidget.setBackground('w')
	self.graphWidget.showGrid(x=False, y=False)
	#self.graphWidget.setXRange(0, 6, padding=0)
        #self.graphWidget.setYRange(0, 6, padding=0)
	self.graphWidget.addLegend(offset=[500,10])
	self.setWindowTitle(sequence + ": " + title)

	self.data_line = {}
	self.horiz_line = {}
	self.horiz_line2 = {}
        self.pen = {}
	x = x_data
	#x.insert(0,0) #Initial time
	colors_inc = 255*3/(len(operations)-0) #Should be -1 bc of 'sequence'
	color_op = []
	color_accum = 0
	for i in range(5):
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

	offset = len(operations)-2 #Should be -1
	i = 0
	for operation in operations:
		if operation != "sequence" and operation != "shake":
			y = [element + offset for element in y_data[operation]]
			y_horiz = [offset] * len(y)
			y_horiz2 = [offset + 0.5] * len(y)
			offset -= 1
			self.pen[operation] = pg.mkPen(color=color_op[i])
			self.data_line[operation] =  self.graphWidget.plot(x, y, pen=self.pen[operation], name=operation)
			self.horiz_line[operation] =  self.graphWidget.plot(x, y_horiz, pen=self.pen[operation])
			self.horiz_line2[operation] =  self.graphWidget.plot(x, y_horiz2, pen=self.pen[operation])
			i += 1


app = QtWidgets.QApplication(sys.argv)
w_1 = MainWindow(x_data = times_total, y_data = prob_total_avg, title = "Transition probability") #Initialization of the class of the graph window
w_1.show()
w_2 = MainWindow(x_data = times_total, y_data = prob_total_avg_1, title = "Starting probability")
w_2.show()
sys.exit(app.exec_())

# Keep this process running until Enter is pressed
print ("Press Enter to quit...")
try:
	sys.stdin.readline()
except KeyboardInterrupt:
	pass
finally:
	pass
