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
path2 = os.path.join(os.path.dirname(__file__), '../files/bottle/sequence/3_pick_pour_place/') #sequence
#path2 = os.path.join(os.path.dirname(__file__), '../files/bottle/sequence/6_pick_place_top/') #sequence
txt_model = "demonstration_complete.txt"
model_dict = {}
prob = {}
prob_2 = {}
avg_prob = {}
first_line = {}
prev_map_line = {}


#Obtain the model
operations = os.listdir(path)
for operation in operations:
  if operation != "sequence" and operation != "shake":
	model_dict[operation] = {}
	#Initializations for the next loop
	prob[operation] = []
	prob_2[operation] = []
	prob[operation].append(1) #Initial prob
	prob_2[operation].append(0) #Initial prob
	first_line[operation] = True
	prev_map_line[operation] = 0
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
				""""Change to solve the :Fast problem in z.txt. Solve it properly in the future. Solve also the initial orientation lines"""
				#map_elem = line.split(":")
				key_elem_list = line.split(':')[:-1]
				aux_text = ""
				for elem in key_elem_list:
				    aux_text += elem
				key_elem = aux_text
				#print(operation)
				#print(model)
				value_elem = int(line.split(':')[-1])
				model_dict[operation][model[:-4]]['map'][key_elem] = value_elem
		f.close()

#print(model_dict['grasp']['demonstration']['map'].keys())

path_seq = path2 + txt_model
f = open(path_seq, "r")
times = []
times.append(0) #Initial time

for line in f:
	line = line.replace("\n", "")
	line = line.replace(" ", "")
	times.append(float(line.split(":")[0]))
	line2 = line.split(":")[0]
	line = line.split(":")[1:]
	aux_text = ""
	for elem in line:
	    aux_text += elem
	line = aux_text
	for operation in operations:
		if operation != "sequence" and operation != "shake":
			#print(operation)
			#print(model_dict[operation][model[:-4]]['map'])
			if(line in model_dict[operation][txt_model[:-4]]['map'].keys()):
				map_line = model_dict[operation][txt_model[:-4]]['map'][line]
				print(map_line)
				if first_line[operation]:
					if model_dict[operation][txt_model[:-4]]['start'][map_line] > 0:
						prob_accum = model_dict[operation][txt_model[:-4]]['start'][map_line]/max(model_dict[operation][txt_model[:-4]]['start'])
						first_line[operation] = False
						#prob_accum = 1 #Skip first line prob
						prob_2[operation].append(model_dict[operation][txt_model[:-4]]['start'][map_line]/max(model_dict[operation][txt_model[:-4]]['start']))
					else:
						prob_accum = 0
						prob_2[operation].append(0)
				elif prev_map_line[operation] != 999:
					if model_dict[operation][txt_model[:-4]]['trans'][prev_map_line[operation]][map_line] > 0:
						prob_accum = prob[operation][-1] * model_dict[operation][txt_model[:-4]]['trans'][prev_map_line[operation]][map_line]/max(model_dict[operation][txt_model[:-4]]['trans'][prev_map_line[operation]])
						prob_2[operation].append(model_dict[operation][txt_model[:-4]]['trans'][prev_map_line[operation]][map_line]/max(model_dict[operation][txt_model[:-4]]['trans'][prev_map_line[operation]]))
					else:
						prob_accum = 0
						prob_2[operation].append(0)
				else:
					prob_accum = 0
					prob_2[operation].append(0)
				prev_map_line[operation] = map_line
			elif line == "":
				continue
			else:
				prob_accum = 0
				prev_map_line[operation] = 999
				first_line[operation] = False #If it doesn't enter in the first line
				prob_2[operation].append(0)

			prob[operation].append(prob_accum)


class MainWindow(QtWidgets.QMainWindow):
    """
    Class of the window with the graph for plotting the probabilities
    """

    def __init__(self, y_data, *args, **kwargs):
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

	self.data_line = {}
        self.pen = {}
	x = times
	#x.insert(0,0) #Initial time
	colors_inc = 255*3/(len(operations)-1) #-1 bc of 'sequence'
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
			offset -= 1
			self.pen[operation] = pg.mkPen(color=color_op[i])
			self.data_line[operation] =  self.graphWidget.plot(x, y, pen=self.pen[operation], name=operation)
			i += 1


print(prob)
app = QtWidgets.QApplication(sys.argv)
w_accum = MainWindow(y_data = prob) #Initialization of the class of the graph window
w_accum.show()
w_inst = MainWindow(y_data = prob_2)
w_inst.show()
sys.exit(app.exec_())

# Keep this process running until Enter is pressed
print ("Press Enter to quit...")
try:
	sys.stdin.readline()
except KeyboardInterrupt:
	pass
finally:
	pass
