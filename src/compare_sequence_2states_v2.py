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


#Variables
path_object = os.path.join(os.path.dirname(__file__), '../files/bottle/') #compare with the models of this object
#seq_name = "8_pick_place_top"
seq_name = "3_pick_pour_place"
file_names = ["hand_xyz.txt", "hand_rotation.txt", "demonstration_complete.txt", "demonstration.txt", "hand_orientation_Z.txt", "z.txt"]
#file_names = ["hand_rotation.txt", "demonstration_complete.txt", "demonstration.txt", "hand_orientation_Z.txt"]
#file_names = ["demonstration_complete.txt", "demonstration.txt", "hand_orientation_Z.txt"]
models_info = {"hand_xyz.txt": {"start": 2, "trans": 2, "weight": 0.5}, "hand_rotation.txt": {"start": 2, "trans": 2, "weight": 1}, "demonstration_complete.txt": {"start": 3, "trans": 2, "weight": 4}, "demonstration.txt": {"start": 3, "trans": 2, "weight": 2}, "hand_orientation_Z.txt": {"start": 1, "trans": 1, "weight": 1}, "z.txt": {"start": 2, "trans": 2, "weight": 0.5}}



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
			print(operation)
			print(line)
			if(line in models_3[operation][txt_model[:-4]]['map'].keys()):
				print(line)
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
					if models_3[operation][txt_model[:-4]]['start'][1][map_line] > 0:
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


			
class MainWindow(QtWidgets.QMainWindow):
    """
    Class of the window with the graph for plotting the probabilities
    """

    def __init__(self, x_data, y_data, title, path, *args, **kwargs):
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
	self.setWindowTitle(title)

	self.data_line = {}
	self.data_line_fit = {}
	self.horiz_line = {}
	self.horiz_line2 = {}
        self.pen = {}
	x = x_data

	operations = os.listdir(path)
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
			
			#Regression curve
			order = 10
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
			self.data_line_fit[operation] =  self.graphWidget.plot(x_pop, y_pop, pen=self.pen[operation])
			i += 1


models_1 = obtain_models_1(path_object, file_names)
models_2 = obtain_models_2(path_object, file_names) 
models_3 = obtain_models_3(path_object, file_names)

times, prob_start, prob_trans = prob_seq_calc(models_1, models_2, models_3, path_object, file_names, models_info, seq_name)

times_total, prob_total_avg_start, prob_total_avg_trans = prob_avg_calc(times, prob_start, prob_trans, path_object, file_names, models_info)
print(prob_start)
print(prob_trans)
app = QtWidgets.QApplication(sys.argv)
w_start = MainWindow(x_data = times_total, y_data = prob_total_avg_start, title = seq_name + ": Starting probability", path = path_object)
w_start.show()
w_trans = MainWindow(x_data = times_total, y_data = prob_total_avg_trans, title = seq_name + ": Transition probability", path = path_object) #Initialization of the class of the graph window
w_trans.show()
sys.exit(app.exec_())

# Keep this process running until Enter is pressed
print ("Press Enter to quit...")
try:
	sys.stdin.readline()
except KeyboardInterrupt:
	pass
finally:
	pass
