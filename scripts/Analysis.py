#!/usr/bin/env python3
import math
import numpy as np
import matplotlib as plt
import csv
from numpy import genfromtxt
import sys


#----------------- Functionality Functions --------------------------
def frange(start, stop=None, step=None):
    if stop == None:
        stop = start + 0.0
        start = 0.0
    if step == None:
        step = 1.0
    while True:
        if step > 0 and start >= stop:
            break
        elif step < 0 and start <= stop:
            break
        yield ("%g" % start) # return float number
        start = start + step
    '''
	This function allows for float range functionality. 
	Inputs:
	start		starting number		type = float
	stop		stopping number		type = float
	step		how to increment	type = float

	Usage:
	if x in frange(float(), float())
		This will allow you to ask if float x is within range of two floats
	For i in frange(float(), float(), float())
		This will allow you to iterate through a float range and iterate by a float.
	'''
	
def progress(count, total, status=''):
    bar_len = 60
    filled_len = int(round(bar_len * count / float(total)))

    percents = round(100.0 * count / float(total), 1)
    bar = '#' * filled_len + '-' * (bar_len - filled_len)
    
    sys.stdout.write('[%s] %s%s ...%s\r' % (bar, percents, '%', status))
    sys.stdout.flush() 
    '''
	This function will print a progress bar in the terminal
	Usage (doesnt need to be while loop, just needs to be inside a loop):

		total = 1000
		i = 0
		while i < total:
    		progress(i, total, status='Doing very long job')
    		i += 1
	'''

# ------------------- Binning Functions ----------------------------
def MakeBins():
	''' Define Bin Limits'''
	arena_dim = (4.8,4.8)
	arena_min = -2.4
	arena_max = 2.4
	bins = 100
	arena_iter = arena_dim[0]/math.sqrt(bins)
	arena = np.zeros((10,10))
	x = np.zeros(11)
	y = np.zeros(11)
	for i in range(11):
		x[i] = i*0.48 - 2.4
		y[i] = i*0.48 - 2.4
	x_bin = np.ndarray((10,2))
	y_bin = np.ndarray((10,2))
	for i in range(10):
		x_bin[i] =(x[i],x[i+1])
		y_bin[i] =(y[i],y[i+1])
	bins = np.ndarray((100, 4))
	count = 0
	for i in range(10):
		for j in range(10):

			bins[count][0] = x_bin[i][0]
			bins[count][1] = x_bin[i][1]
			bins[count][2] = y_bin[j][0]
			bins[count][3] = y_bin[j][1]
			count+=1
	return (bins)

def SortBins(bins, data):
	'''Sort data pointers into specific bins'''
	#create and array of lists
	binned_data = np.empty((100,),dtype = object)
	for i,v in enumerate(binned_data):binned_data[i]=[v]

	counter = 0
	total_data = data.shape[0]
	total = total_data*bins.shape[0]
	#total_data = 5400
	#itterate through all 100 bins
	for i in range(bins.shape[0]):
		#iterate thgough all the rows of data
		for j in range(total_data):
			# show progress
			progress(counter, total, status = 'binning data')
			counter +=1
			# checking x ranges
			if bins[i][0] < data[j][2] and data[j][2] <= bins[i][1]:
				#checking y ranges
				if bins[i][2] < data[j][3] and data[j][3] <=bins[i][3]:
					#checking if there is anything in the list
					if binned_data[i][0] is None:
						binned_data[i][0] = j
					binned_data[i].append(j) 
	#return the array of lists of pointers to the place in the data 
	print('\n')
	return(binned_data)

def StateProb(binned_data, data):
	# Calcualtes the probability of the state of the agent 
	# ocupying bin i being 1. If you want the prob for the 
	# state being 0 then do 1-prob
	
	prob = np.empty((100), dtype = float)
	for i in range(binned_data.shape[0]):
		total = 0
		s = 0
		mark = False
		for j in range(len(binned_data[i])):
			if binned_data[i][j] is not None:
				total+=1
				p = binned_data[i][j]
				s += data[p][4]
				mark = True
		if mark:
			prob[i] = s/total
		else:
			prob[i] = 0
	return(prob)

def Entropy(probMap):
	entropy = 0
	for i in range(probMap.shape[0]):
		if probMap[i] != 0.0:
			entropy += probMap[i]*math.log(probMap[i],2)
	return -entropy

def FindNumberRobots(data):
	robots = 0
	for i in range(data.shape[0]):
		if data[i][1] > robots:
			robots = data[i][1]
	return(int(robots+1))
	
def StateChangeFrequency(numRobots, data):
	changePerRobot = np.empty((numRobots), dtype = float)
	change = 0
	for j in range(numRobots):
		total = 0.0
		time = 0.0
		lastState = None
		for i in range(data.shape[0]):
			if data[i][1] == j:
				time +=1
				if lastState is None:
					lastState = data[i][4]
				else:
					if data[i][4] != lastState:
						total +=1
						lastState = data[i][4]
		# in case we want a per agent frequency 
		changePerRobot[j] = total/time
		change += changePerRobot[j]
	return (change/numRobots)

def AvgCentroid(data):
	state_counter = np.zeros(4)
	state0_counter = 0
	state1_counter = 0
	for i in range(data.shape[0]):
		#print(data[i][4])
		if data[i][4] == 0.0:
			state_counter[0] += data[i][2]
			state_counter[1] += data[i][3]
			state0_counter += 1
		elif data[i][4] == 1.0:
			state_counter[2] += data[i][2]
			state_counter[3] += data[i][3]
			state1_counter +=1
	centroids = np.zeros((2,2))
	centroids[0][0] = state_counter[0]/state0_counter
	centroids[0][1] = state_counter[1]/state0_counter
	centroids[1][0] = state_counter[2]/state1_counter
	centroids[1][1] = state_counter[3]/state1_counter
	
	return(centroids)

def AvgStateScatter(avgCentroid, data):
	R = 0.5*5*math.sqrt(2)
	scatter = np.empty(2)
	state0_counter = 0
	state1_counter = 0
	for i in range(data.shape[0]):
		if data[i][4] == 0:
			scatter[0] += math.sqrt((data[i][2] - avgCentroid[0][0])**2+(data[i][2]-avgCentroid[0][1])**2)
			state0_counter +=1

		elif data[i][4] == 1:
			scatter[1] += math.sqrt((data[i][2] - avgCentroid[1][0])**2+(data[i][3] - avgCentroid[1][1])**2)
			state1_counter +=1
	scatter[0] = scatter[0]/(R*state0_counter)
	scatter[1] = scatter[1]/(R*state1_counter)
	return(scatter)
	

def FinalState(numRobots, data):
	final_time = 0
	final_data = np.empty((numRobots, 7))
	for i in range(data.shape[0]):
		final_time = data[i][0]
	n = 0
	for i in range(data.shape[0]-1, data.shape[0] - numRobots-1, -1):
		if data[i][0] == final_time:
			final_data[n] = data[i]
			n+=1
	return final_data


#--------------------- CODE to Run -----------------------------------

def main():
	data = np.ndarray((54000,7))
	Path = '/home/joshua/Documents/NEST/Summer_project/Experiment_Results/'
	folder = 'data_12345/'
	print('                             Run for '+ folder)
	for file in range(1, 21):

		name = 'experiment_12345_117624_'+str(file)+'.csv'

		with open(Path+folder+name) as csv_file:
			csv_reader  = csv.reader(csv_file, delimiter = ',')
			line_count = 0
			for row in csv_reader:
				for i in range(7):
					data[line_count][i] = row[i]
				line_count += 1


		print('###########################################################################')
		print('############################# Data for file '+str(file)+' #############################')
		print('###########################################################################')
		bins = MakeBins()
		binned_data = SortBins(bins, data)
		probMap1  = StateProb(binned_data, data)
		probMap0 = 1 - probMap1
		entropy1 = Entropy(probMap1)
		entropy0 = Entropy(probMap0)
		print('The Entropy for State = 0 is: '+str(entropy0))
		print('The Entropy for State = 1 is: '+str(entropy1))
		numRobots = FindNumberRobots(data)
		changeFrequency = StateChangeFrequency(numRobots, data)
		print('The State change frequency is: ' + str(changeFrequency))
		centroids = AvgCentroid(data)
		print('The centroid for state 0 is: '+ str(centroids[0]))
		print('The centroid for state 1 is: '+ str(centroids[1]))
		stateScatter = AvgStateScatter(centroids, data)
		print('The average scatter for state 0 is: '+str(stateScatter[0]))
		print('The average scatter for state 1 is: '+str(stateScatter[1]))

		# Final time only--------------------------
		print('\n#####################   Data for Final State of Simulation #################\n')
		
		final_data = FinalState(numRobots, data)

		final_Centroids = AvgCentroid(final_data)
		print('The Final centroid for state 0 is: '+ str(final_Centroids[0]))
		print('The Final centroid for state 1 is: '+ str(final_Centroids[1]))
		finalStateScatter = AvgStateScatter(final_Centroids, final_data)
		print('The average scatter for state 0 is: '+str(finalStateScatter[0]))
		print('The average scatter for state 1 is: '+str(finalStateScatter[1]))

		print('done')

if __name__ == '__main__':
	# main()
	print(MakeBins())
