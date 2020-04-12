#!/usr/bin/env python2

import rospy
import math

from scipy.optimize import linear_sum_assignment
import numpy as np

_LOST_THRESH = 40 # number of loops before an object is considered lost and will be deleted
_DIFF_THRESH = 20 # threshold for considering an object to be a different object
_MIN_SIZE = 1 # Minimus size, lower than this is considered noise and will be deleted
_MAX_SIZE = 25 # Max size, higher than this means something went wrong and the object should be ignored

class object_tracker:
	def __init__(self):
		self.dynamic_objects = []
		self.ID_count = 0;
		
		
	def update(self, blobs, time_change):
		cost = np.empty((len(blobs), len(self.dynamic_objects), dtype=np.uint16)
		# Cost matrix for solving best solutions. See Asignment Problem and hungarian algorithm
		for i in values:
			for j in i:
				cost[i][j] = self.dynamic_objects[j].match(blobs[i])
		row_ind, col_ind = linear_sum_assignment(values)
		blobs_copy = np.copy(blobs)
		# copy of the blobs and dynamic_objects. We will remove blobs and dynamic_objects from these lists as they are applied
		for i in range(len(row_ind)):
			if(cost[col_ind[i]][row_ind[i]] < _DIFF_THRESH):
				# If the object is less than the _diff_thresh then we condiser it a different object
				self.dynamic_objects[row_ind[i]].update(blobs[col_ind[i]], time_change)
				blobs_copy[col_ind[i]] = None
				
		for obj in dynamic_objects:
			if(obj.update_flag == False):
				obj.estimate(time_change)
				
		# remove the Nones, remiaining blobs or objects havent been matched
		blobs_copy[blobs_copy != np.array(None)]
		
		for blob in blobs_copy:
			self.dynamic_objects.append(dynamic_object(self.ID_count, blob.size, blob.position))
			self.ID_count += 1
		
		for obj in self.dynamic_objects:
			if(obj.deletion_flag == True):
				del(obj)
			else
				obj.estimate_object()
		
		


		
class dynamic_object:
	def __init__(self, ID, size, position):
		self.ID = ID
		self.size = size
		self.position = position # 1d array [x, y], relative to lidar, NOT IN VOXELS!
		self.velocity = [0 0] # 1d array [speed (m/s), direction]
		self.title = "unknown"
		self.ready_flag = False # object will not be considered until true, allows time for classification
		self.deletion_flag = False
		self.update_flag = False # true after the object was updated, false after object is estimated. Used to keep track of which objects were seen
		self.lost_loops = 0
		if(self.size < _MIN_SIZE):
			self.deletion_flag = True
		elif(self.size > _MAX_SIZE):
			self.deletion_flag = True
	
	
	def match(self, blob)
		score = 0.0
		size_delta = blob.size-self.size
		x_delta = blob.position[0] - self.position[0]
		y_delta = blob.position[1] - self.position[1]
		distance_delta = ((x_delta**2)+(y_delta**2))**2
		
		score += 0.25*distance_delta**2
		score += size_delta**2
		return score
	
	
	def estimate_object(self):
		# try to recognize the object based on size and velocity
		# large objects = vehicle
		# small slow objects = pedestrian
		# small fast objects = cyclist
		# very small or very large objects = noise
		self.title = "dynamic object"
		self.ready_flag = True # we might not need ready_flag at all...
		# TODO
	
	
	def update(self, blob, time_change):
		# Call this every callback when the object has been detected
		x_displacement = blob[0][0] - self.position[0]
		y_displacement = blob[0][1] - self.position[1]
		speed = (((x_displacement)**2+(y_displacement)**2)**0.5)/time_change
		angle = math.atan2(y_displacement, x_displacement)
		self.velocity = [speed, angle]
		self.size = blob[1] # We might not need to update size, the reason we do (for now) is if the lidar detects something partially outside view it could get the initial estimate wrong which would throw off future detection
		self.update_flag = True
		
		
	def estimate(self, time_change):
		# Call this every callback when the object is not detected
		x = self.position[0]
		y = self.position[1]
		speed = self.velocity[0]
		angle = self.velocity[1]
		x = x + speed*math.cos(angle)*time_change
		y = y + speed*math.sin(angle)*time_change
		self.position = [x, y]
		self.lost_loops += 1
		if(self.lost_loops > _LOST_THRESH):
			self.deletion_flag = True
			
class blobject:
	def __init__(self, position, size):
		self.position = position
		self.size = size
