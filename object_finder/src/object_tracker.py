#!/usr/bin/env python2


import rospy
import math

import cv2

from scipy.optimize import linear_sum_assignment
import numpy as np

_LOST_THRESH = 30 # number of loops before an object is considered lost and will be deleted
_DIFF_THRESH = 400 # threshold for considering an object to be a different object
_MIN_SIZE = 1 # Minimus size, lower than this is considered noise and will be deleted
_MAX_SIZE = 25 # Max size, higher than this means something went wrong and the object should be ignored

_MAX_LEN = 7

class object_tracker:
	def __init__(self):
		self.dynamic_objects = []
		self.ID_count = 0;
		
		
	def update(self, blobs, time_change):
		cost = np.empty((len(blobs), len(self.dynamic_objects)), dtype=np.uint32)
		# Cost matrix for solving best solutions. See Asignment Problem and hungarian algorithm
		for i in range(len(blobs)):
			for j in range(len(self.dynamic_objects)):
				cost[i][j] = self.dynamic_objects[j].match(blobs[i])
		#rospy.loginfo(str(cost))
		row_ind, col_ind = linear_sum_assignment(cost)
		blobs_copy = np.copy(blobs)
		# copy of the blobs and dynamic_objects. We will remove blobs and dynamic_objects from these lists as they are applied

		for i in range(len(row_ind)):
			
			if(cost[row_ind[i]][col_ind[i]] < _DIFF_THRESH):
				
				# If the object is less than the _diff_thresh then we condiser it a different object
				self.dynamic_objects[col_ind[i]].update(blobs[row_ind[i]], time_change)
				blobs_copy[row_ind[i]] = None
				
		for obj in self.dynamic_objects:
			if(obj.update_flag == False):
				obj.estimate(time_change)
			else:
				obj.update_flag = False
				
		# remove the Nones, remiaining blobs or objects havent been matched
		#blobs_copy[blobs_copy != np.array(None)]
		#rospy.loginfo(str(blobs_copy != np.array(None)))
		#rospy.loginfo(str(blobs_copy))
		for blob in blobs_copy:
			if(blob != None):
				self.dynamic_objects.append(dynamic_object(self.ID_count, blob))
				self.ID_count += 1
		for obj in self.dynamic_objects:
			obj.estimate_object()
			
		
		i = 0
		length = len(self.dynamic_objects)
		while(i < length):
			if(self.dynamic_objects[i].deletion_flag == True):
				del(self.dynamic_objects[i])
				length -= 1
			else:
				i += 1
				
		i = 0
		for obj in self.dynamic_objects:
			if(obj.deletion_flag == True):
				rospy.loginfo("Failed to Delete! Index = " + str(i))
			i += 1
	
	def pretty(self):
		string = ""
		for obj in self.dynamic_objects:
			#if(obj.ready_flag == True):
			string += "Object: " + str(obj.title) + " | X: " + str(obj.pt[0]) + " | Y: " + str(obj.pt[1]) + " | Size: " + str(obj.size) + "\n" 
		return string
		
	def tracked_objects(self):
		objects = []
		for obj in self.dynamic_objects:
			if(obj.ready_flag == True):
				objects.append(obj)
		return objects
		


		
class dynamic_object:
	def __init__(self, ID, blob):
		self.ID = ID
		
		self.blobj = blob # blobject
		
		self.title = "unassigned"
		self.ready_flag = False # object will not be considered until true, allows time for classification
		self.deletion_flag = False
		self.update_flag = False # true after the object was updated, false after object is estimated. Used to keep track of which objects were seen
		self.lost_loops = 0
		
		self.past_dx = []
		self.past_dy = []
	
	
	def match(self, blob):
		score = 0.0
		size_delta = blob.size - self.blobj.size
		x_delta = blob.pt[0] - self.blobj.pt[0]
		y_delta = blob.pt[1] - self.blobj.pt[1]
		distance_delta = ((x_delta**2)+(y_delta**2))
		
		score += 2*(distance_delta**2)
		score += size_delta # normally we would squareroot for distance but we would square it here so we do neither to save some computation
		return score
	
	
	def estimate_object(self):
		# try to recognize the object based on size and velocity
		# large objects = vehicle
		# small slow objects = pedestrian
		# small fast objects = cyclist
		# very small or very large objects = noise
		'''if(self.size > 10):
			if(self.size > 30):
				self.title = "Large Noise"
				self.deletion_flag = True
			elif(self.velocity[0] > 0.1):
				rospy.loginfo("Car!")
				self.title = "vehicle"
				self.ready_flag = True
			else:
				self.title = "Unknown"
		elif(self.size >= 1.0):
			if(self.velocity[0] > 0.5):
				rospy.loginfo("Cyclist")
				self.title = "cyclist"
				self.ready_flag = True
			elif(self.velocity[0] > 0.1):
				rospy.loginfo("Pedesitran")
				self.title = "Pedestrian"
				self.ready_flag = True
			else:
				self.title = "Unknown"
		else:
			self.title = "Noise"
			self.deletion_flag = True'''
		avg_dx = 0
		avg_dy = 0
		if(len(self.past_dx) > 0):
			avg_dx = sum(self.past_dx)/len(self.past_dx)
		if(len(self.past_dy) > 0):
			avg_dy = sum(self.past_dy)/len(self.past_dy)
		avg_dist = ((avg_dx**2)+(avg_dy**2))**0.5
		
		# basic classifier, does not use velocity only size
		if(self.blobj.size > 20.0):
			self.title = "Large Noise"
			self.deletion_flag = True
		elif(self.blobj.size > 8.0):
			if(avg_dist != 0):
				self.title = "vehicle"
				self.ready_flag = True
		elif(self.blobj.size >= 4.0):
			if(avg_dist != 0):
				self.title = "cyclist"
				self.ready_flag = True
		elif(self.blobj.size >= 2.0):
			if(avg_dist != 0):
				self.title = "Pedestrian"
				self.ready_flag = True
		else:
			self.title = "Noise"
			self.deletion_flag = True
				
	
	
	def update(self, blob, time_change):
		# Call this every callback when the object has been detected

		
		x_displacement = blob.pt[0] - self.blobj.pt[0]
		y_displacement = blob.pt[1] - self.blobj.pt[1]
		
		self.past_dx.append(x_displacement)
		self.past_dy.append(y_displacement)
		while(len(self.past_dx) > _MAX_LEN):
			self.past_dx.pop(0)
		while(len(self.past_dy) > _MAX_LEN):
			self.past_dy.pop(0)
		
		#self.velocity = [speed, angle]
		self.blobj = blob 
		self.update_flag = True
		self.lost_loops = 0;
		
		
	def estimate(self, time_change):
		# Call this every callback when the object is not detected
		avg_dx = 0
		avg_dy = 0
		if(len(self.past_dx) > 0):
			avg_dx = sum(self.past_dx)/len(self.past_dx)
		if(len(self.past_dy) > 0):
			avg_dy = sum(self.past_dy)/len(self.past_dy)

		self.blobj.move(avg_dx, avg_dy)
		
		if((self.blobj.pt[0] < 0) or (self.blobj.pt[0] > 350) or (self.blobj.pt[1] < 0) or (self.blobj.pt[1] > 350)):
			rospy.loginfo(str(self.ID) + " got too far away: " + str(self.blobj.pt[0]) + ", " + str(self.blobj.pt[1]))
			self.deletion_flag = True
		self.lost_loops += 1
		if(self.lost_loops > _LOST_THRESH):
			rospy.loginfo("Lost " + str(self.ID))
			self.deletion_flag = True
			
class blobject:
	def __init__(self, pt, size, x, y, w, h):
		# all these values are measured in pixels, but they are floating point!
		
		self.pt = pt # this is the center of the blob, used for tracking center 
		self.size = size # approxamate size of the blob, used to filter small objects
		
		# below are the bounding boxes of the blob, measured in pixels
		self.x = x 
		self.y = y 
		self.w = w
		self.h = h
	
	def move(self, dx, dy):
		self.pt[0] = self.pt[0] + dx
		self.pt[1] = self.pt[1] + dy
		self.x = self.x + dx
		self.y = self.y + dy
		#self.w = self.w
		#self.h = self.h
		
		
