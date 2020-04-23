#!/usr/bin/env python2

import rospy
import math

import cv2

from scipy.optimize import linear_sum_assignment
import numpy as np

_LOST_THRESH = rospy.get_param("/lost_threshold") # number of loops before an object is considered lost and will be deleted
_DIFF_THRESH = rospy.get_param("/difference_threshold") # threshold for considering an object to be a different object
_MIN_SIZE = rospy.get_param("/minimum_size") # Minimus size, lower than this is considered noise and will be deleted
_MAX_SIZE = rospy.get_param("/maximum_size") # Max size, higher than this means something went wrong and the object should be ignored
_MAX_LEN = rospy.get_param("/tracked_frames") # number of loops an object will be tracked for. Used to esitmate position of a unseen object

_LARGE_SIZE = rospy.get_param("/large_size") # objects larger than this will be filtered out
_VEHICLE_SIZE = rospy.get_param("/vehicle_size") # objects larger than this will be classified as a vehicle
_PEDESTRIAN_SIZE = rospy.get_param("/pedestrian_size") # objects larger than this will be classified as a pedestrian or cyclist depending on speed
_BICYCLE_SPEED = rospy.get_param("/bicycle_speed") # pedestrians above this speed will be classified as cyclists

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
				
		row_ind, col_ind = linear_sum_assignment(cost)
		'''rospy.loginfo(str(cost))
		rospy.loginfo(str(row_ind))
		rospy.loginfo(str(col_ind))'''
		blobs_copy = np.copy(blobs) # copy of the blobs and dynamic_objects. We will remove blobs and dynamic_objects from these lists as they are applied

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
	
	# Returns a string with data of all the objects. Useful for debugging
	def pretty(self):
		string = ""
		for obj in self.dynamic_objects:
			#if(obj.ready_flag == True):
			string += "Object: " + str(obj.title) + " | X: " + str(obj.pt[0]) + " | Y: " + str(obj.pt[1]) + " | Size: " + str(obj.size) + "\n" 
		return string
	
	# returns all of the ready tracked objects
	def tracked_objects(self):
		objects = []
		for obj in self.dynamic_objects:
			if(obj.ready_flag == True):
				objects.append(obj)
		return objects
		


		
class dynamic_object:
	def __init__(self, ID, blob):
		self.ID = ID # integer ID used for tracking
		self.blobj = blob # blobject
		self.title = "unassigned" # name of the object, typically vehicle, cyclist, or pedestrian
		self.ready_flag = False # object will not be considered until true, allows time for classification
		self.deletion_flag = False # set this to true and the object will be deleted
		self.update_flag = False # true after the object was updated, false after object is estimated. Used to keep track of which objects were seen
		self.lost_loops = 0 # number of loops the object has not been seen. Object will be deleted if this is larger than lost_threshold
		self.past_dx = [] # past changes in x direction, used to estimate next position of object is unseen
		self.past_dy = [] # past changes in y direction, ""
	
	
	# claculates a score between this blob and a different blob, higher score means less likely
	def match(self, blob, check = False):
		score = 0.0
		size_delta = abs(blob.size - self.blobj.size)
		x_delta = blob.pt[0] - self.blobj.pt[0]
		y_delta = blob.pt[1] - self.blobj.pt[1]
		distance_delta = ((x_delta**2)+(y_delta**2))
		
		score += (distance_delta)
		score += 4*(size_delta**2) # normally we would squareroot for distance but we would square it here so we do neither to save some computation

		'''rospy.loginfo("Match Score: " + str(size_delta) + " | " + str(x_delta) + " | " + str(y_delta) + " | " + str(distance_delta) + " | " + str(score))
		rospy.loginfo("Compairing " + str(self.ID) + " at " + str(self.blobj.pt) + " to " + str(blob.pt))'''
		return score
	
	
	def estimate_object(self):
		# try to recognize the object based on size and velocity
		# large objects = vehicle
		# small slow objects = pedestrian
		# small fast objects = cyclist
		# very small or very large objects = noise

		avg_dx = 0
		avg_dy = 0
		if(len(self.past_dx) > 0):
			avg_dx = sum(self.past_dx)/len(self.past_dx)
		if(len(self.past_dy) > 0):
			avg_dy = sum(self.past_dy)/len(self.past_dy)
		avg_dist = ((avg_dx**2)+(avg_dy**2))**0.5
		
		if(self.blobj.size > _LARGE_SIZE):
			self.title = "Large Noise"
			self.deletion_flag = True
			
		elif(self.blobj.size > _VEHICLE_SIZE):
			if(avg_dist != 0):
				self.title = "vehicle"
				self.ready_flag = True
				
		elif(self.blobj.size >= _PEDESTRIAN_SIZE):
			if(avg_dist > _BICYCLE_SPEED):
				self.title = "cyclist"
				self.ready_flag = True
				
			elif(avg_dist != 0):
				self.title = "pedestrian"
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
		if((avg_dy > 4.7) or (avg_dx > 4.7)):
			rospy.loginfo("Fast Delete")
			self.deletion_flag = True
			return

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
		
		
