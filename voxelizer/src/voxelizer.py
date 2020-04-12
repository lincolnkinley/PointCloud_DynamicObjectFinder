#!/usr/bin/env python2

import rospy
import time

import numpy as np
import cv2

from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2

from geometry_msgs.msg import Point

from nav_msgs.msg import GridCells

from sensor_msgs.msg import Image

from cv_bridge import CvBridge

from geometry_msgs.msg import Pose 

PREV_IMG = None
READY = False

PREV_POSITION = [0,0]
CURRENT_POSITION = [0,0]

_SIZE = 350
_RING = 7
_RING_MIN = 5 # lowest ring that will be scanned
_RING_MAX = 8 # highest ring that will



pub = rospy.Publisher('voxel_image', Image, queue_size=10)

def position_callback(data):
	global CURRENT_POSITION
	saved_pos = CURRENT_POSITION
	try:
		CURRENT_POSITION[0] = data.position.x
		CURRENT_POSITION[0] = data.position.y
	except:
		CURRENT_POSITION = saved_pos
		rospy.loginfo("Position update error!")

def callback(data):
	global PREV_IMG
	global PREV_POSITION
	global READY
	measured_position = list(CURRENT_POSITION) # we want the measured position to be as close to the time the lidar data was measured as possible. It's possible that the current position could change during this callback
	bridge = CvBridge()
	gen = point_cloud2.read_points(data, field_names = ("x", "y", "ring"))

	
	voxels = np.zeros((_SIZE, _SIZE, _RING_MAX-_RING_MIN+1), dtype=np.uint8)
	
	lidar_points = [[] for x in xrange(_RING_MIN, _RING_MAX+1)]
	for j in gen:
		if ((j[2] >= _RING_MIN) and (j[2] <= _RING_MAX)):
			if( (abs(j[0]) < 17.5) and (abs(j[1]) < 17.5) ):
				x_voxel = int((j[0]+17.5)*10)
				y_voxel = int((j[1]+17.5)*10)
				ring = int(j[2]-_RING_MIN)
				voxels[x_voxel][y_voxel][ring] = 255
	
	#Original order, subtracting the 3d image
	if READY:
		
		y_shift = int((measured_position[0] - PREV_POSITION[0]) * -10)
		x_shift = int((measured_position[1] - PREV_POSITION[1]) * -10)
		rospy.loginfo("X: " + str(x_shift) + " | Y: " + str(y_shift))
		shift = np.float32([[1, 0, x_shift],[0,1,y_shift]])
		rows, cols, zeasus = voxels.shape
		shifted_prev_img = cv2.warpAffine(PREV_IMG, shift, (cols, rows))
		subtracted_image = cv2.subtract(voxels, shifted_prev_img)
		filtered_image = im_3d_filter(subtracted_image)
		flat_image = flatten(filtered_image)
		filtered_flat_image = im_2d_filter(flat_image)
		
		image_message = bridge.cv2_to_imgmsg(filtered_flat_image, encoding="passthrough")
		image_message.header.stamp = rospy.Time.now()
		pub.publish(image_message)
		
	PREV_IMG = voxels
	PREV_POSITION = measured_position
	READY = True
		
		
	# New order, subtracting 3d filtered image
	'''filtered_image = im_3d_filter(voxels)
	
	
	if READY:
		x_shift = int(measured_position[0] - PREV_POSITION[0])
		y_shift = int(measured_position[1] - PREV_POSITION[1])
		shift = np.float32([[1, 0, x_shift],[0,1,y_shift]])
		rows, cols, zeasus = voxels.shape
		shifted_prev_img = cv2.warpAffine(PREV_IMG, shift, (cols, rows))
		
		subtracted_image = cv2.subtract(filtered_image, shifted_prev_img)
		flat_image = flatten(subtracted_image)
		filtered_flat_image = im_2d_filter(flat_image)
		
		image_message = bridge.cv2_to_imgmsg(filtered_flat_image, encoding="passthrough")
		image_message.header.stamp = rospy.Time.now()
		pub.publish(image_message)
		
	PREV_IMG = filtered_image
	READY = True'''
	
	
def flatten(image):
	#rospy.loginfo(str(np.where(np.logical_and(image > 0, image < 255))))
	imcopy = np.sum(image, axis=2)
	imcopy = imcopy.astype(np.uint8)
	#rospy.loginfo(str(imcopy.shape))
			
	return imcopy
		
			
def im_2d_filter(image):
	imcopy = image
	detection_thresh = 245
	imcopy[imcopy > detection_thresh] = 255
	imcopy[imcopy <= detection_thresh] = 0
	imcopycopy = imcopy
	detected_pixels = np.where(imcopy == 255)
	for i in range(len(detected_pixels[0])):
		pixel = [detected_pixels[0][i], detected_pixels[1][i]]
		adjacent_pixels = check_nearby_pixels(pixel, imcopy)
		if(adjacent_pixels < 2):
			imcopycopy[pixel[0]][pixel[1]] = 0
	imcopycopy[171:180][171:180] = 0
	return imcopycopy


def im_3d_filter(image):
	raw_pixels = np.where(image == 255)
	imcopy = np.zeros((_SIZE, _SIZE, _RING_MAX-_RING_MIN+1), dtype=np.uint8)
	
	pixels = []
	# can speed this up by combining the two for loops
	for i in range(len(raw_pixels[0])):
		pixels.append([raw_pixels[0][i], raw_pixels[1][i], raw_pixels[2][i]])
		
	for pixel in pixels:
		adjacent_pixels = check_nearby_pixels(pixel, image)*36
		if(adjacent_pixels > 255):
			adjacent_pixels = 255
		imcopy[pixel[0]][pixel[1]][pixel[2]] = int(adjacent_pixels)

	return imcopy
		
		
def check_nearby_pixels(pixel_location, image):
	i = 0
	x_lower_bound = pixel_location[0] - 1
	if(x_lower_bound < 0):
		x_lower_bound = 0
		
	x_upper_bound = pixel_location[0] + 2
	if(x_upper_bound > (_SIZE)):
		x_upper_bound = _SIZE

	y_lower_bound = pixel_location[1] - 1
	if(y_lower_bound < 0):
		y_lower_bound = 0
		
	y_upper_bound = pixel_location[1] + 2
	if(y_upper_bound > (_SIZE)):
		y_upper_bound = _SIZE
		
	if(len(pixel_location) == 3): # 3d Image
		z_lower_bound = pixel_location[2] - 1
		if(z_lower_bound < 0):
			z_lower_bound = 0
		
		z_upper_bound = pixel_location[2] + 2
		if(z_upper_bound > ((_RING_MAX+1)-_RING_MIN)):
			z_upper_bound = (_RING_MAX+1)-_RING_MIN
		
		pixels = image[x_lower_bound:x_upper_bound, y_lower_bound:y_upper_bound, z_lower_bound:z_upper_bound]
		for pixel in pixels.flatten():
			if pixel == 255:
				i+= 1
	else: # 2d image
		pixels = image[x_lower_bound:x_upper_bound, y_lower_bound:y_upper_bound]
		for pixel in pixels.flatten():
			if pixel == 255:
				i+= 1
			
	return i

def main():
	rospy.init_node('voxelizer', anonymous=False)
	rospy.Subscriber("/ns1/velodyne_points", PointCloud2, callback)
	rospy.Subscriber("/pose_info", Pose, position_callback)
	rospy.spin()

	
if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException:
		pass_
