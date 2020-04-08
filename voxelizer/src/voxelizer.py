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

PREV_IMG = None
READY = False


SEQ = 0
_SIZE = 350
_RING = 7

pub = rospy.Publisher('voxel_image', Image, queue_size=10)

def callback(data):
	global PREV_IMG
	global READY
	bridge = CvBridge()
	gen = point_cloud2.read_points(data, field_names = ("x", "y", "ring"))


	lidar_points = []
	for j in gen:
		if j[2] == _RING:
			lidar_points.append(j)
			
	# opencv Method
	voxels = np.zeros((_SIZE, _SIZE), dtype=np.uint8)
	for lidar_point in lidar_points:
		if( (abs(lidar_point[0]) < 17.5) and (abs(lidar_point[1]) < 17.5) ):
			x_voxel = int((lidar_point[0]+17.5)*10)
			y_voxel = int((lidar_point[1]+17.5)*10)
			try:
				voxels[x_voxel][y_voxel] = 255
			except:
				rospy.loginfo("voxel Out of range: " + str(lidar_point[0]) + " " + str(lidar_point[1]))
	if READY:
		subtracted_image = cv2.subtract(voxels, PREV_IMG)
		filtered_image = im_filter(subtracted_image)
		image_message = bridge.cv2_to_imgmsg(filtered_image, encoding="passthrough")

		pub.publish(image_message)
	PREV_IMG = voxels
	READY = True
		


			
	# gridcells method
	'''voxels = GridCells()
	voxels.header = data.header
	voxels.cell_width = _SIZE
	voxels.cell_height = _SIZE
	voxels.cells = []
	
	for lidar_point in lidar_points:
		if((abs(lidar_point[0]) < 17.5) and (abs(lidar_point[1] < 17.5))):
			x_voxel = int((lidar_point[0]+17.5)*0.1)
			y_voxel = int((lidar_point[1]+17.5)*0.1)
			z_voxel = 0
			voxel = Point(x_voxel, y_voxel, z_voxel)
			if( not(voxel in voxels.cells) ):
				voxels.cells.append(voxel)
	#rospy.loginfo("voxels: " + str(voxels))
	pub.publish(voxels)	'''
	
	
	
	'''if(max(dist_x) > MAX_X):
		MAX_X = max(dist_x)
		rospy.loginfo("max_dist_x:  " + str(max(dist_x)))	
	if(max(dist_y) > MAX_Y):
		MAX_Y = max(dist_y)
		rospy.loginfo("max_dist_y:  " + str(max(dist_y)))	'''

	
	
	'''processed_data = process_data(data.data)
	if processed_data is None:
		#rospy.loginfo("Bad Message: " + str(data))
		pass
	else:
		#pub.publish(processed_data)
		SEQ += 1'''
def im_filter(image):
	raw_pixels = np.where(image == 255)
	imcopy = image
	
	pixels = []
	for i in range(len(raw_pixels[0])):
		pixels.append([raw_pixels[0][i], raw_pixels[1][i]])
		
	for pixel in pixels:
		adjacent_pixels = check_nearby_pixels(pixel, image)
		if(adjacent_pixels < 3):
			imcopy[pixel[0]][pixel[1]] = 0
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
		
	pixels = image[x_lower_bound:x_upper_bound, y_lower_bound:y_upper_bound]
	for pixel in pixels.flatten():
		if pixel == 255:
			i+= 1
	return i

def main():
	rospy.init_node('voxelizer', anonymous=False)
	rospy.Subscriber("/ns1/velodyne_points", PointCloud2, callback)
	rospy.spin()

	
if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException:
		pass_
