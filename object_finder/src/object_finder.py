#!/usr/bin/env python2

import rospy
import cv2
import rosbag
import time
import numpy as np

from object_tracker import *

from sensor_msgs.msg import Image 
from std_msgs.msg import Float64, Float64MultiArray
from cv_bridge import CvBridge

pub_image=rospy.Publisher('contours', Image, queue_size=5)
pub_position=rospy.Publisher('contour_location', Float64MultiArray, queue_size=5)
pub_size=rospy.Publisher('contour_size', Float64, queue_size=5)

OBJ_TRACKER = object_tracker
PREV_TIME = None

def get_blobs(contours):
	blobs = []
	for obj in contours:
		pos = blob_position(obj)
		size = blob_size(obj)
		blobs.append(blobject(pos, size))
	return blobs


def blob_position(cont):
    pixel=cont[0][0]
    x_pixel=float(pixel[0]-175)*0.1
    y_pixel=float(pixel[1]-175)*0.1
    
    return [x_pixel, y_pixel]


def blob_size(cont):
    area = cv2.contourArea(cont[c])
    perimeter = cv2.arcLength(cont[c], False)
    
    return area


def callback(data):
    global OBJ_TRACKER
    global PREV_TIME
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(data)
    ros_time = data.header.stamp
    if(PREV_TIME == None):
    	# assume that 0.1 seconds has passed. This is approxamately correct for the VLP-16
    	PREV_TIME = ros_time - 0.1
    time_change = PREV_TIME - ros_time

    PREV_TIME = ros_time


    #Blob Method
    '''
    #set-up blob parameters
    params = cv2.SimpleBlobDetector_Params()

    #Filtering white spaces
    params.filterByColor = True
    params.blobColor = 255

    #setting threshold values
    params.minThreshold = 0
    params.maxThreshold = 200

    ## check opencv version and construct the detector
    is_v2 = cv2.__version__.startswith("2.")
    if is_v2:
        detector = cv2.SimpleBlobDetector(params)
    else:
        detector = cv2.SimpleBlobDetector_create(params)

    # Detect blobs.
    keypoints = detector.detect(image)
    nblobs=len(keypoints)
    #print(nblobs)
    
    for i in range(nblobs):
        x = keypoints[i].pt[0] #i is the index of the blob you want to get the position
        y = keypoints[i].pt[1]
        s = keyPoint.size
        #print(x)
        #print(y)
    '''

    contours, hierarchy = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2:]
    #print(len(contours))
    #print(contours)

    blobs = get_blobs(contours)
    OBJ_TRACKER.update(blobs)
	
    #rospy.publish(the blobs and their locations. Might need to make our own ROS message)


def main():
    #rospy.init_node('image_listener')
    rospy.init_node('detector', anonymous=False)
    rospy.Subscriber("/voxel_image", Image, callback)
    rospy.spin()


if __name__ == '__main__':
   main()
