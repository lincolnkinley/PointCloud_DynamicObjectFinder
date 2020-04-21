#!/usr/bin/env python2

import rospy
import cv2
import rosbag
import time
import numpy as np

from object_tracker import *

from sensor_msgs.msg import Image 
from std_msgs.msg import Float64, Float64MultiArray

from object_finder.msg import *

from cv_bridge import CvBridge

pub_objects=rospy.Publisher('tracked_objects', Image, queue_size=5)

pub_image=rospy.Publisher('contours', Image, queue_size=5)
pub_data_image=rospy.Publisher('data_image', Image, queue_size=5)

pub_position=rospy.Publisher('contour_location', Float64MultiArray, queue_size=5)
pub_size=rospy.Publisher('contour_size', Float64, queue_size=5)

OBJ_TRACKER = object_tracker()
PREV_TIME = None

def get_blobs(contours):
	blobs = []
	for obj in contours:
		pos = blob_position(obj)
		size = blob_size(obj)
		blobs.append(blobject(pos, size))
	return blobs


def blob_position(cont):
	M = cv2.moments(cont)
	cX = 0.0
	cY = 0.0
	if(M["m00"] != 0.0):
		cX = int(M["m10"] / M["m00"])
		cY = int(M["m01"] / M["m00"])
	x_pixel=float(cX)
	y_pixel=float(cY)
	return [x_pixel, y_pixel]


def blob_size(cont):
    area = cv2.contourArea(cont)
    #perimeter = cv2.arcLength(cont, False)
    
    return area


def callback(data):
    global OBJ_TRACKER
    global PREV_TIME
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(data)
    ros_time = data.header.stamp
    if(PREV_TIME == None):
    	# assume that 0.1 seconds has passed. This is approxamately correct for the VLP-16
    	PREV_TIME = ros_time - rospy.Duration.from_sec(0.1)
    time_change = ros_time - PREV_TIME

    PREV_TIME = ros_time
    float_time = time_change.to_sec()

    detected_objects = extract_and_filter(image)
    
        
    OBJ_TRACKER.update(detected_objects, float_time)
    
    color = cv2.cvtColor(image,cv2.COLOR_GRAY2RGB)
    #color = cv2.drawKeypoints(color, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    
    # contour detection 
    '''
    contours, hierarchy = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2:]
    #print(len(contours))
    #print(contours)
    

    blobs = get_blobs(contours)
    
    OBJ_TRACKER.update(blobs, float_time)
    '''
    
    tracked_obj = OBJ_TRACKER.tracked_objects()
    #rospy.loginfo(OBJ_TRACKER.pretty())
    
    
    
    font = cv2.FONT_HERSHEY_SIMPLEX
    #rospy.loginfo("Objects Detected: " + str(len(keypoints)))
    #rospy.loginfo("Objects tracked: " + str(len(tracked_obj)))
    for obj in tracked_obj:
    	
    	x = int(obj[0][0])
    	y = int(obj[0][1])
    	try:
    		color[y,x] = (0,255,0)
    	except:
    		pass
            

 

    
    
    image_message = bridge.cv2_to_imgmsg(color, encoding="passthrough")
    pub_objects.publish(image_message)
    #rospy.loginfo(OBJ_TRACKER.pretty())
	
    #rospy.publish(the blobs and their locations. Might need to make our own ROS message)
    
# returns a list of dynamic objects detected in the image
def extract_and_filter(image):
    bridge = CvBridge()
    #imcopy = np.copy(image)
    #Blob Method
    
    #set-up blob parameters
    params = cv2.SimpleBlobDetector_Params()

    #Filtering white spaces
    params.filterByColor = False
    #params.blobColor = 255
    
    params.filterByArea = True
    params.minArea = 3
    params.maxArea = 900

    #setting threshold values
    params.minThreshold = 40
    params.maxThreshold = 256

    ## check opencv version and construct the detector
    is_v2 = cv2.__version__.startswith("2.")
    detector = None
    if is_v2:
        detector = cv2.SimpleBlobDetector(params)
    else:
        detector = cv2.SimpleBlobDetector_create(params)

    # Detect blobs.
    keypoints = detector.detect(image)
    ret, thresh = cv2.threshold(image, 20, 255, 0)
    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2:]
    
    image_message = bridge.cv2_to_imgmsg(thresh, encoding="passthrough")
    pub_data_image.publish(image_message)
    
    detected_contours = []
    
    detected_objects = []
    
    for keypoint in keypoints:
        x = keypoint.pt[0] #i is the index of the blob you want to get the position
        y = keypoint.pt[1]
        for contour in contours:
        	result = cv2.pointPolygonTest(contour, (x,y), False) 
        	if(result >= 0): # this keypoint is on or inside this contour
        		covered_flag = False # 
        		for detected_contour in detected_contours:
        			second_result = cv2.pointPolygonTest(detected_contour, (x,y), False) 
        			if(second_result >= 0): # this keypoint is on or inside a contour that has already been detected
        				covered_flag = True
        				break
        		if(covered_flag == False):
        			detected_contours.append(contour)
        			
    for contour in detected_contours:
    	detected_objects.append(blobject(blob_position(contour), blob_size(contour)))
	x,y,w, h=cv2.boundingRect(contour)
	image=cv2.rectangle(image, (x,y), (x+w, y+h), (2550,0,0), 1)
    	
    return detected_objects
        		
        #rospy.loginfo("Obj[" + str(i) + "] size: "+ str(s))
    

def main():
    #rospy.init_node('image_listener')
    rospy.init_node('detector', anonymous=False)
    rospy.Subscriber("/noise_image", Image, callback)
    rospy.spin()


if __name__ == '__main__':
   main()
