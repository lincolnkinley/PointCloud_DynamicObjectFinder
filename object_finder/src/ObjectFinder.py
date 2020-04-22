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


pub_objects = rospy.Publisher("tracked_objects", BoxArray, queue_size=5)
pub_objects_im=rospy.Publisher('tracked_objects_im', Image, queue_size=5)

pub_data_image=rospy.Publisher('data_image', Image, queue_size=5)

OBJ_TRACKER = object_tracker()
PREV_TIME = None
SEQ = 0
ORIGINAL_IMAGE = None
ORIGINAL_READY = True

_LIDAR_FRAME = rospy.get_param("/lidar_frame") # frame of the LiDAR
_UPPER_Z = rospy.get_param("/upper_z") # upper z limit of bounding box
_LOWER_Z = rospy.get_param("/lower_z") # lower z limit of bounding box

def original_callback(data):
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(data)
    global ORIGINAL_IMAGE
    global ORIGINAL_READY
    
    ORIGINAL_IMAGE = image
    ORIGINAL_READY = True
	
	
def get_bounding_box(blob):
	y = float(blob.blobj.x-175)/10
	x = float(blob.blobj.y-175)/10
	h = float(blob.blobj.w)/10
	w = float(blob.blobj.h)/10
	#rospy.loginfo(str(x) + " | " + str(y) + " | " + str(w) + " | " +str(h))
	
	bb = BoundingBox()
	bb.p1.x = x
	bb.p1.y = y
	bb.p1.z = _UPPER_Z
	
	bb.p2.x = x+w
	bb.p2.y = y+h
	bb.p2.z = _LOWER_Z
	
	bb.text = blob.title
	bb.id = int(blob.ID)
	
	return bb


def get_blobs(contours):
	blobs = []
	for obj in contours:
		pos = blob_position(obj)
		size = blob_size(obj)
		x,y,w,h = cv2.boundingRect(obj)
		blobs.append(blobject(pos, size, x, y, w, h))
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
    return area


def callback(data):
    global OBJ_TRACKER
    global PREV_TIME
    global SEQ
    global ORIGINAL_READY
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(data)
    while(ORIGINAL_READY == False):
    	rospy.sleep(0.01)
    ORIGINAL_READY = False
    original = ORIGINAL_IMAGE.copy()
    ros_time = data.header.stamp
    if(PREV_TIME == None):
    	# assume that 0.1 seconds has passed. This is approxamately correct for the VLP-16
    	PREV_TIME = ros_time - rospy.Duration.from_sec(0.1)
    time_change = ros_time - PREV_TIME

    PREV_TIME = ros_time
    float_time = time_change.to_sec()

    detected_objects = extract_and_filter(image, original)
    
    OBJ_TRACKER.update(detected_objects, float_time)
    
    color = cv2.cvtColor(image,cv2.COLOR_GRAY2RGB)

    tracked_obj = OBJ_TRACKER.tracked_objects()
    
    box_array = []
    for obj in tracked_obj:
    	bb = get_bounding_box(obj)
    	box_array.append(bb)
    	x = int(obj.blobj.pt[0])
    	y = int(obj.blobj.pt[1])
    	try:
    		color[y,x] = (0,255,0)
    	except:
    		rospy.loginfo("X,Y out of bounds: " + str(x) + " | " + str(y))
            
    ba = BoxArray()
    ba.boxes = box_array
    ba.header.seq = SEQ
    SEQ += 1
    ba.header.stamp = ros_time
    ba.header.frame_id = _LIDAR_FRAME
    
    image_message = bridge.cv2_to_imgmsg(color, encoding="passthrough")
    pub_objects_im.publish(image_message)
    pub_objects.publish(ba)

	
# returns a list of dynamic objects detected in the image
def extract_and_filter(image, original):
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
    params.minThreshold = 30
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
    ret, thresh = cv2.threshold(original, 20, 256, 0)
    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2:]
    
    color = cv2.cvtColor(image,cv2.COLOR_GRAY2RGB)
    
    color = cv2.drawKeypoints(color, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    
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
        			
    cv2.drawContours(color, contours, -1, (0,255,0), 1)
    cv2.drawContours(color, detected_contours, -1, (255,0,0), 1)
    image_message = bridge.cv2_to_imgmsg(color, encoding="passthrough")
    pub_data_image.publish(image_message)
    for contour in detected_contours:
    	x,y,w,h = cv2.boundingRect(contour)
    	detected_objects.append(blobject(blob_position(contour), blob_size(contour), x, y, w, h))
	    
    return detected_objects
    

def main():
    #rospy.init_node('image_listener')
    rospy.init_node('detector', anonymous=False)
    rospy.Subscriber("voxel_data/noise", Image, callback)
    rospy.Subscriber("voxel_data/original", Image, original_callback)
    rospy.spin()


if __name__ == '__main__':
   main()
