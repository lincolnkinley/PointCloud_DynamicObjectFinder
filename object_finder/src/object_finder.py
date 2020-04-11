#!/usr/bin/env python2

import rospy
import cv2
import rosbag
import time
import numpy as np

from sensor_msgs.msg import Image 
from std_msgs.msg import Float64, Float64MultiArray
from cv_bridge import CvBridge

pub_image=rospy.Publisher('contours', Image, queue_size=5)
pub_position=rospy.Publisher('contour_location', Float64MultiArray, queue_size=5)
pub_size=rospy.Publisher('contour_size', Float64, queue_size=5)

def position(cont):
    for c in range(len(cont)):
        area = cv2.contourArea(cont[c])
        perimeter = cv2.arcLength(cont[c], False)
        pixel=cont[c][0][0]
        x_pixel=abs(pixel[0]-175)*0.1
        y_pixel=abs((pixel[1]-175))*0.1
        position=Float64MultiArray()
        position.data=[x_pixel, y_pixel]
        pub_position.publish(position)
        print(position.data)


def size(cont):
    for c in range(len(cont)):
        area = cv2.contourArea(cont[c])
        perimeter = cv2.arcLength(cont[c], False)
        pub_size.publish(perimeter)
        #print(perimeter)


def callback(data):
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(data)

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

    color =(255,255,255)

    cv2.drawContours(image, contours, -1, color, 2) 

    image_message = bridge.cv2_to_imgmsg(image, encoding="passthrough")

    pub_image.publish(image_message)

    position(contours)
    size(contours)


def main():
    #rospy.init_node('image_listener')
    rospy.init_node('detector', anonymous=False)
    rospy.Subscriber("/voxel_image", Image, callback)
    rospy.spin()


if __name__ == '__main__':
   main()
