#!/usr/bin/env python2

import rospy
import cv2
import rosbag
import time

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

#pub=rospy.Publisher('points', Image, queue_size=5)

def callback(data):
  bridge = CvBridge()
  image = bridge.imgmsg_to_cv2(data, encoding="passthrough")

  params = cv2.SimpleBlobDetector_Params()
  params.filterByColor = True
  params.blobColor = 255

  # Set up the detector with parameters.
  detector = cv2.SimpleBlobDetector(params)
  # Detect blobs.
  keypoints = detector.detect(image)
  print(keypoints)
  #x = keypoints[i].pt[0] #i is the index of the blob you want to get the position
  #y = keypoints[i].pt[1]
  #print(x)
  #print(y)
  #pub.publish(keypoints)

#contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)


if __name__ == '__main__':
  while not rospy.is_shutdown():
    #rospy.init_node('detector', anonymous=False)
    rospy.Subscriber("/voxel_image", Image, callback)
