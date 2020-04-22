#!/usr/bin/env python2


import rospy

from object_finder.msg import *

from geometry_msgs.msg import *
from std_msgs.msg import ColorRGBA

from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker

marker_pub = rospy.Publisher("visualized_bounding_boxes", MarkerArray, queue_size=5)

def callback(data):
	boxes = data.boxes
	marker_array = MarkerArray()
	markers = []
	i = 0
	for box in boxes:
		new_marker = Marker()
		new_marker.header = data.header
		new_marker.ns = str(i)
		new_marker.id = box.id
		new_marker.type = 5 #line list
		new_marker.action = 0
		
		marker_pose = Pose()
		marker_pose.position.x = 0
		marker_pose.position.y = 0
		marker_pose.position.z = 0
		marker_pose.orientation.x = 0
		marker_pose.orientation.y = 0
		marker_pose.orientation.z = 0
		marker_pose.orientation.w = 1
		new_marker.pose = marker_pose
		
		scale_vec = Vector3()
		scale_vec.x = 0.1
		scale_vec.y = 0.1
		scale_vec.z = 0.1
		new_marker.scale = scale_vec
		
		line_color = ColorRGBA()
		if(str(box.text) == "pedestrian"):
			line_color.r = 0.0
			line_color.g = 0.0
			line_color.b = 1.0
			line_color.a = 1.0
		elif(str(box.text) == "cyclist"):
			line_color.r = 0.0
			line_color.g = 1.0
			line_color.b = 0.0
			line_color.a = 1.0
		elif(str(box.text) == "vehicle"):
			line_color.r = 1.0
			line_color.g = 1.0
			line_color.b = 0.0
			line_color.a = 1.0
		else:
			line_color.r = 1.0
			line_color.g = 1.0
			line_color.b = 1.0
			line_color.a = 1.0
		
		new_marker.color = line_color
		
		new_marker.lifetime = rospy.Duration.from_sec(0.1)
		new_marker.frame_locked = False
		new_marker.text = str(box.text)
		x1 = box.p1.x
		y1 = box.p1.y
		z1 = box.p1.z
		
		x2 = box.p2.x
		y2 = box.p2.y
		z2 = box.p2.z
		
		for i in range(16):
			new_point = Point()
			if(i == 0):
				new_point.x = x1
				new_point.y = y1
				new_point.z = z1
				
			elif(i == 1):
				new_point.x = x1
				new_point.y = y1
				new_point.z = z2
				
			elif(i == 2):
				new_point.x = x2
				new_point.y = y1
				new_point.z = z2
				
			elif(i == 3):
				new_point.x = x2
				new_point.y = y1
				new_point.z = z1
				
			elif(i == 4):
				new_point.x = x2
				new_point.y = y2
				new_point.z = z1
				
			elif(i == 5):
				new_point.x = x2
				new_point.y = y2
				new_point.z = z2
				
			elif(i == 6):
				new_point.x = x1
				new_point.y = y2
				new_point.z = z2
				
			elif(i == 7):
				new_point.x = x1
				new_point.y = y2
				new_point.z = z1
				
			elif(i == 8):
				new_point.x = x1
				new_point.y = y1
				new_point.z = z1
				
			elif(i == 9):
				new_point.x = x2
				new_point.y = y1
				new_point.z = z1
				
			elif(i == 10):
				new_point.x = x2
				new_point.y = y1
				new_point.z = z2
				
			elif(i == 11):
				new_point.x = x2
				new_point.y = y2
				new_point.z = z2
				
			elif(i == 12):
				new_point.x = x2
				new_point.y = y2
				new_point.z = z1
				
			elif(i == 13):
				new_point.x = x1
				new_point.y = y2
				new_point.z = z1
				
			elif(i == 14):
				new_point.x = x1
				new_point.y = y2
				new_point.z = z2
				
			elif(i == 15):
				new_point.x = x1
				new_point.y = y1
				new_point.z = z2
				
			else:
				rospy.loginfo("Visualizer Fail!")
				
			new_marker.points.append(new_point)
		markers.append(new_marker)
	marker_array.markers = markers
	
	marker_pub.publish(marker_array)
		
		
		

	#marker_pub.publish(markers)
	
	
def main():
    rospy.init_node('visualizer', anonymous=False)
    rospy.Subscriber("/tracked_objects", BoxArray, callback)
    rospy.spin()


if __name__ == '__main__':
   main()
