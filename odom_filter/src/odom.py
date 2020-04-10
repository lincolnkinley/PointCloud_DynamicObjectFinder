#!/usr/bin/env python
#-*- coding: utf-8 -*-

import rospy
import math

from dbw_mkz_msgs.msg import WheelSpeedReport
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose 

class Callback:
    def __ini__(self):
        self.Imu = None
        self.Wheel = None

    def callbackImu(self,msg):
        self.Imu = msg
        self.callback()

    def callbackWheel(self,msg):
        self.Wheel = msg
        self.callback()

    def callback(self):
        if self.Imu is not None and self.Wheel is not None:
            wheel_speed = []
            imu_quat = []
            transform_pub = rospy.Publisher('pose_info', Pose, queue_size=5)
            transform_msg = Pose()
            
            #parse the wheel information into the list wheel_speed
            data_wheel = str(self.Wheel)
            data_split_wheel = data_wheel.split('\n')
            for ii in range(6,10):
                speed = data_split_wheel[ii].split(':')
                wheel_speed.append(float(speed[1]))
            
            #parse the imu orientation quaternion into list imu_quat
            data_imu = str(self.Imu)
            data_split_imu = data_imu.split('\n')
	    for ii in range(7,11):
                imu = data_split_imu[ii].split(':')
                imu_quat.append(float(imu[1]))

            #get average speed r/s
            avg_speed = (wheel_speed[0] + wheel_speed[1] + wheel_speed[2] + wheel_speed[3])/4
        
            #time between wheel speed messages
            time = 0.1 

            #tire is 26.7inches diameter or 0.67818m
            #d = d/2*speed * time |||| r/s * m/rad * s = m
            total_displacement = 0.67818/2 * avg_speed * time

            #get the angle of the displacement
            theta = math.pi/4
 
            #filling out the quaternion
            transform_msg.position.x = total_displacement*math.cos(theta) 
            transform_msg.position.y = total_displacement*math.sin(theta)
            transform_msg.position.z = 0
            transform_msg.orientation.x = imu_quat[0]
            transform_msg.orientation.y = imu_quat[1]
            transform_msg.orientation.z = imu_quat[2]
            transform_msg.orientation.w = imu_quat[3]

            #print transform_msg
            transform_pub.publish(transform_msg)


def listener():
    rospy.init_node("transformation_generator")

    callback = Callback()

    rospy.Subscriber("/vehicle/wheel_speed_report", WheelSpeedReport, callback.callbackWheel)
    rospy.Subscriber("/imu/imu", Imu, callback.callbackImu)
    rospy.spin()

if __name__ =='__main__':
    listener()

