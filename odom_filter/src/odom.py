#!/usr/bin/env python
#-*- coding: utf-8 -*-

import rospy
import math

from tf.transformations import euler_from_quaternion

from dbw_mkz_msgs.msg import WheelSpeedReport
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose 

class Callback:
    def __init__(self):
        self.Imu = None
        self.Wheel = None
        self.transform_pub = rospy.Publisher('pose_info', Pose, queue_size=5)
        self.x = 0
        self.y = 0

    def callbackImu(self,msg):
        self.Imu = msg

    def callbackWheel(self,msg):
        self.Wheel = msg
        self.callback()

    def callback(self):
        if ((self.Imu != None) and (self.Wheel != None)):
            wheel_speed = []
            imu_quat = []
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
            time = 0.01 

            #tire is 26.7inches diameter or 0.67818m
            #d = d/2*speed * time |||| r/s * m/rad * s = m
            displacement = 0.67818/2 * avg_speed * time

            #get the angle of the displacement
            roll, pitch, yaw = euler_from_quaternion((imu_quat[0], imu_quat[1], imu_quat[2], imu_quat[3]))
 
            #filling out the quaternion
            self.x = self.x + (displacement*math.cos(yaw))
            self.y = self.y + (displacement*math.sin(yaw))

            transform_msg.position.x = self.x
            transform_msg.position.y = self.y
            transform_msg.position.z = 0
            transform_msg.orientation.x = imu_quat[0]
            transform_msg.orientation.y = imu_quat[1]
            transform_msg.orientation.z = imu_quat[2]
            transform_msg.orientation.w = imu_quat[3]

            #print transform_msg
            self.transform_pub.publish(transform_msg)


def listener():
    rospy.init_node("transformation_generator")

    callback = Callback()

    rospy.Subscriber("/vehicle/wheel_speed_report", WheelSpeedReport, callback.callbackWheel) # publishes about 100 times per sec
    rospy.Subscriber("/imu/imu", Imu, callback.callbackImu) # publishes about 155 times per sec
    rospy.spin()

if __name__ =='__main__':
    listener()

