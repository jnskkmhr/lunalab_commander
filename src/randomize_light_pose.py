#!/usr/bin/env python3
# coding: UTF-8 

import rospy
import numpy as np 
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose


class RandomizeLightPose(object):
    def __init__(self):
        self.pub = rospy.Publisher('/Lunalab/Projector/Pose', Pose, queue_size=1)
        self.pose = Pose()
        np.random.seed(0)
    
    def publish(self):
        # default : x = 2.9, y = -0.5, z = 1.3
        self.pose.position.x = np.random.uniform(2.0, 4.0)
        self.pose.position.y = -0.5
        self.pose.position.z = np.random.uniform(0.5, 1.5)

        # euler angle to quaternion
        # default all 0
        # roll = 0.0
        # pitch = 0.0
        # yaw = 0.0

        roll = np.random.uniform(-10.0, 0.0)
        pitch = 0.0
        yaw = np.random.uniform(-20.0, 20.0)

        roll = np.deg2rad(roll)
        pitch = np.deg2rad(pitch)
        yaw = np.deg2rad(yaw)

        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        self.pose.orientation.x = qx
        self.pose.orientation.y = qy
        self.pose.orientation.z = qz
        self.pose.orientation.w = qw
        self.pub.publish(self.pose)

if __name__ == '__main__':
    rospy.init_node('randomize_light_pose')
    rate = rospy.Rate(10.0)
    randomizer = RandomizeLightPose()
    while not rospy.is_shutdown():
        randomizer.publish()
        rate.sleep()