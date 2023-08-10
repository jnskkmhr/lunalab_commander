#!/usr/bin/env python3
# coding: UTF-8 

import rospy 
from std_msgs.msg import Float32
import numpy as np


class RandomizeLightIntensity:
    def __init__(self):
        self.pub = rospy.Publisher('/Lunalab/Projector/Intensity', Float32, queue_size=1)
        self.intensity = Float32()
        np.random.seed(0)
    
    def publish(self):
        self.intensity.data = np.random.uniform(40.0, 200.0)
        self.pub.publish(self.intensity)

if __name__ == '__main__':
    rospy.init_node('randomize_light_intensity')
    rate = rospy.Rate(10.0)
    randomizer = RandomizeLightIntensity()
    while not rospy.is_shutdown():
        randomizer.publish()
        rate.sleep()