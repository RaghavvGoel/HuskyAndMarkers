#!/usr/bin/env python
# license removed for brevity
import rospy
import array
from std_msgs.msg import String
from visualization_msgs.msg import *
import numpy
import matplotlib.pyplot as plt
from matplotlib import path
from math import *
import copy
from random import randint

pi =  22.0/7.0

m = MarkerArray()

def GetMarkers(msg):
	global m
	m = msg;

def main():
    rospy.init_node('Leader', anonymous=True)

    sub_marker = rospy.Subscriber('leader', MarkerArray , GetMarkers)
    #sub_cursor = rospy.Subscriber('/initialpose' , PoseWithCovarianceStamped , get_pos)
    pub_marker = rospy.Publisher('LeaderS1',MarkerArray,queue_size = 1)

    rate = rospy.Rate(10) # 10hz



    while not rospy.is_shutdown():

        pub_marker.publish(m)

        rate.sleep()

if __name__ == '__main__':
    #x = []
    #y = []
    try:
        main()
    except rospy.ROSInterruptException:
        pass
