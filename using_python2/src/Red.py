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

def callback(msg):
    message = "recieved  " + msg.data
    print(message)
    


def talker():
    rospy.init_node('talker', anonymous=True)

    pub = rospy.Publisher('chatter', String, queue_size=10)
    #sub = rospy.Subscriber("chatter",String,callback)

    pub_marker = rospy.Publisher('marker_py', MarkerArray , queue_size=1)

    M = MarkerArray();
    marker = Marker()

    rate = rospy.Rate(10) # 10hz
    N = 5
    pi = (22.0/7)

    for i in range(N): #to include shepherd too
        marker = Marker()
        print("i " , i)
        marker.header.frame_id = "/my_py";
        marker.header.stamp = rospy.get_rostime();
        

        marker.ns = "py";
        
        marker.id = i;
        
        marker.type = marker.CUBE;
        
        marker.action = Marker.ADD;

        if(i < N):
            marker.pose.position.x = 2*i;            

            marker.pose.position.y = 0;
        
        marker.pose.position.z = 0;

        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 1.0;
        marker.pose.orientation.w = 1.0;
        
        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.2;
        
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 0.0;
   
        marker.lifetime = rospy.Duration();

        M.markers.append(marker)

    # M = Marker();
    # shape = M.ARROW;

    # M.header.frame_id = "/my_py";
    # M.header.stamp = rospy.get_rostime();
    
    # M.ns = "py";
    
    # M.id = 1;
    
    # M.type = shape;
    
    # M.action = Marker.ADD;

    
    # M.pose.position.x = 1.0;
    
    # #M.markers[i].pose.position.x = 2*(-j) + 1;

    # M.pose.position.y = 0.0;
    # #k++;
    # M.pose.position.z = 0;

    # x = M.pose.position.x
    # y =  M.pose.position.y

    # M.pose.orientation.x = 0.0;
    # M.pose.orientation.y = 0.0;
    # M.pose.orientation.z = 0.0;
    # M.pose.orientation.w = 1.0;
    
    # M.scale.x = 1.0;
    # M.scale.y = 0.2;
    # M.scale.z = 0.3;
    
    # M.color.r = 0.0;
    # M.color.g = 1.0;
    # M.color.b = 0.0;
    # M.color.a = 1.0;

    #M.lifetime = rospy.Duration();

    
    count = -pi;
    sign = 1;
    while not rospy.is_shutdown():
        #hello_str = "hello world %s" % rospy.get_time()
        #rospy.loginfo(hello_str)
        #print(hello_str)
        #pub.publish(hello_str)
        for i in range(N):
            M.markers[i].pose.position.x = M.markers[i].pose.position.x + 0.2;

            #M.markers[i].pose.orientation.w = 1
        pub_marker.publish(M)
        print("pos: " , M.markers[0].pose.position.x)
        #M.markers[0].pose.orientation.w = 
        #M.markers[0].pose.orientation.w = count + .01;
        count = count + sign/pi
        # #th = numpy.arctan2()
        #print("count: " , count  , " angle: " , )
        #if((count > 25 and sign == 1) or (count < -25 and sign == -1)):
            # sign = sign*(-1)
            # print("yoyoyoo " , sign)
        #M.pose.position.y = y + 0.1
        #y =  M.pose.position.y

        rate.sleep()




if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
 