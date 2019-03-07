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
#geometry_msgs/PoseWithCovarianceStamped
from geometry_msgs.msg import PoseWithCovarianceStamped

rviz_pos = [10000 , 10000]

def get_pos(msg):
	global rviz_pos
	rviz_pos[0] = msg.pose.pose.position.x
	rviz_pos[1] = msg.pose.pose.position.y


def main():
    rospy.init_node('Leader', anonymous=True)

    pub_marker = rospy.Publisher('leader', MarkerArray , queue_size=1)
    sub_cursor = rospy.Subscriber('/initialpose' , PoseWithCovarianceStamped , get_pos)

    rate = rospy.Rate(10) # 10hz

    N = 1
    nos = 0;
    Xstartpos = 0
    Ystartpos = 0
    threshold = 5

    #function [ X,Y,vels] = puneetsCouzins(X,Y,vels,Rrep,Rori,Ratt,s,dt,phi,omega)
    k = 1
    x=Xstartpos+numpy.random.rand(1,N)*k - (k/2) #swarm initialization
    y=Ystartpos+numpy.random.rand(1,N)*k - (k/2)  #gives random values from [0,1)
    initpoints=numpy.concatenate((x,y),axis=0)
    vels=numpy.zeros((N,2))
    for i in range(N):
        vels[i][0] = 1
    #vels[1] = 1
    vels_temp = vels
    unitvec=numpy.zeros((2,N))
    #temppos = numpy.zeros((N,2))
    #tempdi = numpy.zeros((N,2))
    Rrep = 2.5
    Rori = 5
    Ratt = 10
    s = 1
    dt = 0.2    
    phi = 60*(pi)/180
    omega = 1.0

    destination = [0,100]

    M = MarkerArray()    
    
    shape = Marker.CUBE;
    for i in range(N+nos+1): #to include shepherd too
        marker = Marker()
        print("i " , i)
        marker.header.frame_id = "/multi_sheep_leader";
        marker.header.stamp = rospy.get_rostime();
        

        marker.ns = "mul_leader";
        
        marker.id = i;
        
        marker.type = shape;
        
        marker.action = Marker.ADD;

        if(i < N):
            marker.pose.position.x = x[0][i];            

            marker.pose.position.y = y[0][i];
        
        marker.pose.position.z = 0;

        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        
        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;
        
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
   
        marker.lifetime = rospy.Duration();

        if(i >= N and i<N+nos):
            marker.ns = "sheperd"
            marker.pose.position.x = Sh[i-N][0];
            marker.pose.position.y = Sh[i-N][1];
            marker.color.r = 1.0;
            marker.color.g = 0.0;

        if(i >= N+nos):
            marker.ns = "Point"
            marker.type = Marker.CYLINDER
            marker.pose.position.x = destination[0]
            marker.pose.position.y = destination[1]
            marker.color.b = 0.0;
            marker.color.r = 0.0;
            marker.scale.x = 2*threshold;
            marker.scale.y = 2*threshold;
            marker.scale.z = 0.5;   

        M.markers.append(marker)    


    global rviz_pos    
    while not rospy.is_shutdown():

        pub_marker.publish(M)

        print("initpoints: ",initpoints )
        print("rviz_pos: " , rviz_pos)
        if(rviz_pos[0] != 10000 and rviz_pos[1] != 10000):
        	#dr = (initpoints[:,0]-rviz_pos)/numpy.linalg.norm((initpoints[:,0]-rviz_pos),axis = 0)
        	print("in first if")
        	
        	theta = numpy.arctan((initpoints[1,0] - rviz_pos[1])/(initpoints[0,0] - rviz_pos[0]))

        	initpoints[0,0] = initpoints[0,0] + numpy.cos(theta)*dt*s
        	initpoints[1,0] = initpoints[1,0] + numpy.sin(theta)*dt*s

        	M.markers[0].pose.position.x = initpoints[0,0]
        	M.markers[0].pose.position.y = initpoints[1,0]

        if(sqrt(pow(initpoints[0][0] - rviz_pos[0],2) + pow(initpoints[1][0] - rviz_pos[1],2)) <= threshold):
        	rviz_pos = [10000 , 10000] # to make the agent stop	

        rate.sleep()


if __name__ == '__main__':
    #x = []
    #y = []
    try:
        main()
    except rospy.ROSInterruptException:
        pass
 