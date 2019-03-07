#!/usr/bin/env python
# license removed for brevity
import rospy
import array
from std_msgs.msg import String
from visualization_msgs.msg import *
import numpy

def callback(msg):
    message = "recieved  " + msg.data
    print(message)
    

def talker():
    rospy.init_node('python3', anonymous=True)
    #pub = rospy.Publisher('chatter', String, queue_size=10)
    #sub = rospy.Subscriber("chatter",String,callback)


    pub_marker = rospy.Publisher('marker_py3', MarkerArray , queue_size=1)

    N = 10    

    rate = rospy.Rate(10) # 10hz

    M = MarkerArray()
    

    #M.markers.resize(3)

    x = []
    y = []


    shape = Marker.CUBE;
    for i in range(N):
        marker = Marker()
        print("i " , i)
        marker.header.frame_id = "/my_py3";
        marker.header.stamp = rospy.get_rostime();
        
        marker.ns = "py3";
        
        marker.id = i;
        
        marker.type = shape;
        
        marker.action = Marker.ADD;

        
        marker.pose.position.x = 2*i + 1;            

        marker.pose.position.y = 0.0;
        
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

        x.append(marker.pose.position.x)
        y.append(marker.pose.position.y)


        M.markers.append(marker)
        print("id = " , M.markers[i].id)

    #for i in range(3):
        #M.markers[i].action = marker.ADD


    while not rospy.is_shutdown():
        #hello_str = "hello world %s" % rospy.get_time()
        #rospy.loginfo(hello_str)
        #print("hello_str")
        #pub.publish(hello_str)
        #print(M.markers[0].id , " " , M.markers[1].id , "  " , M.markers[2].id , "tolo")
        pub_marker.publish(M)
        #print(M.markers[0].id , " " , M.markers[1].id , "  " , M.markers[2].id)
        for i in range(N):
            M.markers[i].pose.position.x = x[i] + 0
            M.markers[i].pose.position.y = y[i] + 1
            x[i] = M.markers[i].pose.position.x
            y[i] = M.markers[i].pose.position.y
            #print(i , " x " , M.markers[i].pose.position.x , " id " , M.markers[i].id)

        rate.sleep()




if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
