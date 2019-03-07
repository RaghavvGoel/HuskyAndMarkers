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
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
#from tf.msg import tfMessage
#from 

husky1_x=10000;husky1_y=10000;
husky2_x=10000;husky2_y=10000;
husky_orientation1 = 0.0;
husky_orientation2 = 0.0;

yaw = 0.0;

def position_husky1(data1):
	#global husky_xy,husky_orientation
	global yaw;
	global husky1_x, husky1_y,husky_orientation1
	husky1_x = data1.pose.pose.position.x
	husky1_y = data1.pose.pose.position.y
	husky_orientation1 = data1.pose.pose.orientation
	q1_inv = numpy.zeros(4)
	q2 = numpy.zeros(4)

	# q1_inv[0] = 0.0
	# q1_inv[1] = 0.0
	# q1_inv[2] = 0.0
	# q1_inv[3] = 1.0 # Negate for inverse
	orientation_list = [husky_orientation1.x,husky_orientation1.y,husky_orientation1.z,husky_orientation1.w]
	# q2[0] = data1.pose.pose.orientation.x
	# q2[1] = data1.pose.pose.orientation.y
	# q2[2] = data1.pose.pose.orientation.z
	# q2[3] = data1.pose.pose.orientation.w

	(roll,pitch,yaw) = euler_from_quaternion(orientation_list)
	#qr = tf.transformations.quaternion_multiply(q2, q1_inv)			
	#print("yaw: " , yaw)

def position_husky2(data2):
	#global husky_xy,husky_orientation
	global husky2_x, husky2_y,husky_orientation2
	husky2_x = data2.pose.pose.position.x
	husky2_y = data2.pose.pose.position.y
	husky_orientation2 = data2.pose.pose.orientation.z

def main():
	rospy.init_node('gazebo_rviz_test', anonymous=True)
	pub_vel_husky1 = rospy.Publisher('/Husky1/husky_velocity_controller/cmd_vel',Twist, queue_size=10)
	pub_vel_husky2 = rospy.Publisher('/Husky2/husky_velocity_controller/cmd_vel',Twist, queue_size=10)

	#pub_rviz_bot1 = 

	rospy.Subscriber('/Husky1/husky_velocity_controller/odom', Odometry, position_husky1)
	rospy.Subscriber('/Husky2/husky_velocity_controller/odom', Odometry, position_husky2)

	pub_marker = rospy.Publisher('multi_sheep_sheperd200', MarkerArray , queue_size=1)

	rate = rospy.Rate(10) # 10hz

	H1 = Twist()
	H2 = Twist()
	M = MarkerArray()
	N = 1; nos = 0; nod = 0;
	shape = Marker.CUBE;
	ini_x = 0.0; ini_y = 0.0;
	nos = 1; #to keep track of the orgin


	for i1 in range(N+nos+nod): #to include shepherd too
		marker = Marker()
		#print("i1 " , i1)
		marker.header.frame_id = "/base_link";
		marker.header.stamp = rospy.get_rostime();
		

		marker.ns = "mul_shp";
		
		marker.id = i1;
		
		marker.type = shape;
		
		marker.action = Marker.ADD;

		if(i1 < N):
			marker.pose.position.x = ini_x;            

			marker.pose.position.y = ini_y;
		
		marker.pose.position.z = 0;

		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;
		
		marker.scale.x = 1.0;
		marker.scale.y = 1.0;
		marker.scale.z = 1.0;
		
		#marker.color.r = i1/10.0;
		marker.color.r = 0.0;
		#marker.color.g = 2*i1/10.0;
		marker.color.g = 1.0;
		marker.color.b = 0.0;
		marker.color.a = 1.0;
		if(i1 >= N):
			marker.pose.position.x = ini_x;            

			marker.pose.position.y = ini_y;
		
			marker.scale.x = 0.5;
			marker.scale.y = 0.5;
			marker.scale.z = 1.5;
			marker.color.r = 1.0;
			marker.color.g = 0.0;
   
		marker.lifetime = rospy.Duration();

		# if(i1 >= N and i1<N+nos):
		# 	marker.ns = "sheperd"
		# 	marker.pose.position.x = Sh[i1-N][0];
		# 	marker.pose.position.y = Sh[i1-N][1];
		# 	#marker.color.r = 2*i1/10.0;
		# 	marker.color.r = i1/4.0;
		# 	marker.color.g = 0.0;
		# 	#marker.color.b = i1/5.0;
		# 	marker.color.b = (1.0 - i1/4.0);

		# #marker.color.r = 0.0;    
		# if(i1 >= N+nos):
			
		# 	marker.ns = "Point"
		# 	marker.type = Marker.CYLINDER
		# 	marker.scale.x = 2*threshold;
		# 	marker.scale.y = 2*threshold;
		# 	if(l == 0):
		# 		marker.pose.position.x = destination1[0]
		# 		marker.pose.position.y = destination1[1]
		# 		marker.pose.position.x = -128
		# 		marker.pose.position.y =  225                    
		# 		l = 1
		# 	elif(l == 1):
		# 		marker.pose.position.x = destination2[0]
		# 		marker.pose.position.y = destination2[1]
		# 		marker.pose.position.x = 164
		# 		marker.pose.position.y = 187                    
		# 		marker.scale.x = 2*threshold;
		# 		marker.scale.y = 2*threshold;                    
		# 		l = 2 
		# 	else:
		# 		marker.pose.position.x = 21
		# 		marker.pose.position.y = 231
		# 		#marker.color.r = 0.8
		# 		marker.scale.x = 2*(threshold+5);
		# 		marker.scale.y = 2*(threshold+5);
		# 	marker.color.b = 1.0;
			

		# 	marker.scale.z = 0.5;   

		M.markers.append(marker)
			#print("id = " , M.markers[i1].id , "ini pos " , x[0][i1%N] , " , " , y[0][i1%N])

			#m=numpy.size(initpoints[0])	

	#ini_x = 0.0
	#ini_y = 0.0
	ini_angle = 0.0	
	given_angle = pi/2;
	q1_inv = numpy.zeros(4)
	q2 = numpy.zeros(4)
	time1 = rospy.get_time()
	while not rospy.is_shutdown():
		global husky1_x, husky1_y, husky_orientation1,husky2_x, husky2_y, husky_orientation2 
		global yaw;

		pub_marker.publish(M)

		#angle_in_rviz = angle_gazebo_to_rviz(yaw, ini_angle) #dont need when robot ini heading coincides with the pos x axis
		#rx,ry = gazebo_to_rviz(husky1_x, husky1_y, yaw)
		
		#H1.linear.x  = 0.0
		#H2.linear.x  = 0.5
		#H1.linear.y  = -1.0
		#print("diff: " , yaw - given_angle)
		#if(abs(ini_angle - given_angle) > 0.1):
			#H1.angular.z = (yaw - given_angle)/(2*pi);
		#if(rospy.get_time() - time1 < 3.0):
			#H1.angular.z = 0.2
		# else:
		# 	H1.angular.z = 0.0	

		#current_angle = (pi/2 + yaw)%2*pi
		#pub_vel_husky1.publish(H1)
		#print("yaw: " , pi/2 + yaw)
		#pub_vel_husky2.publish(H2)

		#print("husky1_x:" , husky1_x , " husky1_y:" , husky1_y , " ")
		#print("orientation1:" , husky_orientation1)
		#print("in degress:", husky_orientation1*180/pi)

		#print("husky2_x:" , husky2_x , " husky2_y:" , husky2_y , " ")
		#print("orientation2:" , husky_orientation2)
		#d = numpy.sqrt((pow(ini_y - husky1_y,2)) + (pow(ini_x-husky1_x,2)))
		#theta = yaw - ini_angle

		#dx = d*numpy.cos(theta)
		#dy = d*numpy.sin(theta)
		#print("gazebo position:"  , husky1_x," " ,husky1_y, " " ,yaw*180/pi)
		#print("rviz position should be same as gazebo") 
		a,a,t1 = euler_from_quaternion((0.000148517548168,-1.89087259865e-06,0.000126522262906 ,0.999999980966))
		print("t1: " , degrees(t1))

		a,a,t2 = euler_from_quaternion((0.0, 0.0,  0.706906369979 , 0.707307135608))

		print("t2: " , degrees(t2))
		M.markers[0].pose.position.x = husky1_x  #+ ini_x
		M.markers[0].pose.position.y = husky1_y  #+ ini_y
		#print("marker:")
		#print(M.markers[0].pose.position.y)
		#ini_x = husky1_x;
		#ini_y = husky1_y;



		

		rate.sleep()





if __name__ == '__main__':
	#x = []
	#y = []
	try:
		main()
	except rospy.ROSInterruptException:
		pass
