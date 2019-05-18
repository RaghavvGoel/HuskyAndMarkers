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


def get_q(no_of_agents,agents,i,s):
	#temp_q = numpy.zeros(1,2);
	for j in range(no_of_agents): # if you'll take summation then mind that temp_q isnt in on the RHS
		if(j != i):
			#print("agents: " , agents )
			#print("agents[:,i] " , agents[:,i])
			#print("difference: " , agents.T - agents[:,i])
			#print("NORM: " , numpy.linalg.norm(agents.T - agents[:,i],axis = 1))
			norm_of_inter_agent_dist = numpy.linalg.norm(agents.T - agents[:,i],axis = 1)
			total_of_all_norms = 0;
			for k in range(numpy.size(norm_of_inter_agent_dist)):
				total_of_all_norms = total_of_all_norms + norm_of_inter_agent_dist[k];
			#print("total : " , total_of_all_norms)	
			#temp_q = s[0][i]*(agents[:,i] - agents[:,j])/total_of_all_norms
			#FOR TWO AGENTS
			#print("norm for two agents: " , numpy.linalg.norm(agents[:,j] - agents[:,i],axis = 0))
			temp_q = s[0][i]*(agents[:,j] - agents[:,i])/numpy.linalg.norm(agents[:,j] - agents[:,i],axis = 0)
	return temp_q
	
def get_q_NAgents(no_of_agents,agents,i,radius, prev_qi, s):
	#for j in range(no_of_agents):
	#	if(j != i):
	inter_agent_dist = numpy.linalg.norm(agents.T - agents[:,i],axis = 1)
	
	#neighbours = numpy.where(inter_agent_dist != 0)
	sort_dist = numpy.argsort(inter_agent_dist)
	#let's take the first 3
	no_nearest_neigh = 3	
	# if(numpy.size(numpy.where(inter_agent_dist <= radius/2)) - 1 > no_nearest_neigh):
	# 	no_nearest_neigh = numpy.size(numpy.where(inter_agent_dist < radius/2))-1
	# 	no_nearest_neigh = 3;

	#print("no_nearest_neigh " , no_nearest_neigh)
	if(no_of_agents > 2):
		neighbours = sort_dist[1:no_nearest_neigh+1].reshape([1,no_nearest_neigh])
	else:
		neighbours = sort_dist[1:].reshape([1,1])

	print("neighbours: " , neighbours , " shape:" , numpy.shape(neighbours))		
	#the first entry is of i itself, ignore that
	total_of_all_norms = 0;
	temp_qi = 0;
	temp_norm_qi = 0;
	for j in range(numpy.size(neighbours)):
		#if(i != j):
		qij_not = (agents[:,neighbours[0][j]] - agents[:,i])/numpy.linalg.norm(agents[:,neighbours[0][j]] - agents[:,i],axis = 0)
		check_dot = numpy.dot(qij_not , prev_qi)
		if(check_dot >= 0):
			qij = qij_not
		else:
			qij = qij_not

		temp_qi += qij
		temp_norm_qi += numpy.linalg.norm(qij)

	return s[0][i]*temp_qi/temp_norm_qi


def get_n(qi):
	return [-qi[1],qi[0]] #R = [0 1;-1 0] ,can also take the dot product

def get_theta(qi):
	#print("qi: " , qi)
	#print("theta= ", numpy.arctan2(qi[0],qi[1]))
	theta = numpy.arctan2(qi[1],qi[0])
	#now theta will change depending on the quadrant
	# if(qi[0] < 0 and qi[1] > 0):
	# 	#2nd quad => theta = pi + theta | theta E -pi/2 to pi/2
	# 	theta = pi + theta;
	# elif(qi[0] < 0 and qi[1] < 0):
	# 	#3rd quad => theta = -pi + theta
	# 	theta = -pi - theta
	return theta	
		


def get_vq(no_of_agents, agents , expected_inter_agent_dist, i , k1, qi, s):	 	
	inter_agent_dist = numpy.linalg.norm(agents.T - agents[:,i],axis = 1)
	
	#neighbours = numpy.where(inter_agent_dist != 0)
	sort_dist = numpy.argsort(inter_agent_dist)
	#let's take the first 3
	no_nearest_neigh = 3

	if(no_of_agents > 2):
		neighbours = sort_dist[1:no_nearest_neigh+1].reshape([1,no_nearest_neigh])
	else:
		neighbours = sort_dist[1:].reshape([1,1])	

	vqi = 0;	
	for j in range(numpy.size(neighbours)):
		#print("j = " , j , " i = " , i)
		#if(j != i):
		#print("agents :qi " , qi , " qi[0] " , qi[0])
		inter_agent_dist = agents[:,neighbours[0][j]] - agents[:,i];
		#print("inter_agent_dist = " , inter_agent_dist)
		#print("dot product: " , numpy.dot(inter_agent_dist , qi))
		#vqi = (1/s[0][i])*k1*(numpy.dot(inter_agent_dist , qi) - expected_inter_agent_dist[0][i])
		vqi += k1*(numpy.dot(inter_agent_dist , qi) - expected_inter_agent_dist[i][neighbours[0][j]])
	# if(abs(vqi) <= 1.25):				
	# 	return vqi
	# elif(vqi > 1.25):
	# 	return 1.25
	# else:
	# 	return -1.25
	# if(i == 0):
	# 	return 1.0
	# else:
	# 	return -1.0		
	#return 0.0		
	return vqi*(1.0)

def get_vn(i,k,C,z,pos):
	y = z + exp(-(pow(pos[0],2) + pow(pos[1],2)))
	vni = k*y + C #max velocity = 20, so let's scale by 10 to get max vni = 2
	# if(vni < 1.25):
	# 	return vni;
	# else:
	# 	return 1.25
	#return 0.0	
	# if(i == 0):
	# 	return 2.0
	# else:
	# 	return 2.0	
	return vni*(1.0);

def get_Vx(vqi,vni,theta):
	#print("vqi: " , vqi , " vni: " , vni , " in getVx")
	#print("sin: " , numpy.sin(theta) , " cos: " , numpy.cos(theta))
	return vqi*numpy.cos(theta) - vni*numpy.sin(theta)
	
def get_Vy(vqi,vni,theta):
	#print("vqi: " , vqi , " vni: " , vni , " in getVyyyyy")
	return vqi*numpy.sin(theta) + vni*numpy.cos(theta)

def get_conc(agent_pos,source, thickness_of_circle, r_small_circle, r_big_circle, no_of_circles, center):
	dist_from_source_centre = numpy.linalg.norm(agent_pos - source , axis = 0)
	# if(dist_from_source_centre <= r_small_circle):
	# 	#at the source
	# elif(dist_from_source_centre > r_small_circle and dist_from_source_centre < r_small_circle + thickness_of_circle)	
	
	# if(dist_from_source_centre <= r_big_circle and dist_from_source_centre > r_small_circle):
	# 	current_circle_of_agent = ceil((dist_from_source_centre - r_small_circle)*(1.0)/thickness_of_circle)
	# 	#print("current_circle_of_agent : " , current_circle_of_agent)
	# 	#scalar_value = no_of_circles % current_circle_of_agent; #nearer the source more the value	
	# 	scalar_value = current_circle_of_agent
	# 	return scalar_value #no of circles are 20
	# elif(dist_from_source_centre <= r_small_circle):
	# 	scalar_value = no_of_circles
	# 	return scalar_value	
	# else:
	# 	return 0;
	#return dist_from_source_centre
	#return 1/dist_from_source_centre

	# if(dist_from_source_centre > 5.0):
	# 	return 1/dist_from_source_centre
	# else:
	# 	return 0.4	


	#z(r) = (r.T)r= r^2	, r is the postion
	return numpy.linalg.norm(agent_pos - center)

def main():
	rospy.init_node('source', anonymous=True)
	pub_marker = rospy.Publisher('agents', MarkerArray , queue_size=1)
	pub_marker_field = rospy.Publisher('field', MarkerArray , queue_size=1)	

	rate = rospy.Rate(10) # 10hz
	no_of_agents = 4
	Xstart = 30.0
	Ystart = 30.0
	agentx = Xstart + numpy.random.rand(1,no_of_agents);
	agenty = Ystart + numpy.random.rand(1,no_of_agents);
	#creatng gap between the agents from th estart only
	# agentx[0][1] = agentx[0][1] + 5.0
	# agenty[0][1] = agenty[0][1]
	agents = numpy.concatenate((agentx,agenty),axis=0)

	#acc to papaer
	C = 0
	k = 1
	k1 = 1
	center_of_source = [0,0]
	field_length = 100
	field_width =  100
	F = MarkerArray()

	point_shape = Marker.CYLINDER
	
	source_x = center_of_source[0]
	source_y = center_of_source[1]
	source = [source_x,source_y]
	r_big_circle = 100.0
	r_small_circle = 5.0
	no_of_circles = 20
	thickness_of_circle = (r_big_circle - r_small_circle)/no_of_circles
	red_circles = no_of_circles/2
	blue_circles = no_of_circles/2


	for i1 in range(no_of_circles):
		marker = Marker()

		marker.header.frame_id = "/multi_sheep_sheperd";
		marker.header.stamp = rospy.get_rostime()

		marker.ns = "field"

		marker.id = i1;

		marker.type = point_shape;
		
		marker.action = Marker.ADD;

		marker.color.a = 1.0

		marker.pose.position.x = center_of_source[0]
		marker.pose.position.y = center_of_source[1]
		marker.pose.position.z = 0.0

		marker.scale.x = 5  #(r_big_circle - thickness_of_circle*i1)*2
		marker.scale.y = 5   #(r_big_circle - thickness_of_circle*i1)*2
		marker.scale.z = 0.5
		# if(i1 < red_circles):
		# 	marker.color.r = i1*(1.0)/red_circles
		# else:
		# 	marker.color.r = 1.0
		# 	marker.color.b = (i1-red_circles)*(1.0)/blue_circles
		marker.color.b = 1.0;
		marker.lifetime = rospy.get_rostime()
		F.markers.append(marker)		


	# for i1 in range(field_length):
	# 	for j1 in range(field_width):
	# 		if(j1%2 == 0):
	# 			marker.header.frame_id = "/multi_sheep_sheperd";
	# 			marker.header.stamp = rospy.get_rostime()

	# 			marker.ns = "field"

	# 			marker.id = i1;

	# 			marker.type = point_shape

	# 			marker.action = Marker.ADD;

	# 			marker.color.a = 1.0
			
	# 			marker.pose.position.x = field_width - j1*level
	# 			marker.pose.position.y = field_length - i1*level
	# 			marker.pose.position.z = 0.0

	# 			marker.scale.x = 

	M = MarkerArray()

	shape = Marker.CUBE;

	for i1 in range(no_of_agents):
		marker = Marker()

		marker.header.frame_id = "/multi_sheep_sheperd";
		marker.header.stamp = rospy.get_rostime()

		marker.ns = "agents"

		marker.id = i1;

		marker.type = Marker.CYLINDER;
		
		marker.action = Marker.ADD;

		marker.color.a = 1.0

		if(i1 < no_of_agents):			
			marker.pose.position.x = agents[0][i1];
			marker.pose.position.y = agents[1][i1];
			marker.pose.position.z = 0;

			marker.scale.x = 3.0/2;
			marker.scale.y = 3.0/2;
			marker.scale.z = 1.0;

			marker.color.g = 1.0 #*(1-i1)
			#marker.color.r = i1*0.5

		marker.lifetime = rospy.Duration();
		M.markers.append(marker)			

	#expected distance
	expected_inter_agent_dist = numpy.zeros([no_of_agents,no_of_agents]) #1xN
	#expected_inter_agent_dist[:no_of_agents/2][0] =  5.0
	#expected_inter_agent_dist[0][1] = -5.0
	dist = 5.0/2;
	for i in range(no_of_agents):
		for j in range(i+1,no_of_agents):			
			expected_inter_agent_dist[i][j] = dist
		
		for j in range(i-1,0,-1):
			expected_inter_agent_dist[i][j] = -dist;		
	
	q = numpy.zeros([no_of_agents,2])
	n = numpy.zeros([no_of_agents,2])
	s = numpy.ones([1,no_of_agents]) #assuming all are same direction, we dont care of the diagnol
	vq = numpy.zeros(no_of_agents)
	vn = numpy.zeros(no_of_agents)
	Vx = numpy.zeros(no_of_agents)
	Vy = numpy.zeros(no_of_agents)	
	ini_s = numpy.ones([1,no_of_agents])
	theta = numpy.zeros(no_of_agents) #angle with pos x
	alpha = numpy.zeros(no_of_agents) #alpha in the paper
	alpha_dot = numpy.zeros(no_of_agents)
	#for source seeking, si = -sj, hence making s[0][1] = -1 as s[0][0] = 1
	if(no_of_agents == 2):
		s[0][1] = -s[0][0] #made them same for testing circular movement
		ini_s[0][1] = -ini_s[0][0]

	time_step = 0.05
	delta1 = pow(10,-4)
	delta2 = 1.0
	count_of_algo_fail = 0
	time_gap = numpy.zeros([1,no_of_agents]) #this represents after how much time we take the next conc. reading to compare and make decision
	time_after_conc_compared = 1
	radius = 7.5
	#time_gap[0][0] = 5
	#time_gap[0][1] = 5

	start_time = rospy.get_rostime().to_sec()
	while not rospy.is_shutdown():		
		pub_marker_field.publish(F)

		for i in range(no_of_agents):
			M.markers[i].pose.position.x = agents[0][i];
			M.markers[i].pose.position.y = agents[1][i];
			#M.markers[i].scale.x = 10*get_conc(agents[:,i], source, thickness_of_circle, r_small_circle, r_big_circle, no_of_circles)
			#M.markers[i].scale.y = 10*get_conc(agents[:,i], source, thickness_of_circle, r_small_circle, r_big_circle, no_of_circles)

		pub_marker.publish(M)
		
		#print("agents position : " , agents)	

		for i in range(no_of_agents):
			print("i: " , i , " s_i: " , s[0][i] , " % " , numpy.arctan2(-1.0,-1.0))
			ini_s[0][0] = s[0][0];
			ini_s[0][1] = s[0][1];
			# if(no_of_agents == 2):
			# 	q[i] = get_q(no_of_agents,agents,i,s)
			# else:
			q[i] = get_q_NAgents(no_of_agents,agents,i,radius,q[i],s)
			#for j in range(no_of_agents - 1):
			#	temp_q = get_q(i,j,a); #will give the axis q
				#we can also put the inner loop in the function get_q as later on consensus is used for finding q
			#q[i] = temp_q # this 2x1, has x and y both
			n[i] = get_n(q[i])
			#n[i] = temp_n , #do we even need n ?? 
		
			theta[i] = get_theta(q[i]) # angle with x axis

			conc_t = get_conc(agents[:,i], source, thickness_of_circle, r_small_circle, r_big_circle, no_of_circles, center_of_source)
			vq[i] = get_vq(no_of_agents, agents, expected_inter_agent_dist, i, k1, q[i], s) #k1*(numpy.dot([a[0][j] - a[0][i] , a[1][j] - a[1][i]],[q[i]]) - a_expected[i,j])
			vn[i] = get_vn(i,k,C,conc_t,agents[:,i]) # y = 0
			print("q: " , q[i])
			print("n: " , n[i])
			print("Vn: " , vn[i])
			print("Vq: " , vq[i])
			print("theta: " , theta[i]/pi*180)
			Vx[i] = get_Vx(vq[i], vn[i], theta[i]) #q[i][0]*numpy.cos(temp_theta) - n[i][0]*numpy.sin(temp_theta)
			Vy[i] = get_Vy(vq[i], vn[i], theta[i]) #q[i][0]*numpy.sin(temp_theta) + n[i][0]*numpy.cos(temp_theta)		
			print("Vxi: " , Vx[i])
			print("Vyi: " , Vy[i])
			# vq[i] = vqi
			# vn[i] = vni
			# Vx[i] = Vxi
			# Vy[i] = Vyi
			#print("vqi : " , vqi , " vni : " , vni )
			#print("Vxi : " , Vxi , " Vyi : " , Vyi)
			#CHANGE IN POSITION:
			agents[0][i] = agents[0][i] + Vx[i]*time_step
			agents[1][i] = agents[1][i] + Vy[i]*time_step

			time_gap[0][i] = time_gap[0][i] + 1			


			#swaitching the directions according to conc:
			change = get_conc(agents[:,i], source, thickness_of_circle, r_small_circle, r_big_circle, no_of_circles, center_of_source) - conc_t

			print("change :", change)
			if(change > 0 and time_gap[0][i] % 2 == 0):
				#meaning that agent moving towards high conc, we change the s
				#if(s[0][0] != s[0][1]):
				print("changing the direction:")
				print("initial s: ", s)
				s[0][i] = -1*(s[0][i])			
				ini_s = s
				print("new s: " , s)
					

			#Switching Conditions
			print("time_gap : " , time_gap[0][i])			
			if(time_gap[0][i] % time_after_conc_compared == 0):
				#below condition implies, forward movment
				if(s[0][1] == -s[0][0]): #as only two agents hence we can do this
					conc_t_1 = get_conc(agents[:,i], source, thickness_of_circle, r_small_circle, r_big_circle, no_of_circles, center_of_source)
					if(abs(conc_t_1 - conc_t) <= delta1):
						s[0][i] = s[0][i]*(-1)

				else:
					conc_t_1 = get_conc(agents[:,i], source, thickness_of_circle, r_small_circle, r_big_circle, no_of_circles, center_of_source)
					if(abs(conc_t_1 - conc_t) > delta2):
						s[0][i] = s[0][i]*(-1)		

				print("change in conc: " , abs(conc_t_1 - conc_t))	

		if(s[0][0] != ini_s[0][0] and s[0][1] != ini_s[0][1]):#algo fails when both agents change sign together
			count_of_algo_fail = count_of_algo_fail + 1;
		print("count_of_algo_fail: " , count_of_algo_fail)		

		agent_in = 0;
		threshold = 2.0/2;
		for i in range(no_of_agents):
			if(numpy.linalg.norm(agents[:,i] - center_of_source) < threshold):
				agent_in += 1;
		if(agent_in >= floor(0.9*no_of_agents)):
			print("STOPPPPPP")
			break;
				
		rate.sleep();	

	stop_time = rospy.get_rostime().to_sec()	
	print("total time taken: " , stop_time - start_time)

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass