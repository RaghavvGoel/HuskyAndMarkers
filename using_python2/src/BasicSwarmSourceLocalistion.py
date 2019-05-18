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
from geometry_msgs.msg import PoseWithCovarianceStamped

def stepspertimestep(currentpoints,initialpoints,steps): # to limit the step taken per unit time
#steps: steps to be taken towards currentpoints from initialpoints
    difference=currentpoints-initialpoints
    norm=numpy.linalg.norm(difference,axis=0)
    l=numpy.shape(difference)
    #print("in stepper  L : " , l , " llolololooloolooololo " ) 
    direction=numpy.zeros((2,l[1]))
    for i in range(l[1]):
        step=steps
        if norm[i]<steps:
            step=norm[i]
        if norm[i]==0:
            unitvec=[0,0]
        else:
            unitvec=[difference[0][i]/norm[i],difference[1][i]/norm[i]]
            direction[0,i]=unitvec[0]
            direction[1,i]=unitvec[1]
        currentpoints[0][i]=initialpoints[0][i]+unitvec[0]*step
        currentpoints[1][i]=initialpoints[1][i]+unitvec[1]*step
    return currentpoints,direction

def moveSheperd( point1,point2,intensity,direction):  #can we treat this similar to leader ??
    #print("point1 : " , point1, "  point2 " , point2, " PPPPPPPPPPPPPPPP")
    point1 = numpy.asarray(point1).reshape(1,2)
    point2 = numpy.asarray(point2).reshape(1,2)   
    if numpy.shape(point1)[0]!=0 and numpy.shape(point2)[0]!=0:
        #difference=numpy.asarray(point1).reshape(1,2)-numpy.asarray(point2).reshape(1,2)
        difference=point1-point2
        #print("point1 : " , point1, "  point2 " , point2, " pppppppppp")   
        norm = numpy.zeros(numpy.shape(point1)[0])
        for i in range(numpy.shape(point1)[0]):
            norm[i] = numpy.sqrt(numpy.square(difference[i][0]) + numpy.square(difference[i][1]))
        l=numpy.shape(point2)
        #print("numpy.shape(point1) " , numpy.shape(point1) , "  ",point1[0], " " , norm , )
        #if numpy.shape(point1)[0]>1:
        for i in range(numpy.shape(point2)[0]):
            #print(i , "  " , " iiiiiiiiiiiiiiiiiiiiiiiiiiiii")
            if norm[i]==0:
                unitvec=[0,0]
            else:
                unitvec=[difference[i][0]/norm[i],difference[i][1]/norm[i]]
            #print("unitvec " , unitvec , " " , )    
            point1[i][0]=point1[i][0]-direction*(intensity)*unitvec[0]
            point2[i][0]=point2[i][0]+direction*(intensity)*unitvec[0]
            point1[i][1]=point1[i][1]-direction*(intensity)*unitvec[1]
            point2[i][1]=point2[i][1]+direction*(intensity)*unitvec[1]
        # else:
        #     #print("here fear")
        #     if norm==0:
        #         unitvec=[0,0]
        #     else:
        #         unitvec=[difference[0]/norm,difference[1]/norm]
        #     point1[0][0]=point1[0][0]-direction*(intensity)*unitvec[0]
        #     point2[0][0]=point2[0][0]+direction*(intensity)*unitvec[0]
        #     point1[0][1]=point1[0][1]-direction*(intensity)*unitvec[1]
        #     point2[0][1]=point2[0][1]+direction*(intensity)*unitvec[1]
    #print("changed points: " , point1 , "  " , point2 , " "  )        
    return point1,point2

def check_conc(agent):
    conc = numpy.sqrt(numpy.power(agent[0],2) + numpy.power(agent[1],2))
    #conc = ((x**2) + (y**2))**(0.5);
    return conc;

def find_unit_vec(initpoints,i,agents_ind):
	temp = numpy.zeros(2)  #[0.0, 0.0];
	#print("here1")
	#print(agents_ind , " " )
	# if(numpy.size(agents_ind) > 0):
	# 	print("here6")
	# 	print(agents_ind[0])
	sumx = 0; sumy = 0 ;
	for j in range(numpy.size(agents_ind)):
		#print("here2:")
		#print(agents_ind[j])
		#print(agents_ind[0][j])
		#print(initpoints[:,agents_ind[0][j]])
		temp2 = numpy.asarray(initpoints[:,agents_ind[j]]) - numpy.asarray(initpoints[:,i]) #/numpy.linalg.norm(initpoints[:,agents_ind[j]] - initpoints[:,i])
		temp2 = temp2/numpy.linalg.norm(temp2)
		#print("here5")
		#print(initpoints[:,i] , " " , numpy.shape(temp2))
		sumx = sumx + temp2[0]
		sumy = sumy + temp2[1]
		#print("here4")
		#print(sumx)
	temp[0] = sumx 
	temp[1] = sumy
	temp = temp/numpy.linalg.norm(temp);
	#print("here3")
	#print(temp)
	return temp	

def find_source(initpoints,leader,Rrep,Ratt,leaderRrep,leaderRatt,Wrep,Watt,Wfollow,Wconc_grad,direction,count,conc_t_1):
	# ini_conc = []
	# for i in range(numpy.shape(initpoints)[1]):
	# 	ini_conc.append(check_conc(initpoints[:,i]))
	thresh = 50
	N = numpy.shape(initpoints)[1];
	temp_initpoints = copy.deepcopy(initpoints)
	conc_t = numpy.zeros(N)
	final_vec = numpy.zeros(2);
	final_vec_leader = numpy.zeros(2)
	temp_leader = copy.deepcopy(leader)
	#print("initpoints: " , initpoints)
	rep_vec = numpy.zeros(2)
	for i in range(N):
		dist_from_ith = numpy.linalg.norm(initpoints.T - initpoints[:,i].T, axis=1)
		print("here0")
		print(dist_from_ith)
		dist_from_leader_ith = numpy.linalg.norm(initpoints[:,i] - leader);
		print(dist_from_leader_ith)
		aa_rep0 = numpy.where(dist_from_ith <= Rrep)
		aa_rep1 = numpy.where(dist_from_ith > 0)
		aa_rep = numpy.intersect1d(aa_rep0[0], aa_rep1[0])
		print("here12")
		print(aa_rep)
		aa_rep_vec = find_unit_vec(initpoints,i,aa_rep)
		aa_rep_vec = -aa_rep_vec
		print( aa_rep_vec)

		al_rep0 = numpy.where(dist_from_leader_ith > 0)
		al_rep1 = numpy.where(dist_from_leader_ith <= Rrep)	
		al_rep = numpy.intersect1d(al_rep0[0], al_rep1[0])
		print("here7: ")
		print(al_rep)
		#al_rep_list = []
		#for k in range(numpy.size(al_rep)):
		#	al_rep_list.append(al_rep[0][k])
		if(numpy.size(al_rep) > 0):	
			al_rep_vec = find_unit_vec(initpoints,i,al_rep)
			al_rep_vec = -al_rep_vec
		else:
			al_rep_vec = numpy.zeros(2);	
			print("here9")
			print(al_rep_vec , " " , type(al_rep_vec))	

		rep_vec[0] = aa_rep_vec[0] + al_rep_vec[0]#/numpy.linalg.norm(aa_rep_vec+al_rep_vec)
		rep_vec[1] = aa_rep_vec[1] + al_rep_vec[1]
		print("here10")
		print(rep_vec , " " , type(rep_vec))


		aa_att0 = numpy.where(dist_from_ith >= Rrep)
		aa_att1 = numpy.where(dist_from_ith <= Ratt)
		aa_att = numpy.intersect1d(aa_att0[0] , aa_att1[0])
		aa_att_vec = find_unit_vec(initpoints, i , aa_att)
		# print("here8")
 	# 	print(aa_att_vec)

		if(count > thresh):
			al_att0 = numpy.where(dist_from_leader_ith > leaderRrep)
			al_att1 = numpy.where(dist_from_leader_ith <= leaderRatt)
			al_att = numpy.intersect1d(al_att0[0], al_att1[0])
			al_att_vec = find_unit_vec(initpoints,i, al_att)
		else:
			al_att_vec = numpy.zeros(2)
		print("here11")
		print(al_att_vec , " " , type(al_att_vec))	

		Wrep_leader	= 0.3
		Wfollow = 0.1
		# print("rep_vec: " , rep_vec , " aa_att_vec: " , aa_att_vec)
		final_vec[0] = Wrep*aa_rep_vec[0] + Wfollow*al_att_vec[0];#+ Watt*aa_att_vec[0] #
		final_vec[1] = Wrep*aa_rep_vec[1] + Wfollow*al_att_vec[1];#+ Watt*aa_att_vec[1] #

		if(count > thresh):
			conc_t[i] = check_conc(initpoints[:,i])
			if(conc_t[i] > conc_t_1[i]):
				#change direction move left or right
				z = numpy.random.rand(1)
				conc_vec = numpy.zeros(2);
				if(z < 0.5):
					#move left
					conc_vec[0] = final_vec[1]
					conc_vec[1] = -final_vec[0]
				else:
					#move right	
					conc_vec[0] = -final_vec[1]
					conc_vec[1] = final_vec[0]
			else:
				conc_vec = numpy.zeros(2)	

		else:
			conc_vec = numpy.zeros(2);
			conc_t[i] = check_conc(initpoints[:,i])			

		#add final_vec and conc_Vec
		#final_vec[0] = final_vec[0] + Wconc_grad*conc_vec[0];
		#final_vec[1] = final_vec[1] + Wconc_grad*conc_vec[1]

		#new_ith_pos, a = moveSheperd()			
		step = 0.05;
		temp_initpoints[0,i] = initpoints[0,i] + step*final_vec[0];
		temp_initpoints[1,i] = initpoints[1,i] + step*final_vec[1];
		direction[i] = final_vec;

	
	dist_from_leader = numpy.linalg.norm(initpoints.T - leader,axis = 1)
	l_rep0 = numpy.where(dist_from_leader <= leaderRrep)
	l_rep1 = numpy.where(dist_from_leader > 0.0)
	l_rep = numpy.intersect1d(l_rep0[0], l_rep1[0])
	temp = numpy.zeros(2)
	for i in range(numpy.size(l_rep)):
		temp += (numpy.asarray(leader) - numpy.asarray(initpoints[:,l_rep[i]]))/numpy.linalg.norm(numpy.asarray(leader) - numpy.asarray(initpoints[:,l_rep[i]]))
	
	l_rep_vec = temp;

	if(count > thresh):
		sorted_conc = numpy.argsort(conc_t)
		min_conc = sorted_conc[0];
		#leader move towards the min conc:
		min_conc_agent_vec = (numpy.asarray(initpoints[:,min_conc]) - numpy.asarray(leader))/numpy.linalg.norm(numpy.asarray(initpoints[:,min_conc]) - numpy.asarray(leader))
	else:
		min_conc_agent_vec = numpy.zeros(2)

	final_vec_leader[0] = 0.6*l_rep_vec[0] + 0.4*min_conc_agent_vec[0]
	final_vec_leader[1] = 0.6*l_rep_vec[1] + 0.4*min_conc_agent_vec[1]	
	temp_leader[0] = leader[0] + step*final_vec_leader[0]
	temp_leader[1] = leader[1] + step*final_vec_leader[1]


	return temp_initpoints, temp_leader , direction, conc_t, count	

def main():
    rospy.init_node('Physicometics', anonymous=True)

    pub_marker = rospy.Publisher('Physico', MarkerArray , queue_size=1)
    #sub_cursor = rospy.Subscriber('/initialpose' , PoseWithCovarianceStamped , get_pos)

    rate = rospy.Rate(1) # 10hz

    MC = 1; #total number of runs

    for mc1 in range(MC):

        Xstartpos = 50
        Ystartpos = 0
        threshold = 0
        N = 10 # 1 is nos 
        #function [ X,Y,vels] = puneetsCouzins(X,Y,vels,Rrep,Rori,Ratt,s,dt,phi,omega)
        k = 15
        # x=Xstartpos+numpy.random.rand(1,N)*k - (k/2) #swarm initialization
        # y=Ystartpos+numpy.rando m.rand(1,N)*k - (k/2)  #gives random values from [0,1)
        mu = 0;
        sigma = 2
        x = Xstartpos + numpy.random.normal(mu,sigma,(1,N))
        y = Ystartpos + numpy.random.normal(mu,sigma,(1,N))

        initpoints=numpy.concatenate((x,y),axis=0)
        V1=numpy.zeros((N,2))

        unitvec=numpy.zeros((2,N))    
        
        leader = numpy.zeros(2)
        leader[0] = 55; leader[1] = 0.0;

        Rrep = 30;
        Ratt = 60;
        leaderRrep = 10;
        leaderRatt = 25;
        Wrep = 0.35 #agent agent repulsion
        Wfollow = 0.275; #follow leaders
        Wconc_grad = 0.225; #follow conc grad
        Watt = 0.175  #attracted towards other agents 
        direction = numpy.zeros([N,2])
        M = MarkerArray()    
        l = 0;
        nos = 0; #MAKING nos =0; as we dont want leaders as of now
        nol = 1;
        shape = Marker.CUBE;
        for i in range(N+nol): #to include shepherd too
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

            if(i >= N): #for leader
                marker.ns = "leader" #now leader
                marker.pose.position.x = leader[0] #leader[i-N][0];
                marker.pose.position.y = leader[1] #leader[i-N][1];            
                marker.color.r = 1.0;
                marker.color.g = 0.0;                

            M.markers.append(marker)   
        count = 0;
        conc_t_1 = numpy.zeros(N)    
        while not rospy.is_shutdown():
            pub_marker.publish(M)

            initpoints, leader, direction , conc_t_1 , count  = find_source(initpoints,leader,Rrep,Ratt,leaderRrep,leaderRatt,Wrep,Watt,Wfollow,Wconc_grad,direction,count,conc_t_1);
            count += 1;
            print("yolo: " , count)
            for i in range(N+nol):
                if(i < N):
                    #print("x[0][i]: " , x[i])
                    M.markers[i].pose.position.x = initpoints[0][i] 
                    M.markers[i].pose.position.y = initpoints[1][i]
                
                
                else:
                    #leaderidx = []
                    M.markers[i].pose.position.x = leader[0] #[i-N]
                    M.markers[i].pose.position.y = leader[1] #[i-N]                   	


            rate.sleep()

if __name__ == '__main__':
    #x = []
    #y = []
    try:
        main()
    except rospy.ROSInterruptException:
        pass
