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
from random import randint,choice
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

#pi =  22.0/7.0

N = 20
temppos = numpy.zeros((N,2))
tempdi = numpy.zeros((N,2))

huskyX = numpy.zeros(N)
huskyY = numpy.zeros(N)
Yaw    = numpy.zeros(N)
husky_predX = [0.0, 0.0]
husky_predY = [0.0, 0.0]
Yaw_pred = [0.0, 0.0]

def ang_wrapPositive(angle):
    #print("angle: " , angle)
    #for i in range(len(angle)):
    if(angle < 0):
        angle = angle + 2*pi;
    if(angle > 2*pi):
        angle = angle - 2*pi
    if(angle < -2*pi):
        angle = angle + 4*pi #2*pi;
    return angle                

def SheepMovement(initpoints,vels,Rrep,Rori,Ratt,s1,dt,phi,omega,S,rs,leader_pos,leaderRrep,leaderRori,leaderRatt,destination,destinationthreshold,nol,leader_vels,destination_other,leader_idx,size1,size2):
    pos= initpoints.T

    if(numpy.shape(S)[0] == 0):
        #leader_idx = [len(initpoints[0]),len(initpoints[0])+1]
        nos = 0;
        nol = 0;
    else:
        #print("entered in else: ")
        leader_idx = [];
        nos = nol 
        nol = 0

    flag = -1;
    #print("------------------------------------------------")
    #print("size, initpoints: " , len(initpoints[0]) , " N= " , N)
    dist = numpy.zeros(len(initpoints[0]) + nol)
    for i in range(len(initpoints[0])+nol):
        if(i < N):
            dist[i] = numpy.sqrt(pow(pos[i][1] - destination[0][1],2) + pow(pos[i][0] - destination[0][0],2))
        else:
            dist[i] = (numpy.sqrt(pow((leader_pos[i-N][1] - destination[0][1]),2) + pow((leader_pos[i-N][0] - destination[0][0]),2)))
    #finding all the agents not within the destination threshold    
    
    idx1 = numpy.where(dist > destinationthreshold - 5) #3
    idx11 = numpy.where(dist > destinationthreshold - 7) #6
    for i in range(len(initpoints[0])+nol):
        if(i < N):
            dist[i] = (numpy.sqrt(pow((pos[i][1] - destination_other[0][1]),2) + pow((pos[i][0] - destination_other[0][0]),2)))
        else:
            dist[i] = (numpy.sqrt(pow((leader_pos[i-N][1] - destination_other[0][1]),2) + pow((leader_pos[i-N][0] - destination_other[0][0]),2))) 
    
    idx2 = numpy.where(dist > destinationthreshold - 5)#reduced the threshold to make agents come closer to the center of the tasks
    #idx2 = numpy.setdiff1d(idx2[0],leader_idx)    
    idx = numpy.intersect1d(idx1[0],idx2[0]);
    idx = numpy.setdiff1d(idx,leader_idx)
    
    idx21 = numpy.where(dist > destinationthreshold - 7) # -5, so that leaders dont stop at the boundary, making it difficult for the agents to enter
    idx_temp = numpy.intersect1d(idx21[0],idx11[0])
    
    leader_ag = numpy.intersect1d(idx_temp,leader_idx)
    
    #print("idx " , idx)    
    idx = numpy.union1d(idx,leader_ag)
    #print("idx new : ")
    #print(idx)
    #print("idx1: " , idx1)
    #print("idx2: " ,idx2)
    #print("idx:" , idx)
    #print("dist: " , dist)
    #print("idx: ", idx[0][:] , " " , len(numpy.where(1 == 2)))    

    dists = numpy.zeros(len(initpoints[0]) + nol)  # ADDING ! FOR 1 LEADER FOR A SUB SWARM
    for i in range(len(initpoints[0])+nol):
        # print("=====================================================")
        # print("idx : " , idx)
        # print("size of idx : " , numpy.size(idx))
        #print("i : " , i)
        
        # for k in idx:
        #     if(i == k):
        #         flag = 1
        #         break
        #     else:
        #         flag = -1    
        
        # if(flag != -1): #isnt this same as if(i in idx)
        if(i in idx):
            ind = []
            for j in range(len(initpoints[0]) + nol):
                #dists.append(numpy.sqrt(pow(pos[j,0]-pos[i,0],2) + pow(pos[j,1]-pos[i,1],2)));
                if(j not in idx and j not in leader_idx):
                    ind.append(j)    
                #     dists[j] = 1000; #removing the agetns who have reached from the run
                if(j < N and i < N):
                    dists[j] = numpy.sqrt(pow(pos[j,0]-pos[i,0],2) + pow(pos[j,1]-pos[i,1],2));
                else:
                    if(j >= N and i < N):
                        dists[j] = numpy.sqrt(pow(leader_pos[N-j][0]-pos[i,0],2) + pow(leader_pos[N-j][1]-pos[i,1],2));           
                    elif(j < N and i >= N):    
                        dists[j] = numpy.sqrt(pow(pos[j,0]-leader_pos[i-N][0],2) + pow(pos[j,1]-leader_pos[i-N][1],2));
            
            vi = numpy.where((dists == 0))
            #print("YYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYY")
            #print(i , "  " , vi)
            #vi is  i only
            dists[i] = 10000;
            
        
            v = numpy.where((dists <= Rrep))

            #Removing the repulsion from leaders as a normal agent
            #v = numpy.setdiff1d(v[0],leader_idx)

            v12 = numpy.where(dists <= leaderRrep)   
            v13 = numpy.intersect1d(v12[0], leader_idx)
            #v2 = numpy.intersect1d(v13,leaderidx)
            #print("LLLLLLLLLLLLLLLLLLLLLLLLLLLLLL")
            #print("v: " , v , " " )                
            #print("leaderidx: " , leaderidx)
            v  = numpy.union1d(v[0],v13)

            v = v.astype(int)
            nagentFlag = 0;
            dr = [0,0]; drFlag = 0;
            #print("Flag for repulsion: " , v)
            for j in range(numpy.size(v)):
                if(v[j] < N and i < N):
                    #dotx = numpy.dot(vels[v[j],0],vels[i,0])
                    #doty = numpy.dot(vels[v[j],1],vels[i,1])
                    deno1 = numpy.linalg.norm(vels[v[j],:],axis = 0)
                    deno2 = numpy.linalg.norm(vels[i,:],axis = 0)
                    deno = deno1*deno2
                    #if(abs(numpy.arccos((vels[v[j]][1]*vels[i][1] + vels[v[j]][0]*vels[i][0])/deno)) <= phi):
                    # print("deno:" , deno)
                    # print("cos angle " , numpy.arccos(abs(numpy.dot(vels[v[j],:],vels[i,:])/deno)))
                    # print("tan angle" , numpy.arctan2(vels[v[j],1]-vels[i,1],vels[v[j],0]-vels[i,0]))
                    if(numpy.arccos(abs(numpy.dot(vels[v[j],:],vels[i,:])/deno)) <= phi):
                        dr = dr + (pos[v[j],:]-pos[i,:])/numpy.linalg.norm((pos[v[j],:]-pos[i,:]),axis = 0)
                        drFlag = 1;
                elif(v[j] >= N and i < N):    
                    print("1st elif of repulsion")
                    deno1 = numpy.linalg.norm(vels[v[j]-N,:],axis = 0)
                    deno2 = numpy.linalg.norm(vels[i,:],axis = 0)
                    deno = deno1*deno2                                    
                    if(abs(numpy.arccos(numpy.dot(leader_vels[v[j]-N,:],vels[i,:])/deno)) <= phi):
                        dr = dr + (leader_pos[v[j]-N]-pos[i,:])/numpy.linalg.norm((leader_pos[v[j]-N]-pos[i,:]),axis = 0)
                        drFlag = 1; 

                elif(v[j] < N and i >= N):
                    print(" 2nd elif of repulsion")
                    deno1 = numpy.linalg.norm(vels[v[j],:],axis = 0)
                    deno2 = numpy.linalg.norm(vels[i-N,:],axis = 0)
                    deno = deno1*deno2                    
                    if(abs(numpy.arccos(numpy.dot(vels[v[j],:],leader_vels[i-N,:])/deno)) <= phi):   
                        dr = dr + (pos[v[j],:]-leader_pos[i-N])/numpy.linalg.norm((pos[v[j],:]-leader_pos[i-N]),axis = 0) 
                        drFlag = 1;     


            # % orientation - equation 2
            #checking if agent away from destination or not, if no then proceed
            # if(i in leader_idx):
            #     flag = 1
            #     #break
            # else:
            #     flag = -1  

            zooFlag = 0      
            # LzooFlag = 0                
            zoo = [0 ,0]
            #print("Flag for orientation: " , flag)
            #if(flag == -1):
            if(i not in leader_idx):
                dists1 = numpy.where(dists>Rrep)
                dists2 = numpy.where(dists <= Rori)
                v = numpy.intersect1d(dists1[0] , dists2[0])
                #print("v: " , v)
                #v =union(intersect(find(dists >= leaderRrep & dists<=leaderRori ),leaderidx),v)
                v11 = numpy.where(dists > leaderRrep)                                
                v12 = numpy.where(dists <= leaderRori)
                v13 = numpy.intersect1d(v11[0], v12[0])
                #print("v13: ", v13)
                v2 = numpy.intersect1d(v13,leader_idx)
                
                
                #---------FOR-ORIENTATION-TOWARDS-NEAREST-LEADER-------------------------------------------------------------------------------------------------
                v = numpy.union1d(v2,v)    
                for j in range(numpy.size(v)):
                    #deno = numpy.sqrt(pow(vels[v[j]][1],2)+pow(vels[v[j]][0],2))*numpy.sqrt(pow(vels[i][1],2)+pow(vels[i][0],2))
                    if(v[j] < N and i < N):
                        #print(abs(numpy.arccos((vels[v[j]][1]*vels[i][1] + vels[v[j]][0]*vels[i][0]))))
                        #deno = numpy.sqrt(pow(vels[v[j]][1],2)+pow(vels[v[j]][0],2))*numpy.sqrt(pow(vels[i][1],2)+pow(vels[i][0],2))
                        #print("dot value : " , numpy.dot(vels[v[j],:],vels[i,:]))
                        deno1 = numpy.linalg.norm(vels[v[j],:],axis = 0)
                        deno2 = numpy.linalg.norm(vels[i,:],axis = 0)
                        deno = deno1*deno2                        
                        if(abs(numpy.arccos(numpy.dot(vels[v[j],:],vels[i,:])/deno)) <= phi):
                        #if(v[j] != N):
                            zoo[0] = zoo[0] + vels[v[j]][0]
                            zoo[1] = zoo[1] + vels[v[j]][1]
                            zooFlag = 1;    
                    elif(v[j] >= N and i < N):
                        deno1 = numpy.linalg.norm(leader_vels[v[j]-N,:],axis = 0)
                        deno2 = numpy.linalg.norm(vels[i,:],axis = 0)
                        deno = deno1*deno2                        
                        if(abs(numpy.arccos(numpy.dot(leader_vels[v[j]-N,:],vels[i,:])/deno)) <= phi):
                            zoo[0] = zoo[0] + leader_vels[v[j]-N][0]
                            zoo[1] = zoo[1] + leader_vels[v[j]-N][1]                                
                            zooFlag = 1
                            #LzooFlag = 1
                    elif(v[j] < N and i >= N):
                        deno1 = numpy.linalg.norm(vels[v[j],:],axis = 0)
                        deno2 = numpy.linalg.norm(vels[i-N,:],axis = 0)
                        deno = deno1*deno2                        
                        if(abs(numpy.arccos(numpy.dot(vels[v[j],:],leader_vels[i-N,:])/deno)) <= phi):
                        #if(v[j] != N):
                            zoo[0] = zoo[0] + vels[v[j]][0]
                            zoo[1] = zoo[1] + vels[v[j]][1]
                            zooFlag = 1;                                
        
            # if(i in leader_idx):
            #     flag = 1
            #     #break
            # else:
            #     flag = -1                

            da = [0, 0];        
            #if(flag == -1):  
            if(i not in leader_idx):
                dists1 = numpy.where(dists > Rori)
                dists2 = numpy.where(dists <= Ratt)
                v = numpy.intersect1d(dists2, dists1)                
                zoaFlag = 0;
                for j in range(numpy.size(v)):
                    if(v[j] < N): #EXTRA
                        #deno = numpy.sqrt(pow(vels[v[j]][1],2)+pow(vels[v[j]][0],2))*numpy.sqrt(pow(vels[i][1],2)+pow(vels[i][0],2))
                        deno1 = numpy.linalg.norm(vels[v[j],:],axis = 0)
                        deno2 = numpy.linalg.norm(vels[i,:],axis = 0)
                        deno = deno1*deno2          
                        # print("vels[v[j = " , vels[v[j],:])
                        # print("vels[i,: = " , vels[i,:])
                        # print("deno1" , deno1)
                        # print("angle: " , numpy.arccos(abs(numpy.dot(vels[v[j],:],vels[i,:])/deno)))              
                        if(numpy.arccos(abs(numpy.dot(vels[v[j],:],vels[i,:])/deno)) <= phi):                        
                            da = da + (pos[v[j],:]-pos[i,:])/numpy.linalg.norm((pos[v[j],:]-pos[i,:]),axis = 0) # 2* part EXTRA
                            zoaFlag = 1;    


           # %% Predatory Effect, interested in this at the beginning when 2 predators slplit the swarm
            dpredFlag=0
            #dpred = [0, 0]
            # print("ssssssssssssssssssssssssssssss")
            # print(S)
            if(nos != 0):
                dpred = [0, 0]
                distp = []
                # for j in range(nos):
                #     distp.append(numpy.sqrt(pow(pos[i,0]-S[j,0],2) + pow(pos[i,1]-S[j,1],2)))
                #for j in range(nos):
                distp = numpy.linalg.norm(pos[i] - S,axis = 1)
                v = numpy.where(distp < rs)
                for j in range(numpy.size(v)):
                    #print("IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII")
                    #print("v= " , v)
                    #deno = numpy.sqrt(pow(S[v[0][j]][1],2)+pow(S[v[0][j]][0],2))*numpy.sqrt(pow(vels[i][1],2)+pow(vels[i][0],2))
                    #if(abs(numpy.arccos((S[v[0][j]][1]*vels[i][1] + S[v[0][j]][0]*vels[i][0]))) <= phi):
                    dpred = dpred + (S[v[0][j],:]-pos[i,:])/numpy.linalg.norm((S[v[0][j],:]-pos[i,:]),axis = 0)
                    dpredFlag = 1

            # %% Leader Effect, interested in this
            #print("Flag for LEADER: " , flag)
            #print("leader_idx: " , leader_idx)
            dleaderFlag1 = 0; dleaderFlag2 = 0;
            dleader1 = [0, 0]; dleader2 = [0, 0]
            dleader = [0.0,0.0]
            dleaderFlag = 0;
            #ENTER IF LEADER
            #if(numpy   .intersect1d(leaderidx,i) == i):
            if(i in leader_idx):

                if(i in leader_idx[:size1]):
                    dleader1 = dleader1 + (destination[0,:]-pos[i,:])/numpy.linalg.norm((destination[0,:]-pos[i,:]),axis = 0)        
                    dleaderFlag1 = 1
                    #print("LLLLLLLLLLLLLLLLLLLLLLLLLLLLLL")
                    #print("dleader1: " , dleader1)
                    #print("norm1: " , numpy.linalg.norm(dleader1,axis = 0))
                elif(i in leader_idx[size1:]):
                    print("leaderidx[size1:]" , leader_idx[size1:])
                    dleader2 = dleader2 + (destination_other[0,:]-pos[i,:])/numpy.linalg.norm((destination_other[0,:]-pos[i,:]),axis = 0)        
                    dleaderFlag2 = 1
                    #print("LLLLLLLLLLLLLLLLLLLLLLLLLLLLLL")
                    #print("dleader2: " , dleader2)                    
                    #print("norm2: " , numpy.linalg.norm(dleader2,axis = 0))

                # distp = numpy.linalg.norm(pos[i] - destination)
                # v = numpy.where(distp > 0)
                # for j in range(numpy.size(v)):
                #     dleader = dleader + (destination[0,:]-pos[i,:])/numpy.linalg.norm((destination[0,:]-pos[i,:]),axis = 0)
                #     dleaderFlag = 1;         

                # %% decision making
            di = [0, 0]    
            if(dpredFlag == 1):
                di = di - dpred
                #di = -dpred                
                nagentFlag = 1;
                #print("di: " , di)
                #print("di - dpred" , di_new)
                s = s1*2;
            else:
                s = s1;    
                #print("dr: " , dr , " dleader1: " , dleader1 , " zoo: " , zoo , " da: ", da , "dleader2 ", dleader2)
                #print("SIZE INIPOTS: " , len(initpoints[0]) , " " , pos[N,:] , " N=" , N )
                #if(drFlag == 1 and dleaderFlag == 1):
                    #di = di - dr + dleader
                    #nagentFlag = 1
                #print("zooFlag= " , zooFlag, " drFlag= " , drFlag)                       
                if(drFlag == 1):
                    di = di - dr;
                    nagentFlag = 1

                elif(dleaderFlag1 == 1):
                    di = di + dleader1
                    nagentFlag = 1
                    #print("B1")
                elif(dleaderFlag2 == 1):
                    di = di + dleader2
                    nagentFlag = 1
                    #print("B2")                                    
                    nagentFlag = 1                    

                elif(zooFlag == 1 and zoaFlag == 1):
                    di = di + 0.5*(zoo+da)                    
                    nagentFlag = 1
                    #print("C")
                elif(zooFlag == 1):
                    #print("di: " , di)
                    #print("zoo: " , zoo, " " , zoo[0] , " " , zoo[1])
                    #di = di + zoo
                    di[0] = di[0] + zoo[0]
                    di[1] = di[1] + zoo[1]
                    nagentFlag = 1
                    #print("D")
                    #print("di after changes: " , di)    
                elif(zoaFlag == 1):
                    #print("di inside elif: " , di , " " , da)
                    di[0] = di[0] + da[0]
                    di[1] = di[1] + da[1]
                    nagentFlag = 1
                    #print("E " )
                #else:

                    #print("else:p: EEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE " , i)    

            if(nagentFlag > 0):
                #add noise
                #di = di + randn(1,2)*dt;
                #print("di " , di)
                # if(numpy.intersect1d(i,leader_idx) == i):
                #     di = numpy.asarray(di).reshape(1,2)
                # else:    
                di = di + numpy.random.rand(1,2)*dt
                if(numpy.linalg.norm(di) != 0):
                    di = di/numpy.linalg.norm(di)
                #print("di later" , di)    
                #print("normDI= " , numpy.linalg.norm(di) , " " , numpy.linalg.norm(di,axis = 0))
                #print("di: " , di , " " , di[0] , " i= ", i )
                if(i < N):
                    deno = numpy.sqrt(pow(vels[i][1],2)+pow(vels[i][0],2))*numpy.sqrt(pow(di[0][1],2)+pow(di[0][0],2))
                    #print("numpy.dot : " , (numpy.dot(di,vels[i,:])))
                    #print("without dot : " , ((di[0][1]*vels[i][1]) + (di[0][0]*vels[i][0]))/deno )
                    #deno1 = numpy.linalg.norm(di)
                    #deno2 = numpy.linalg.norm(vels[i],axis = 0)
                    #deno = deno1*deno2                    
                    #if(deno1 != 0):
                    #print("deno" , deno)
                    dtheta = numpy.arccos(numpy.dot(di,vels[i,:])/deno)
                    #print("dtheta" , dtheta)
                    #dtheta = numpy.arccos(((di[0][1]*vels[i][1]) + (di[0][0]*vels[i][0]))/deno)
                    vangle = ang_wrapPositive(numpy.arctan2(vels[i][1],vels[i][0]))
                    #print("vels: " , vels[i])
                    #print("vangle" , vangle)    
                else:
                    #print("ELSE")
                    #deno = numpy.sqrt(pow(leader_vels[1],2)+pow(leader_vels[0],2))*numpy.sqrt(pow(di[0][1],2)+pow(di[0][0],2))
                    deno1 = numpy.linalg.norm(di)
                    deno2 = numpy.linalg.norm(leader_vels[i-N],axis = 0)
                    deno = deno1*deno2                     
                    dtheta = numpy.arccos(numpy.dot(di,leader_vels[i-N,:])/deno)                    
                    vangle = ang_wrapPositive(numpy.arctan2(leader_vels[i-N][1],leader_vels[i-N][0]))
                # if(vels[i][1] == 0.0 and vels[i][0] == 0.0):
                #     vangle = 0.0
                # else:    
                #     vangle = ang_wrapPositive(numpy.arctan(vels[i][1]/vels[i][0]))
                #print("vels[i][1] " , vels[i][1] , " vels[i][0] " , vels[i][0])
                
                #vangle = ang_wrapPositive(numpy.arctan2(vels[i][1],vels[i][0]))
                dangle = ang_wrapPositive(numpy.arctan2(di[0][1],di[0][0]));
                #print("dangle" , dangle)
                #print("dangle: " , numpy.arctan2(di[0][1],di[0][0]) , " " , dangle , " " , vangle)
                R = numpy.zeros((2,2))
                #R = [numpy.cos(2*pi - vangle) , -numpy.sin(2*pi - vangle); numpy.sin(2*pi - vangle), numpy.cos(2*pi - vangle)]
                R[0][0] = numpy.cos(2*pi - vangle)
                R[0][1] = -numpy.sin(2*pi - vangle)
                R[1][0] = numpy.sin(2*pi - vangle)
                R[1][1] = numpy.cos(2*pi - vangle)
                #print("++++++++++++++++++++++++++++++++++++++++++++++++")
                #print("di.T: " , di.T ,)
                #print( " R[0]: " , R[0])
                #print("vels: " , vels[i]) #vels and R[0] should be same. Then change the dth thing as it seems wrong.
                dth = [0 ,0]
                dth[0] = R[0][0]*di[0][0] + R[0][1]*di[0][1]
                dth[1] = R[1][0]*di[0][0] + R[1][1]*di[0][1]
                #print("++++++++++++++++++++++++++++++++++++++++++++++++")
                #print("dth: " , dth , " " , dth[1] , "dangle: " , dangle)
                if(dtheta > omega*dt):
                    dangle1 = vangle + numpy.sign(dth[1])*omega*dt #dth[0] was dth[1] , checking with the change
                else:
                    #if(dangle)    
                    dangle1 = vangle + numpy.sign(dth[1])*dangle*dt
                #temppos = [0,0]
                #tempdi  = [0,0]
                #print("dangle1" , dangle1)
                if(i < N):
                    tempdi[i][0] = numpy.cos(dangle1)
                    tempdi[i][1] = numpy.sin(dangle1)
                    temppos[i][0] = pos[i,0]+numpy.cos(dangle1)*(s)*dt 
                    temppos[i][1] = pos[i,1]+ numpy.sin(dangle1)*(s)*dt
                    #print("temppos" , temppos)
                else:    
                    leader_vels[i-N][0] = numpy.cos(dangle1)
                    leader_vels[i-N][1] = numpy.sin(dangle1)
                    leader_pos[i-N][0] = leader_pos[i-N][0]+numpy.cos(dangle1)*s*dt # we'll change the speed of the leader
                    leader_pos[i-N][1] = leader_pos[i-N][1]+numpy.sin(dangle1)*s*dt                    
            else:
                if(i < N):
                    tempdi[i] = vels[i]
                    temppos[i][0] = pos[i,0] + vels[i,0]*(s)*dt 
                    temppos[i][1] = pos[i,1] + vels[i,1]*(s)*dt
                else:
                    #print("NO change in DIRECTION of LEADER :P")
                    # leader_vels = vels[i]
                    leader_pos[i-N][0] = leader_pos[i-N][0] + leader_vels[i-N][0]*s*dt
                    leader_pos[i-N][1] = leader_pos[i-N][1] + leader_vels[i-N][1]*s*dt 
                    # temppos[i][1] = pos[i,1] + vels[i,1]*s*dt                    

            #print("BBBBBBBBBBBBBBBBBBBBBBBBBBBB")        
            #print(temppos.T)        
            #print("CCCCCCCCCCCCCCCCCCCCCCCCCCCCC")
            #print(initpoints)

        else:
            if(i < N):
                temppos[i][0] = pos[i,0]
                temppos[i][1] = pos[i,1]
                tempdi[i] = vels[i]
            #else:
                #print("NO CHANGE IN LEADER POSITION ")    

    return temppos.T,tempdi              


def moveLeader( point1,point2,intensity,direction ): 
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
       
    return point1,point2

def position_husky0(data):
    global husky_predX, husky_predY, Yaw_pred
    husky_predX[0] = data.pose.pose.position.x
    husky_predY[0] = data.pose.pose.position.y
    husky_orientation = data.pose.pose.orientation

    orientation_list = [husky_orientation.x,husky_orientation.y,husky_orientation.z,husky_orientation.w]
    (roll,pitch,yaw) = euler_from_quaternion(orientation_list)
    Yaw_pred[0] = yaw    

def position_husky1(data):
    global husky_predX, husky_predY, Yaw_pred
    husky_predX[1] = data.pose.pose.position.x
    husky_predY[1] = data.pose.pose.position.y
    husky_orientation = data.pose.pose.orientation

    orientation_list = [husky_orientation.x,husky_orientation.y,husky_orientation.z,husky_orientation.w]
    (roll,pitch,yaw) = euler_from_quaternion(orientation_list)
    Yaw_pred[1] = yaw

def position_husky2(data):
    global huskyX, huskyY, Yaw
    huskyX[0] = data.pose.pose.position.x
    huskyY[0] = data.pose.pose.position.y
    husky_orientation = data.pose.pose.orientation

    orientation_list = [husky_orientation.x,husky_orientation.y,husky_orientation.z,husky_orientation.w]
    (roll,pitch,yaw) = euler_from_quaternion(orientation_list)
    Yaw[0] = yaw

def position_husky3(data):
    global huskyX, huskyY, Yaw
    huskyX[1] = data.pose.pose.position.x
    huskyY[1] = data.pose.pose.position.y
    husky_orientation = data.pose.pose.orientation

    orientation_list = [husky_orientation.x,husky_orientation.y,husky_orientation.z,husky_orientation.w]
    (roll,pitch,yaw) = euler_from_quaternion(orientation_list)
    Yaw[1] = yaw    

def position_husky4(data):
    global huskyX, huskyY, Yaw
    huskyX[2] = data.pose.pose.position.x
    huskyY[2] = data.pose.pose.position.y
    husky_orientation = data.pose.pose.orientation

    orientation_list = [husky_orientation.x,husky_orientation.y,husky_orientation.z,husky_orientation.w]
    (roll,pitch,yaw) = euler_from_quaternion(orientation_list)
    Yaw[2] = yaw    

def position_husky5(data):
    global huskyX, huskyY, Yaw
    huskyX[5-2] = data.pose.pose.position.x
    huskyY[5-2] = data.pose.pose.position.y
    husky_orientation = data.pose.pose.orientation

    orientation_list = [husky_orientation.x,husky_orientation.y,husky_orientation.z,husky_orientation.w]
    (roll,pitch,yaw) = euler_from_quaternion(orientation_list)
    Yaw[5-2] = yaw           

def position_husky6(data):
    global huskyX, huskyY, Yaw
    huskyX[6-2] = data.pose.pose.position.x
    huskyY[6-2] = data.pose.pose.position.y
    husky_orientation = data.pose.pose.orientation

    orientation_list = [husky_orientation.x,husky_orientation.y,husky_orientation.z,husky_orientation.w]
    (roll,pitch,yaw) = euler_from_quaternion(orientation_list)
    Yaw[6-2] = yaw    

def position_husky7(data):
    global huskyX, huskyY, Yaw
    huskyX[7-2] = data.pose.pose.position.x
    huskyY[7-2] = data.pose.pose.position.y
    husky_orientation = data.pose.pose.orientation

    orientation_list = [husky_orientation.x,husky_orientation.y,husky_orientation.z,husky_orientation.w]
    (roll,pitch,yaw) = euler_from_quaternion(orientation_list)
    Yaw[7-2] = yaw        

def position_husky8(data):
    global huskyX, huskyY, Yaw
    huskyX[8-2] = data.pose.pose.position.x
    huskyY[8-2] = data.pose.pose.position.y
    husky_orientation = data.pose.pose.orientation

    orientation_list = [husky_orientation.x,husky_orientation.y,husky_orientation.z,husky_orientation.w]
    (roll,pitch,yaw) = euler_from_quaternion(orientation_list)
    Yaw[8-2] = yaw    

def position_husky9(data):
    global huskyX, huskyY, Yaw
    huskyX[9-2] = data.pose.pose.position.x
    huskyY[9-2] = data.pose.pose.position.y
    husky_orientation = data.pose.pose.orientation

    orientation_list = [husky_orientation.x,husky_orientation.y,husky_orientation.z,husky_orientation.w]
    (roll,pitch,yaw) = euler_from_quaternion(orientation_list)
    Yaw[9-2] = yaw        

def position_husky10(data):
    global huskyX, huskyY, Yaw
    huskyX[10-2] = data.pose.pose.position.x
    huskyY[10-2] = data.pose.pose.position.y
    husky_orientation = data.pose.pose.orientation

    orientation_list = [husky_orientation.x,husky_orientation.y,husky_orientation.z,husky_orientation.w]
    (roll,pitch,yaw) = euler_from_quaternion(orientation_list)
    Yaw[10-2] = yaw        

def position_husky11(data):
    global huskyX, huskyY, Yaw
    huskyX[11-2] = data.pose.pose.position.x
    huskyY[11-2] = data.pose.pose.position.y
    husky_orientation = data.pose.pose.orientation

    orientation_list = [husky_orientation.x,husky_orientation.y,husky_orientation.z,husky_orientation.w]
    (roll,pitch,yaw) = euler_from_quaternion(orientation_list)
    Yaw[11-2] = yaw                

def position_husky12(data):
    global huskyX, huskyY, Yaw
    huskyX[12-2] = data.pose.pose.position.x
    huskyY[12-2] = data.pose.pose.position.y
    husky_orientation = data.pose.pose.orientation

    orientation_list = [husky_orientation.x,husky_orientation.y,husky_orientation.z,husky_orientation.w]
    (roll,pitch,yaw) = euler_from_quaternion(orientation_list)
    Yaw[12-2] = yaw        

def position_husky13(data):
    global huskyX, huskyY, Yaw
    huskyX[13-2] = data.pose.pose.position.x
    huskyY[13-2] = data.pose.pose.position.y
    husky_orientation = data.pose.pose.orientation

    orientation_list = [husky_orientation.x,husky_orientation.y,husky_orientation.z,husky_orientation.w]
    (roll,pitch,yaw) = euler_from_quaternion(orientation_list)
    Yaw[13-2] = yaw        
    
def position_husky14(data):
    global huskyX, huskyY, Yaw
    huskyX[14-2] = data.pose.pose.position.x
    huskyY[14-2] = data.pose.pose.position.y
    husky_orientation = data.pose.pose.orientation

    orientation_list = [husky_orientation.x,husky_orientation.y,husky_orientation.z,husky_orientation.w]
    (roll,pitch,yaw) = euler_from_quaternion(orientation_list)
    Yaw[14-2] = yaw        
    
def position_husky15(data):
    global huskyX, huskyY, Yaw
    huskyX[15-2] = data.pose.pose.position.x
    huskyY[15-2] = data.pose.pose.position.y
    husky_orientation = data.pose.pose.orientation

    orientation_list = [husky_orientation.x,husky_orientation.y,husky_orientation.z,husky_orientation.w]
    (roll,pitch,yaw) = euler_from_quaternion(orientation_list)
    Yaw[15-2] = yaw        
    
def position_husky16(data):
    global huskyX, huskyY, Yaw
    huskyX[16-2] = data.pose.pose.position.x
    huskyY[16-2] = data.pose.pose.position.y
    husky_orientation = data.pose.pose.orientation

    orientation_list = [husky_orientation.x,husky_orientation.y,husky_orientation.z,husky_orientation.w]
    (roll,pitch,yaw) = euler_from_quaternion(orientation_list)
    Yaw[16-2] = yaw        
    
def position_husky17(data):
    global huskyX, huskyY, Yaw
    huskyX[17-2] = data.pose.pose.position.x
    huskyY[17-2] = data.pose.pose.position.y
    husky_orientation = data.pose.pose.orientation

    orientation_list = [husky_orientation.x,husky_orientation.y,husky_orientation.z,husky_orientation.w]
    (roll,pitch,yaw) = euler_from_quaternion(orientation_list)
    Yaw[17-2] = yaw        

def position_husky18(data):
    global huskyX, huskyY, Yaw
    huskyX[18-2] = data.pose.pose.position.x
    huskyY[18-2] = data.pose.pose.position.y
    husky_orientation = data.pose.pose.orientation

    orientation_list = [husky_orientation.x,husky_orientation.y,husky_orientation.z,husky_orientation.w]
    (roll,pitch,yaw) = euler_from_quaternion(orientation_list)
    Yaw[18-2] = yaw        
    
def position_husky19(data):
    global huskyX, huskyY, Yaw
    huskyX[19-2] = data.pose.pose.position.x
    huskyY[19-2] = data.pose.pose.position.y
    husky_orientation = data.pose.pose.orientation

    orientation_list = [husky_orientation.x,husky_orientation.y,husky_orientation.z,husky_orientation.w]
    (roll,pitch,yaw) = euler_from_quaternion(orientation_list)
    Yaw[19-2] = yaw        
    
def position_husky20(data):
    global huskyX, huskyY, Yaw
    huskyX[20-2] = data.pose.pose.position.x
    huskyY[20-2] = data.pose.pose.position.y
    husky_orientation = data.pose.pose.orientation

    orientation_list = [husky_orientation.x,husky_orientation.y,husky_orientation.z,husky_orientation.w]
    (roll,pitch,yaw) = euler_from_quaternion(orientation_list)
    Yaw[20-2] = yaw                 

def position_husky21(data):
    global huskyX, huskyY, Yaw
    huskyX[21-2] = data.pose.pose.position.x
    huskyY[21-2] = data.pose.pose.position.y
    husky_orientation = data.pose.pose.orientation

    orientation_list = [husky_orientation.x,husky_orientation.y,husky_orientation.z,husky_orientation.w]
    (roll,pitch,yaw) = euler_from_quaternion(orientation_list)
    Yaw[21-2] = yaw       

def main():
    rospy.init_node('HuskyLeader', anonymous=True)

    pub_marker = rospy.Publisher('leader2', MarkerArray , queue_size=1)
    
    pub_vel_husky2 = rospy.Publisher('/Husky4/husky_velocity_controller/cmd_vel',Twist, queue_size=10)
    pub_vel_husky3 = rospy.Publisher('/Husky5/husky_velocity_controller/cmd_vel',Twist, queue_size=10)
    pub_vel_husky0 = rospy.Publisher('/Husky2/husky_velocity_controller/cmd_vel',Twist, queue_size=10)
    pub_vel_husky1 = rospy.Publisher('/Husky3/husky_velocity_controller/cmd_vel',Twist, queue_size=10)
    pub_vel_husky4 = rospy.Publisher('/Husky6/husky_velocity_controller/cmd_vel',Twist, queue_size=10)
    pub_vel_husky5 = rospy.Publisher('/Husky7/husky_velocity_controller/cmd_vel',Twist, queue_size=10)
    pub_vel_husky6 = rospy.Publisher('/Husky8/husky_velocity_controller/cmd_vel',Twist, queue_size=10)
    pub_vel_husky7 = rospy.Publisher('/Husky9/husky_velocity_controller/cmd_vel',Twist, queue_size=10)
    pub_vel_husky8 = rospy.Publisher('/Husky10/husky_velocity_controller/cmd_vel',Twist, queue_size=10)
    pub_vel_husky9 = rospy.Publisher('/Husky11/husky_velocity_controller/cmd_vel',Twist, queue_size=10)
    pub_vel_husky10 = rospy.Publisher('/Husky12/husky_velocity_controller/cmd_vel',Twist, queue_size=10)
    pub_vel_husky11 = rospy.Publisher('/Husky13/husky_velocity_controller/cmd_vel',Twist, queue_size=10)
    pub_vel_husky12 = rospy.Publisher('/Husky14/husky_velocity_controller/cmd_vel',Twist, queue_size=10)
    pub_vel_husky13 = rospy.Publisher('/Husky15/husky_velocity_controller/cmd_vel',Twist, queue_size=10)
    pub_vel_husky14 = rospy.Publisher('/Husky16/husky_velocity_controller/cmd_vel',Twist, queue_size=10)
    pub_vel_husky15 = rospy.Publisher('/Husky17/husky_velocity_controller/cmd_vel',Twist, queue_size=10)
    pub_vel_husky16 = rospy.Publisher('/Husky18/husky_velocity_controller/cmd_vel',Twist, queue_size=10)
    pub_vel_husky17 = rospy.Publisher('/Husky19/husky_velocity_controller/cmd_vel',Twist, queue_size=10)
    pub_vel_husky18 = rospy.Publisher('/Husky20/husky_velocity_controller/cmd_vel',Twist, queue_size=10)
    pub_vel_husky19 = rospy.Publisher('/Husky21/husky_velocity_controller/cmd_vel',Twist, queue_size=10)

    pub_vel_husky_pred0 = rospy.Publisher('/Husky0/husky_velocity_controller/cmd_vel',Twist, queue_size=10)
    pub_vel_husky_pred1 = rospy.Publisher('/Husky1/husky_velocity_controller/cmd_vel',Twist, queue_size=10)

    rospy.Subscriber('/Husky4/base_pose_ground_truth', Odometry, position_husky4)
    rospy.Subscriber('/Husky5/base_pose_ground_truth', Odometry, position_husky5)
    rospy.Subscriber('/Husky1/base_pose_ground_truth', Odometry, position_husky1)
    rospy.Subscriber('/Husky2/base_pose_ground_truth', Odometry, position_husky2)
    rospy.Subscriber('/Husky3/base_pose_ground_truth', Odometry, position_husky3)
    rospy.Subscriber('/Husky0/base_pose_ground_truth', Odometry, position_husky0)
    rospy.Subscriber('/Husky6/base_pose_ground_truth', Odometry, position_husky6)
    rospy.Subscriber('/Husky7/base_pose_ground_truth', Odometry, position_husky7)
    rospy.Subscriber('/Husky8/base_pose_ground_truth', Odometry, position_husky8)
    rospy.Subscriber('/Husky9/base_pose_ground_truth', Odometry, position_husky9)    
    rospy.Subscriber('/Husky10/base_pose_ground_truth', Odometry, position_husky10)
    rospy.Subscriber('/Husky11/base_pose_ground_truth', Odometry, position_husky11)
    rospy.Subscriber('/Husky12/base_pose_ground_truth', Odometry, position_husky12)
    rospy.Subscriber('/Husky13/base_pose_ground_truth', Odometry, position_husky13)
    rospy.Subscriber('/Husky14/base_pose_ground_truth', Odometry, position_husky14)
    rospy.Subscriber('/Husky15/base_pose_ground_truth', Odometry, position_husky15)
    rospy.Subscriber('/Husky16/base_pose_ground_truth', Odometry, position_husky16)
    rospy.Subscriber('/Husky17/base_pose_ground_truth', Odometry, position_husky17)
    rospy.Subscriber('/Husky18/base_pose_ground_truth', Odometry, position_husky18)
    rospy.Subscriber('/Husky19/base_pose_ground_truth', Odometry, position_husky19)
    rospy.Subscriber('/Husky20/base_pose_ground_truth', Odometry, position_husky20)
    rospy.Subscriber('/Husky21/base_pose_ground_truth', Odometry, position_husky21)    

    rate = rospy.Rate(10) # 10hz

    destination1x = [-30. , -71. ,-108. ,-140. , -63. , -82. , -78. ,-116. , -63. , -96., -119. , -71.,  -109. , -62. ,-105. , -91. ,-142. , -68. ,-128. ,-133. ,-136. , -80. ,-128. , -59.,
  -130.]
    destination1y =  [  0.,  108. ,-124.,  -24. , 227. ,  53., -185.,   78.,   90.,   55., -192. ,  68.,
  -167., -147. , -91. , 173. ,  34. , 105. , -22., -146. , -95.  , -5. , 225. , 205.,
   -44.]

#('dest2: ',)
    destination2x = [ 30. , 198. , 169. , 218.,  179. , 171.,  223.,  187.,  240. , 212. , 232. , 239.,
   196. , 248. , 176.  ,243. , 157. , 220. , 179. , 151. , 197. , 196. , 164. , 240.,
   230.]
    destination2y = [ 0. ,-107., -228.,  234., -157., -223. ,-209. ,-141. , 145. , 138. ,  37. ,-162.,
    67. ,  80. , 249. ,  48. ,  42. ,  62.,  103. , 125. ,-151.,  186. , 187., -222.,
  -100.]

#('dest: ',)
    destinationx = [ 0.  , 16. , -57.,   33. , -10. ,  54. ,  11. ,  72.,    7. , -92. ,  47. , -85.,
   -56.  , 40. , -24. , -26. ,  15. ,   7. , -71. ,  -8.,   61. ,  57. ,  21.,  -38.,
    74.]
    destinationy = [  100. , 238. , 159.,  -76. , -20. ,-209. ,  31. , -59. ,-171.  , -6. ,-180. , -20.,
   144., -230. ,  86. ,  69., -127., -119. , -48., -211. , 120. ,  36. , 231.,  -82.,
   142.]

    MC = len(destination1x) 
    MC = 1
    start_time = numpy.zeros((1,MC))
    stop_time = numpy.zeros((1,MC))
    split_time = numpy.zeros((1,MC))
    diff = numpy.zeros((1,MC))
    dest1_arr = numpy.zeros((2,MC))
    dest2_arr = numpy.zeros((2,MC))
    dest_arr = numpy.zeros((2,MC))
    split_diff = numpy.zeros((1,MC))
    not_completed = numpy.zeros((1,MC))
    agent_sep = numpy.zeros((1,MC))
    task = numpy.zeros((1,MC))


    for mc1 in range(MC):
        global N, temppos, tempdi
        N = 20
        # tempdi = numpy.zeros((N,2))
        # temppos = numpy.zeros((N,2))

        
        Xstartpos = 0.0
        Ystartpos = 0.0
        threshold = 0

        nol = 2
        leader_pos1 = [0.0,9.0]
        leader_pos2 = [0.0,-9.0]
        #leader_pos = numpy.asarray(leader_pos1).reshape(nol,2)
        LeaderPos = numpy.asarray([leader_pos1,leader_pos2]).reshape(nol,2)
        #leaderidx = numpy.zeros(nol)
        leaderidx = N
        #NT = N + nol;

        #function [ X,Y,vels] = puneetsCouzins(X,Y,vels,Rrep,Rori,Ratt,s,dt,phi,omega)
        #k = 15
        #x=Xstartpos+numpy.random.rand(1,N)*k - (k/2) #swarm initialization
        #y=Ystartpos+numpy.random.rand(1,N)*k - (k/2)  #gives random values from [0,1)
        
        tempdi = numpy.zeros((N,2))
        temppos = numpy.zeros((N,2))
        mu = 0;
        sigma = 3;
        x = Xstartpos + numpy.random.normal(mu,sigma,(1,N))
        y = Ystartpos + numpy.random.normal(mu,sigma,(1,N))

        #x[0][N] = 30;
        #y[0][N] = 0;

        initpoints=numpy.concatenate((x,y),axis=0)
        initpoints[0][0] = -1.5
        initpoints[0][1] = 0.0
        initpoints[1][0] = 1.5
        initpoints[1][1] = 0.0

        vels=numpy.zeros((N,2))
        for i in range(N):
            vels[i][0] = numpy.random.rand(1,1)
            vels[i][1] = numpy.random.rand(1,1)

        vels[0,0] = 1.0; vels[0,1] = 0.0    
        vels[1,0] = -1.0; vels[1,1] = 0.0 
        leader_vels1 = numpy.zeros((1,2)); 
        leader_vels2 = numpy.zeros((1,2))
        
        leader_vels1[0][0] = numpy.random.rand(1,1)    
        leader_vels1[0][1] = numpy.random.rand(1,1)             

        leader_vels2[0][0] = numpy.random.rand(1,1)    
        leader_vels2[0][1] = numpy.random.rand(1,1)    

        LeaderVels = numpy.asarray([leader_vels1,leader_vels2]).reshape(nol,2)
        #leader_vels2 = [numpy.random.rand(1,1), numpy.random.rand(1,1)]    
        #print("initpoints: " , initpoints[:,N])    
        #initpoints[0][N] = 0
        #initpoints[1][N] = 40
        #velsx = numpy.random.rand(1,N)
        #velsy = numpy.random.rand(1,N)
        # #print(vels)
        # #print(vels[:3][0])
        # #print(numpy.size(vels)) 
        #vels=numpy.concatenate((velsx,velsy),axis=0)
        #print("ssssssssssssssssssssssssssssss")
        #print(vels)       
        #vels[:][0] = velsx[0]
        #vels[:][1] = velsy[1]
        #vels[1] = 1
        vels_temp = vels
        #unitvec=numpy.zeros((2,N))
        #temppos = numpy.zeros((N,2))
        #tempdi = numpy.zeros((N,2))
        Rrep = 2.5
        Rori = Rrep + 7.5 #can increase these
        Ratt = Rori + 2.5
        s = 1
        s1 = 1.0  
        dt = 0.1    
        phi = 100*(pi)/180
        omega = 1.0

        S = []

        leaderRrep = 2.5      
        leaderRori = 10 + leaderRrep #The agents between the distance of 10(leaderRori) and 20(leaderRatt) get attracted towards them
        leaderRatt = 0 # We

        # destination1 = [-45.0,25.0]
        # destination2 = [45.0,25.0]    
        # destination = [0.0,50.0]
        destination = [destinationx[mc1],destinationy[mc1]]
        destination1 = [destination1x[mc1],destination1y[mc1]]
        destination2 = [destination2x[mc1],destination2y[mc1]]
        # destination = numpy.asarray([destinationx[mc1],destinationy[mc1]]).reshape(1,2)
        # destination1 = numpy.asarray([destination1x[mc1],destination1y[mc1]]).reshape(1,2)
        # destination2 = numpy.asarray([destination2x[mc1],destination2y[mc1]]).reshape(1,2)
        destination = numpy.asarray(destination).reshape(1,2)
        destination1 = numpy.asarray(destination1).reshape(1,2)
        destination2 = numpy.asarray(destination2).reshape(1,2)

        destinationthreshold = 10.0
        rs = 15;
        nod = 3;
        M = MarkerArray()

        H = []
        for i in range(N):
            H.append(Twist())

        H_pred = [Twist(), Twist()]        

        l = 0;
        shape = Marker.CUBE;
        for i in range(N + nol + nod): #to include shepherd too
            marker = Marker()
            #print("i " , i)
            marker.header.frame_id = "/multi_sheep_leader2";
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
       
            
            # if(i in leaderidx):
            #     marker.color.r = 1.0;
            #     marker.color.g = 0.0;


            if(i >= N and i<N+nol):
                marker.ns = "shep"
                marker.pose.position.x = LeaderPos[i - N][0];
                marker.pose.position.y = LeaderPos[i - N][1];
                marker.color.r = 1.0;
                marker.color.g = 0.0;

            if(i >= N+nol):
                marker.ns = "Point"
                marker.type = Marker.CYLINDER
                marker.scale.x = 2*destinationthreshold;
                marker.scale.y = 2*destinationthreshold;
                if(l == 0):
                    marker.pose.position.x = destination1[0][0]
                    marker.pose.position.y = destination1[0][1]
                    l = 1;
                elif(l == 1):
                    marker.pose.position.x = destination2[0][0]
                    marker.pose.position.y = destination2[0][1]
                    l = 2;
                elif(l == 2):
                    marker.pose.position.x = destination[0][0]
                    marker.pose.position.y = destination[0][1]                        
                    marker.scale.x = 2*(destinationthreshold+5)
                    marker.scale.y = 2*(destinationthreshold+5)
                marker.color.b = 1.0;
                marker.color.r = 0.0;

                marker.scale.z = 0.5;   

            marker.lifetime = rospy.Duration();

            M.markers.append(marker)    

        count = 0;
        grp1 = [] ; grp2 = []
        for i in range(N):
            grp1.append(i)
            grp2.append(i)

        g1 = grp1;
        g2 = grp2;   


        near_swarm1 = 0; near_swarm2 = 0;
        in1 = 0; in2 = 0;
        eaten1 = 0; eaten2 = 0; eaten3 = 0; eating3 = 0;
        entered1 = 0; entered2= 0;
        dest1 = numpy.zeros((1,2)) ; dest2 = numpy.zeros((1,2))
        Lreached1 = 0 ; Lreached2 = 0;
        orient1 = 0; orient2 = 0;
        entered_once = 0;
        sub_dest1 = [0.0,0.0] ; sub_dest2 = [0.0,0.0];
        swarm_split = 0;
        leader_idx = [];
        entered_once2 = 0;
        entered_once4 = 0;
        size_dec = 0.02
        G1 = [] ; ind_l1 = [] ; ind_l2 = []
        entered_once3 = 0;
        k_pred = 0.40 #1.5 times sheep | k is declared below only as another k is being used in one of the for loops
        angle_bound = pi
        slow_speed_for_some_dist = False
        check_mode = numpy.zeros(2)
        threshold_for_sub_destination = 5;      

        sheperd_speed = numpy.zeros(nol);
        delta_theta_sheperd = numpy.zeros(nol);
        change_in_heading_sheperd = numpy.zeros(nol)
        dist_left_to_cover_sheph = numpy.zeros(nol)

        sheep_speed = numpy.zeros(N);
        delta_theta = numpy.zeros(N)
        change_in_heading = numpy.zeros(N)
        new_change_in_heading = numpy.zeros(N)
        dist_left_to_cover = numpy.zeros(N)
        temp_LeaderPos = copy.deepcopy(LeaderPos)
        sp = [0.0, 0.0]
        rad_to_deg = 180/pi
        start_time[0][mc1] = rospy.get_rostime().to_sec()

        while not rospy.is_shutdown():
            pub_marker.publish(M)        
            print("pred:", H_pred[0].linear.x , H_pred[1].linear.x)
            pub_vel_husky_pred0.publish(H_pred[0])            
            pub_vel_husky_pred1.publish(H_pred[1])            

            pub_vel_husky0.publish(H[0])
            pub_vel_husky1.publish(H[1])
            pub_vel_husky2.publish(H[2])
            pub_vel_husky3.publish(H[3])            
            pub_vel_husky4.publish(H[4])
            pub_vel_husky5.publish(H[5])
            pub_vel_husky6.publish(H[6])
            pub_vel_husky7.publish(H[7]) #put a loop tried will have to google
            pub_vel_husky8.publish(H[8])
            pub_vel_husky9.publish(H[9])
            pub_vel_husky10.publish(H[10])
            pub_vel_husky11.publish(H[11])
            pub_vel_husky12.publish(H[12])
            pub_vel_husky13.publish(H[13])
            pub_vel_husky14.publish(H[14])
            pub_vel_husky15.publish(H[15])
            pub_vel_husky16.publish(H[16])
            pub_vel_husky17.publish(H[17])
            pub_vel_husky18.publish(H[18])
            pub_vel_husky19.publish(H[19])
            

            print("yolo")
            #print("initpoints before:")
            #print(initpoints) 
            #print("before: WWWWWWWWWWWWWWWW")
            #print(vels)
            #print(initpoints )
            #print("before: ",initpoints[:,1] , " lead: " , leader_pos2)
            #temp_ini = initpoints[:,1] # ????
            temp_lead = LeaderPos
            temp_initpoints = copy.deepcopy(initpoints)
            # if(numpy.size(G1) > 0): # groups formed
            #     DD1 = numpy.linalg.norm(initpoints.T - LeaderPos[0].T,axis = 1)
            #     DD2 = numpy.linalg.norm(initpoints.T - LeaderPos[1].T,axis = 1)
            #     far_from1 = numpy.where(DD1 > 3*rs)
            #     far_from2 = numpy.where(DD2 > 3*rs)
            #     separated_agents = numpy.intersect1d(far_from1[0],far_from2[0])
            #     grp1 = numpy.setdiff1d(grp1,separated_agents)
            #     grp2 = numpy.setdiff1d(grp2,separated_agents)
            #     #print("grp1: " , grp1)
            #     # grp1 = numpy.intersect1d(grp1[0],grp1[0])
            #     # grp2 = numpy.intersect1d(grp2[0],grp2[0])
            #     agent_sep[0][mc1] = numpy.size(separated_agents)
            #     #print("separated_agents: " , separated_agents)                

            #AFTER SWARM HAS SPLIT, 2nd STAGE
            #if(((near_swarm1 == 1 and near_swarm2 == 1) or swarm_split == 1) and (eaten1 == 0 or eaten2 == 0)):
            if((swarm_split == 1) and (eaten1 == 0 or eaten2 == 0)):
                print("2222222222222222222222222222222222222222")
                #print("eaten: " , eaten1 , " " , eaten2)
                #if(swarm_split == 0):
                    #ORITENT BOTH
                if(entered_once == 0):
                    #DIVISION OF GROUPS 
                    ind_gcm1 = numpy.where(initpoints[0,:] < 0)
                    ind_gcm2 = numpy.where(initpoints[0,:] > 0)
                    GCM0 = [numpy.average(initpoints[0,ind_gcm1[0]]),numpy.average(initpoints[1,ind_gcm1[0]])]
                    GCM1 = [numpy.average(initpoints[0,ind_gcm2[0]]),numpy.average(initpoints[1,ind_gcm2[0]])]

                    m1 = numpy.arctan2(destination1[0][1]-GCM0[1],destination1[0][0]-GCM0[0])
                    sub_dest1[0] = GCM0[0] + 5.0*numpy.cos(m1)
                    sub_dest1[1] = GCM0[1] + 5.0*numpy.sin(m1)

                    m2 = numpy.arctan2(destination2[0][1]-GCM1[1],destination2[0][0]-GCM1[0])
                    sub_dest2[0] = GCM1[0] + 5.0*numpy.cos(m2)
                    sub_dest2[1] = GCM1[1] + 5.0*numpy.sin(m2)
                    entered_once = 1;
                    #print("HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH")    
                    initpoints,vels =  SheepMovement(initpoints,vels,Rrep,Rori,Ratt,s1,dt,phi,omega,LeaderPos,rs,[],leaderRrep,leaderRori,leaderRatt,numpy.asarray([100,100]).reshape(1,2),0,0,[],numpy.asarray([0,0]).reshape(1,2),leader_idx,numpy.size(ind_l1),
                        numpy.size(ind_l2))    

                #else:    
                #if(entered_once2 == 0):
                    # DD1 = numpy.linalg.norm(initpoints.T - LeaderPos[0],axis = 1)
                    # DD2 = numpy.linalg.norm(initpoints.T - LeaderPos[1],axis = 1)
                    # far_from1 = numpy.where(DD1 > 100)
                    # far_from2 = numpy.where(DD2 > 100)
                    # separated_agents = numpy.intersect1d(far_from1[0],far_from2[0])
                    # g1 = numpy.setdiff1d(g1[0],separated_agents)
                    # g2 = numpy.setdiff1d(g2[0],separated_agents)
                    # agent_sep[0][mc1] = numpy.size(separated_agents)
                    ##############################################################################################################
                    #ind_l1 = numpy.random.choice(numpy.size(ind_gcm1),1)
                    #ind_l2 = numpy.random.choice(numpy.size(ind_gcm2),1)
                    #mid_a1 = numpy.where(interD < Rrep + 2)
                    within1 = numpy.zeros((1,numpy.size(ind_gcm1)))
                    #CHOOSING LEADERS!!
                    #here leaders = size(ind_gcm1)/10
                    for k in range(numpy.size(ind_gcm1)):
                        #for l in range(numpy.size(ind_gcm1)):
                        #   D = numpy.linalg.norm(initpoints[:,ind_gcm1[l]] - initpoints[:,ind_gcm1[k]])
                        #print("ind_gcm1: " , ind_gcm1 , " ")
                        #print("k: " , ind_gcm1[0] ,)
                        #print(ind_gcm1[0][k])
                        #print(initpoints[:,ind_gcm1[0][k]])
                        no_of_agents_per_leader = 4 #earlier 10
                        interD = numpy.linalg.norm(initpoints[:,ind_gcm1[0]].T - numpy.asarray(initpoints[:,ind_gcm1[0][k]]).reshape(1,2),axis = 1)
                        within1[0][k] = numpy.size(numpy.where(interD < Rori))
                    sorted_agents = numpy.argsort(within1)
                    #print("sorted: " , sorted_agents)
                    #print(sorted_agents[0][numpy.size(ind_gcm1)-numpy.size(ind_gcm1)/10:])
                    ind_l1 = sorted_agents[0][numpy.size(ind_gcm1)-numpy.size(ind_gcm1)/5:] #5 instead of 10
                    if(numpy.size(ind_gcm1)/(no_of_agents_per_leader+1) == 0):
                        ind_l1 = sorted_agents[0][numpy.size(ind_gcm1)-2:]

                    within2 = numpy.zeros((1,numpy.size(ind_gcm2)))
                    for k in range(numpy.size(ind_gcm2)):
                        #for l in range(numpy.size(ind_gcm1)):
                        #   D = numpy.linalg.norm(initpoints[:,ind_gcm1[l]] - initpoints[:,ind_gcm1[k]])
                        #print("ind_gcm2")
                        #print(initpoints[:,ind_gcm2[0][k]])
                        interD = numpy.linalg.norm(initpoints[:,ind_gcm2[0]].T - numpy.asarray(initpoints[:,ind_gcm2[0][k]]).reshape(1,2),axis = 1)
                        within2[0][k] = numpy.size(numpy.where(interD < Rori))
                    sorted_agents = numpy.argsort(within2)
                    ind_l2 = sorted_agents[0][numpy.size(ind_gcm2)-numpy.size(ind_gcm2)/5:] #5 instead of 10 for 2 leaders-                   
                    if(numpy.size(ind_gcm2)/(no_of_agents_per_leader+1) == 0):
                        ind_l2 = sorted_agents[0][numpy.size(ind_gcm2)-2:]                    

                    #ind_l1 = numpy.random.choice(numpy.size(ind_gcm1),5)
                    #ind_l2 = numpy.random.choice(numpy.size(ind_gcm2),5)                    
                    #print("ind_l1: " , ind_l1 ," size: " , numpy.size(ind_gcm1) , " ")
                    #print("ind_l2: " , ind_l2," size: " , numpy.size(ind_gcm2) , " " ,)
                    #leader_idx = [ind_gcm1[0][ind_l1[0]],ind_gcm2[0][ind_l2[0]]]
                    
                    for k in range(numpy.size(ind_l1)):
                        leader_idx.append(ind_gcm1[0][ind_l1[k]])
                        M.markers[ind_gcm1[0][ind_l1[k]]].color.r = 1.0
                    for k in range(numpy.size(ind_l2)):
                        leader_idx.append(ind_gcm2[0][ind_l2[k]])   
                        M.markers[ind_gcm2[0][ind_l2[k]]].color.r = 1.0
                    #leader_idx = [ind_gcm1[0][ind_l1[0]],ind_gcm1[0][ind_l1[1]],ind_gcm2[0][ind_l2[0]],ind_gcm2[0][ind_l2[1]]]
                    #leader_idx = numpy.union1d(ind_l1[0],ind_l2[0])
                    #print("leader_idx: " , leader_idx)
                    #M.markers[leader_idx[0][0]].color.r = 1.0
                    #M.markers[leader_idx[0]].color.g = 1.0
                    #M.markers[leader_idx[0][1]].color.r = 1.0
                    #M.markers[leader_idx[1]].color.g = 1.0                
                    #entered_once2 = 1;
                #S is always empty
                initpoints,vels = SheepMovement(initpoints,vels,Rrep,Rori,Ratt,s1,dt,phi,omega,S,rs,LeaderPos,leaderRrep,leaderRori,leaderRatt,destination1,destinationthreshold,nol,LeaderVels,destination2,leader_idx,numpy.size(ind_l1),
                    numpy.size(ind_l2))
                #Leader_pos is of no use in the above, it is only used when nol is > 0 
                #in above function call of sheepMovement nol = 0 as S = []
                #print("LeaderPos: " , LeaderPos)

                    # if(numpy.linalg.norm((LeaderPos[0] - destination1[0]),axis = 0) <= destinationthreshold  and in1 == 1 and entered1 == 0):
                    #     entered1 = 1;
                    #     m1 = numpy.arctan2((destination[0][1] - destination1[0][1]),(destination[0][0] - destination1[0][0]))
                    #     # if(m1 > pi/2):
                    #     #     m1 = m1 - pi/2
                    #     dest1[0][0] = destination1[0][0] + (5)*numpy.cos(m1);
                    #     dest1[0][1] = destination1[0][1] + 5*numpy.sin(m1);    


                    # if(numpy.linalg.norm((LeaderPos[1] - destination2[0]),axis = 0) <= destinationthreshold and in2 == 1 and entered2 == 0):
                    #     entered2 = 1;
                    #     m2 = numpy.arctan2((destination[0][1] - destination2[0][1]),(destination[0][0] - destination2[0][0]))
                    #     # if(m2 > pi/2):
                    #     #     m2 = m2 - pi/2
                    #     dest2[0][0] = destination2[0][0] + (5)*numpy.cos(m2);
                    #     dest2[0][1] = destination1[0][1] + 5*numpy.sin(m2);    
                    
                    #initpoints,vels,LeaderPos[0],LeaderVels[0] =  SheepMovement(initpoints,vels,Rrep,Rori,Ratt,s1,dt,phi,omega,S,rs,LeaderPos[0],leaderRrep,leaderRori,leaderRatt,numpy.asarray(LeaderPos[0]).reshape(1,2),2,nol,LeaderVels[0],numpy.asarray(LeaderPos[1]).reshape(1,2))        
                    #print("++++++++++++++++++++++++++++++++")
                    #print("leader1: " , LeaderPos[0,:] , " " , LeaderVels[0])
                    #print("leader2: " , LeaderPos[1,:] , " " , LeaderVels[1])
                    #print("vels: ")
                    #print(vels)

            #INITIAL STATE, BEFORE SPLIT
            elif((near_swarm1 == 0 or near_swarm2 == 0 or swarm_split == 0) and (eaten1 == 0 and eaten2 == 0)):#Rori same as Rrep => swarm state
                #print("111111111111111111111111111111111111111")
                #print("befire function call")
                #print(initpoints)
                #print("vels", vels)
                initpoints,vels =  SheepMovement(initpoints,vels,Rrep,Rori,Ratt,s1,dt,phi,omega,LeaderPos,rs,[],leaderRrep,leaderRori,leaderRatt,numpy.asarray([10000,0]).reshape(1,2),0,nol,[],numpy.asarray([10000,0]).reshape(1,2),leader_idx,numpy.size(ind_l1),
                    numpy.size(ind_l2))        
                #initpoints,vels,Rrep,Rori,Ratt,s1,dt,phi,omega,S,rs,leader_pos,leaderRrep,leaderRori,leaderRatt,destination,destinationthreshold,nol,leader_vels,destination_other,leader_idx,size1,size2
                # print("initpoints right after function call")
                # print(initpoints)
                # print("vels", vels)
                # print("")

            #JOINING BACK STAGE
            elif(eaten1 == 1 and eaten2 == 1):
                print("333333333333333333333333333333333333333")
                if(entered_once4 == 0):
                    destination = numpy.asarray([0.0, 10.0]).reshape(1,2)
                    M.markers[N+nol+nod-1].pose.position.y = destination[0][1];
                    entered_once4 = 1;
                initpoints,vels =  SheepMovement(initpoints,vels,Rrep,Rori,Ratt,s1,dt,phi,omega,S,rs,LeaderPos,leaderRrep,leaderRori,leaderRatt,destination,destinationthreshold+5,nol,LeaderVels,destination,leader_idx,
                    numpy.size(ind_l1),numpy.size(ind_l2))                        
                #print("HERE")
                in_g1 = numpy.intersect1d(ind_gcm1,near_dest)
                in_g2 = numpy.intersect1d(ind_gcm2,near_dest)
                if(numpy.size(near_dest) > 0 and numpy.size(in_g1) > 0 and numpy.size(in_g2)>0):
                    #START EATING 3
                    #print("decreasing") 
                    M.markers[N+nol+2].scale.x = M.markers[N+nol+2].scale.x - size_dec*numpy.size(near_dest)
                    M.markers[N+nol+2].scale.y = M.markers[N+nol+2].scale.y - size_dec*numpy.size(near_dest)
                    if(M.markers[N+nol+2].scale.x < 0.2):
                        eaten3 = 1;                                             
                    #eating3 = 1;


            #print("before: ",temp_ini , " lead: " , temp_lead)
            #print("initpoints after: " , initpoints[:,1], " lead: " , leader_pos2)
            #print(initpoints)
            #print("AFTER:ZZZZZZZZZZZZZZZZZZZZZZ")
            #print(vels.T)
            #print("AAAAAAAAAAAAAAA")
            #print(vels_temp)
            #leader_pos[0] = leader_pos1;
            #leader_pos[1] = leader_pos2;
            #print("initpoints before for: ")
            #print(initpoints)
            #print("")
            for i in range(N):
                print(i)
                #if(i in leaderidx):
                #print("i in main: " , i)
                #if(i < N):
                #print(i," vel ",vels[i])
                change_in_heading[i] = numpy.arctan2(vels[i][1] , vels[i][0])    
                #vels will be updated after every iteration
                vels[i][1] = numpy.sin(Yaw[i])
                vels[i][0] = numpy.cos(Yaw[i])
                #new_change_in_heading[i] = numpy.arctan2(initpoints[1,i] - temp_initpoints[1,i] , initpoints[0,i] - temp_initpoints[0,i])
                delta_theta[i] = change_in_heading[i] - Yaw[i]; #we can scale this                    
                #print("new and old") #coming SAME
                #print(new_change_in_heading[i] , change_in_heading[i])
                #print("change_in_heading" , change_in_heading[i]*rad_to_deg)
                #print("delta_theta" , delta_theta[i]*rad_to_deg)
                #print("Yaw" , Yaw[i]*rad_to_deg)
                #To prevent very fast rotation which can lead to toppling
                #Below not running good
                if(huskyX[i] < 0 and abs(delta_theta[i]) > angle_bound): # and eaten1 != 1 
                #if((Yaw[i] < -pi/2 and change_in_heading[i] > pi/2) or (Yaw[i] > pi/2  and change_in_heading[i] < -pi/2)):
                #or should the condition be fpr those who ae facing the 3rd and 4th quad 
                    #delta_theta[i] = numpy.sign(delta_theta[i])*angle_bound
                    delta_theta[i] = -numpy.sign(delta_theta[i])*(2*pi - abs(delta_theta[i]))
                    if(abs(delta_theta[i]) > angle_bound):
                        delta_theta[i] = numpy.sign(delta_theta[i])*angle_bound
                    # if(huskyX[i] < 0):
                    #     delta_theta[i] = -1*delta_theta[i]  #(numpy.sign(delta_theta[i]))*((2*pi)%delta_theta[i])        
                #print("changed delta_theta" , delta_theta[i]*rad_to_deg)
                if(eaten2 == 1 and huskyX[i] > 0 and abs(delta_theta[i]) > angle_bound):
                    delta_theta[i] = -numpy.sign(delta_theta[i])*(2*pi - abs(delta_theta[i]))
                    if(abs(delta_theta[i]) > angle_bound):
                        delta_theta[i] = numpy.sign(delta_theta[i])*angle_bound                    

                if(swarm_split == 1 and entered_once4 == 0):        
                    if(leader_idx[0] in near_dest1[0]):
                        #turn the leader towards the new destination
                        new_angle = numpy.arctan2(destination[0][1]-huskyY[leader_idx[0]],destination[0][0]-huskyX[leader_idx[0]])
                        delta_theta[leader_idx[0]] = new_angle - Yaw[leader_idx[0]];

                    if(leader_idx[1] in near_dest1[0]):
                        #turn the leader towards the new destination
                        new_angle = numpy.arctan2(destination[0][1]-huskyY[leader_idx[1]],destination[0][0]-huskyX[leader_idx[1]])
                        delta_theta[leader_idx[1]] = new_angle - Yaw[leader_idx[1]];                    

                    if(leader_idx[2] in near_dest2[0]):
                        #turn the leader towards the new destination
                        new_angle = numpy.arctan2(destination[0][1]-huskyY[leader_idx[2]],destination[0][0]-huskyX[leader_idx[2]])
                        delta_theta[leader_idx[2]] = new_angle - Yaw[leader_idx[2]];

                    if(leader_idx[3] in near_dest2[0]):
                        #turn the leader towards the new destination
                        new_angle = numpy.arctan2(destination[0][1]-huskyY[leader_idx[3]],destination[0][0]-huskyX[leader_idx[3]])
                        delta_theta[leader_idx[3]] = new_angle - Yaw[leader_idx[3]];                                            

                temp_initpoints2 = copy.deepcopy(initpoints)
                #print("init: " , initpoints[:,i])
                M.markers[i].pose.position.x = huskyX[i] #initpoints[0][i] 
                M.markers[i].pose.position.y = huskyY[i] #initpoints[1][i]
                initpoints[0][i] = M.markers[i].pose.position.x
                initpoints[1][i] = M.markers[i].pose.position.y
                
                dist_left_to_cover[i] = numpy.sqrt(pow(temp_initpoints2[1][i]-huskyY[i],2) + pow(temp_initpoints2[0][i]-huskyX[i],2))
                print("speed:" , dist_left_to_cover[i])
                #print("pos:" , huskyX[i] , huskyY[i])
                #print("angle : ", Yaw[i])
                #print("speed: " , dist_left_to_cover[i])
                #print("angular sp:", delta_theta[i])
                sheep_speed[i] = dist_left_to_cover[i]; #can remove the dt if leaders taking more time
                
                # if(dist_left_to_cover[i] < 0.01):
                #     sheep_speed[i] = 0.1
                k = 1;
                H[i].linear.x = sheep_speed[i]
                H[i].angular.z = k*(delta_theta[i]) #removed k*                
                #print("angular" , H[i].angular.z)
            #print("")    
            for i in range(nol):
                print(N + nol)
                #print("LLLLLLLLLLLLLLLLLLLLLLLLL")
                #print(leader_pos1)
                # print("swarm_split: " , swarm_split)
                # print("pred pos: " , LeaderPos[i])
                # print("sheph speed: " , sheperd_speed[i])
                change_in_heading_sheperd[i] = numpy.arctan2(LeaderPos[i,1]-temp_LeaderPos[i,1],LeaderPos[i,0]-temp_LeaderPos[i,0])
                delta_theta_sheperd[i] = change_in_heading_sheperd[i] - Yaw_pred[i];
                #To prevent very fast rotation which can lead to toppling
                if(abs(delta_theta_sheperd[i]) > angle_bound and husky_predX[i] < 0):
                    delta_theta_sheperd[i] = numpy.sign(delta_theta_sheperd[i])*angle_bound
                    if(husky_predX[i] < 0):
                        delta_theta_sheperd[i] = -1*delta_theta_sheperd[i]  #(numpy.sign(delta_theta[i]))*((2*pi)%delta_theta[i])        
                
                if(swarm_split == 1):
                    sheperd_speed[i] = 0.0
                    #nol = 0;
                else:
                    sheperd_speed[i] = 0.35 #1.25*sp[i-N]
                #print("sheperd_speed:",sheperd_speed[i])
                M.markers[i+N].pose.position.x = husky_predX[i] #LeaderPos[0][0]
                M.markers[i+N].pose.position.y = husky_predY[i] #LeaderPos[0][1]
                LeaderPos[i][0] = husky_predX[i]
                LeaderPos[i][1] = husky_predY[i]

                H_pred[i].linear.x = sheperd_speed[i]
                H_pred[i].angular.z = 0.0 #k*(delta_theta_sheperd[i-N])
                # elif(i == N+1):    
                #     M.markers[i].pose.position.x = LeaderPos[1][0]
                #     M.markers[i].pose.position.y = LeaderPos[1][1]

            #print(2*(1.0)/5)        
                # elif(i < N):
                #     M.markers[i].pose.position.x = initpoints[0][i] 
                #     M.markers[i].pose.position.y = initpoints[1][i]

                # if(i >= N):
                #     M.markers[i].pose.position.x = leaderidx[i-N][0]
                #     M.markers[i].pose.position.y = leaderidx[i-N][1]
                #     #print("leader: " , leaderidx[i-N][0] , " " , destination)              

            #S1 , a = moveSheperd(S1,numpy.asarray(dest2).reshape(1,2),1,1)
            GCM = [numpy.average(initpoints[0]),numpy.average(initpoints[1])]
            
            temp_LeaderPos = copy.deepcopy(LeaderPos)
            limit = 1;
            # if(near_swarm1 == 0 and swarm_split == 0):
            #     #D1 = numpy.linalg.norm(initpoints.T - LeaderPos[0],axis=1)
            #     #instead of sheep, lets compare how far from origin
            #     D1 = numpy.linalg.norm(LeaderPos[0],axis=0)
            #     sp[0] = numpy.linalg.norm(LeaderPos[0] - GCM)*(0.1)                                                           
            #     #print("sp1= " , sp1)         
            #     idxD1 = numpy.where(D1 < rs/3.0)            
            #     #if(numpy.size(idxD1) > 0):
            #     if(D1 < limit):
            #         near_swarm1 = 1;
                # else:
                #     LeaderPos[0],a = moveLeader(LeaderPos[0],GCM,sp[0]*dt,1)

            # if(near_swarm2 == 0 and swarm_split == 0):
            #     #D2 = numpy.linalg.norm(initpoints.T - LeaderPos[1],axis=1)
            #     D2 = numpy.linalg.norm(LeaderPos[1],axis=0)
            #     sp[1] = numpy.linalg.norm(LeaderPos[1] - GCM)*(0.1)
            #     #print(" sp2= " , sp2)
            #     idxD2 = numpy.where(D2 < rs/3.0)            
            #     if(D2 < limit):
            #         near_swarm2 = 1
                # else:
                #     LeaderPos[1],a = moveLeader(LeaderPos[1],GCM,sp[1]*dt,1)        
            print("dist betwen sheps ", abs(LeaderPos[0][1] - LeaderPos[1][1]))    
            time_split = rospy.get_rostime().to_sec()
            print("time:",time_split - start_time[0][mc1])
            print("swarm_split",swarm_split)
            if(time_split - start_time[0][mc1] > 1.5):
                if(abs(LeaderPos[0][1] - LeaderPos[1][1]) <= 1.0 and swarm_split == 0):
                    #print("swarm_split",swarm_split)
                    swarm_split = 1; 
                    split_time[0][mc1] = rospy.get_rostime().to_sec() 
                    G1 = numpy.where(initpoints[0] < 0)
                    G2 = numpy.where(initpoints[0] > 0)

                #entered_once3 = 0;



            # if(orient1 == 0 and swarm_split == 1 and entered_once == 1):
            #     #print("GCM0: " , GCM0 , " LeaderPos: " , LeaderPos[0])
            #     LeaderPos[0],a = moveLeader(LeaderPos[0],sub_dest1,1,1)
            #     D1 = numpy.linalg.norm(initpoints.T - LeaderPos[0],axis=1)
            #     idx_D1 = numpy.where(D1 < rs/2.0)
            #     if(numpy.size(idx_D1) > 0):
            #         LeaderPos[0],a = moveLeader(LeaderPos[0],GCM0,1,-1)

            # if(orient2 == 0 and swarm_split == 1 and entered_once == 1):
            #     #print("GCM1: " , GCM1 , " LeaderPos: " , LeaderPos[1])
            #     LeaderPos[1],a = moveLeader(LeaderPos[1],sub_dest2,1,1)
            #     D2 = numpy.linalg.norm(initpoints.T - LeaderPos[1],axis=1)
            #     idx_D2 = numpy.where(D2 < rs/2.0)
            #     if(numpy.size(idx_D2) > 0):
            #         LeaderPos[1],a = moveLeader(LeaderPos[1],GCM1,1,-1)                

            # if(swarm_split==1 and entered_once == 1 and LeaderPos[0][0] <= GCM0[0] and LeaderPos[0][1] >= GCM0[1] and orient1 == 0):
            #     #print("PPPPPPPPPPPPPPPPPPPPPPPPP")
            #     orient1 = 1;        
            # if(swarm_split==1 and entered_once == 1 and LeaderPos[1][0] >= GCM1[0] and LeaderPos[1][1] >= GCM1[1] and orient2 == 0):
            #     orient2 = 1;                    

            #print("GCM: ", GCM , " near_swarm1: " , near_swarm1, " near2= " , near_swarm2 )
            #print("l1 " , LeaderPos[0] , " l2 " , LeaderPos[1])


            D = numpy.linalg.norm(initpoints.T - destination,axis=1)
            near_dest = numpy.where(D < destinationthreshold) # + 5 removed #will reduce this too
            #print("near_dest: " , numpy.size(near_dest))

            D1 = numpy.linalg.norm(initpoints.T - destination1,axis=1)
            away_dest1 = numpy.where(D1 > destinationthreshold)
            near_dest1 = numpy.where(D1 <= destinationthreshold-5)

            D2 = numpy.linalg.norm(initpoints.T - destination2,axis=1)
            away_dest2 = numpy.where(D2 > destinationthreshold)
            near_dest2 = numpy.where(D2 <= destinationthreshold-5)

            #g1 = numpy.intersect1d(away_dest1[0],grp1)
            #g2 = numpy.intersect1d(away_dest2[0],grp2)

            #print("near_dest1= " , near_dest1 , " size= " , numpy.size(near_dest1))
            #print("near_dest2= " , near_dest2)
            dist1 = numpy.linalg.norm(initpoints.T - LeaderPos[0],axis=1)
            dist2 = numpy.linalg.norm(initpoints.T - LeaderPos[1],axis=1)
            g1 = numpy.where(initpoints[0,:] < 0)
            g2 = numpy.where(initpoints[0,:] > 0)
            #print("g1: " , g1)
            #print("g2: " , g2)
            #if(numpy.size(near_dest1) > 0.9*())
            if((near_swarm1 == 1 and near_swarm2 == 1) or swarm_split == 1):
                if(numpy.size(near_dest1) >= 0.9*(numpy.size(g1)) and in1 == 0):
                #if(numpy.size(near_dest1) >= 1): #trying to make the eating as a function of the number of agents reached
                    in1 = 1;


                if(numpy.size(near_dest2) >= 0.9*(numpy.size(g2)) and in2 == 0):
                    in2 = 1;

                #START FEEDING
                if(eaten1 == 0):
                    reached1 = numpy.size(near_dest1)*(1.0)
                    #print("reached1= " , reached1 , " near_dest1= " , (reached1/numpy.size(g1)) , " total= " , numpy.size(near_dest1))
                    M.markers[N+nol].scale.x = M.markers[N+nol].scale.x - size_dec*numpy.size(near_dest1)
                    M.markers[N+nol].scale.y = M.markers[N+nol].scale.y - size_dec*numpy.size(near_dest1)
                    if(M.markers[N+nol].scale.x < 0.2):
                        eaten1 = 1; 

                if(eaten2 == 0):
                    M.markers[N+nol+1].scale.x = M.markers[N+nol+1].scale.x - size_dec*numpy.size(near_dest2)
                    M.markers[N+nol+1].scale.y = M.markers[N+nol+1].scale.y - size_dec*numpy.size(near_dest2)
                    if(M.markers[N+nol+1].scale.x < 0.2):
                        eaten2 = 1; 

            #if(eaten1 == 1 and eaten2 == 1 and eating3 == 1):
                #print("EEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE")


            speed = 0.2;         
            #Lreached added so that if the sheperd reaches the new position after reaching dest1, it comes out of this and follows the function in which 
            #the agetns are in(the first if)   
            # if(entered1 == 1 and (eaten1 == 0 or Lreached1 == 1)):
            #     if((LeaderPos[0][0] < dest1[0][0] or LeaderPos[0][1] < dest1[0][1])):
            #         #dest1[0][1] = destination1[0][1] + 15*numpy.sin(m1);

            #         LeaderPos[0],a = moveLeader(LeaderPos[0],dest1,speed,1) 
            #         D_from_L1 = numpy.linalg.norm(initpoints.T - LeaderPos[0],axis =1)      
            #         close_to_L1 = numpy.where(D_from_L1 < leaderRrep/2.0)
            #         if(numpy.size(close_to_L1) > 0):
            #             LeaderPos[0],a = moveLeader(LeaderPos[0],destination1,speed,-1)    
            #         #print("dest1= " , dest1," " , dest1[0][1] , " m1= " , m1 , " sin(m1)= ", numpy.sin(m1))  
            #         #print("destination1= " , destination1 , " ", destination1[0][1] + 10.0 , " " , destination1[0][1] + 15*numpy.sin(m1))        
            #     else:
            #         Lreached1 = 1;    

            # if(entered2 == 1 and (eaten2 == 0 or Lreached2 == 1)):
            #     if((LeaderPos[1][0] > dest2[0][0] or LeaderPos[1][1] < dest2[0][1])):
            #         #dest2[0][1] = destination2[0][1] + 15*numpy.sin(m2);

            #         LeaderPos[1],a = moveLeader(LeaderPos[1],dest2,speed,1) 
            #         D_from_L2 = numpy.linalg.norm(initpoints.T - LeaderPos[1],axis =1)      
            #         close_to_L2 = numpy.where(D_from_L2 < leaderRrep/2.0)
            #         if(numpy.size(close_to_L2) > 0):
            #             LeaderPos[1],a = moveLeader(LeaderPos[1],destination2,speed,-1)          
            #         #print("dest2= " , dest2 ," " , dest1[0][1], " m2= " , m2 , "sin(m2)= ", numpy.sin(m2))    
            #     else:
            #         Lreached2 = 1;    
            #if(numpy.size(near_dest1) + numpy.size(near_dest2) > 0.9*N):
                #print("STOPPPPPPPPPPPPPPPP")
                #flag = 1;
                #break      
            print("COUNT: " , count)    
            count = count + 1;  
            if(count > 6000):
                print("mc1 : " , mc1 ,"  not completed " , count)

                not_completed[0][mc1] = 1;
                if(eaten1 != 1 or M.markers[N+nol].scale.x > 0.2):
                    #agent_sep[0][mc1] = numpy.size(ind_gcm1) - numpy.size(near_dest1) + numpy.size(ind_gcm2) - numpy.size(near_dest2)
                    #if(numpy.size(ind_gcm1) - numpy.size(near_dest1) > 0):
                    task[0][mc1] = 1;
                    if(eaten2 == 1):
                        agent_sep[0][mc1] = N - numpy.size(near_dest1) - numpy.size(ind_gcm2)
                    else:         
                        agent_sep[0][mc1] = N - numpy.size(near_dest1) - numpy.size(near_dest2) 
                
                if(eaten2 != 1 or M.markers[N+nol+1].scale.x > 0.2):                    
                    task[0][mc1] = task[0][mc1] + 2;
                    if(eaten2 == 1):
                        agent_sep[0][mc1] = N - numpy.size(near_dest2) - numpy.size(ind_gcm1)
                    else:         
                        agent_sep[0][mc1] = N - numpy.size(near_dest1) - numpy.size(near_dest2)                     
                    #if(numpy.size(ind_gcm2) - numpy.size(near_dest2) > 0):
                    #task[0][mc1] = task[0][mc1] + 2
                elif(eaten3 != 1):
                    agent_sep[0][mc1] = N - numpy.size(near_dest)
                    task[0][mc1] = 4
                #agent_sep[0][mc1] = N - numpy.size(near_dest)        
                break

            if(eaten3 == 1):
                #print("one complete");
                break;            

            rate.sleep()      

        #print("SSSSSSSSSSSSSSSSSSSSSSSSSSSSS")
        #print("mc1 : " ,mc1)    
        stop_time[0][mc1] = rospy.get_rostime().to_sec()
        split_diff[0][mc1] = split_time[0][mc1] - start_time[0][mc1]
        diff[0][mc1] = stop_time[0][mc1] - start_time[0][mc1]
        #print("diff: " , diff)
    print("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")
    diff = stop_time - start_time
    split_diff = stop_time - split_time
    print("total diff: " , diff)
    print("FINAL SPLIT DIFF" , split_diff)
    print("not_completed: " , not_completed)
    print("agent_sep: " , agent_sep)
    print("task where problem: " ,task)
    
      


if __name__ == '__main__':
    #x = []
    #y = []
    try:
        main()
    except rospy.ROSInterruptException:
        pass