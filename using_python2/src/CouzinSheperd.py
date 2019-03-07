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

N = 50
temppos = numpy.zeros((N,2))
tempdi = numpy.zeros((N,2))

def getcircularpoints(centre,radius,Nopoints,ang,q):
    #Nopoints = no. of sheperds + 1 
    #print("In getcircularpoints")
    #print("Nopoints: " , Nopoints , "  " , numpy.zeros([1,2]))
    #print("^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^")
    #print("center " , centre , " rad " , radius , " ang " , ang)
    if(Nopoints%2 == 0):
        ang = ang - 0.5*q*(22.0/7)/Nopoints
    points = numpy.zeros([Nopoints,2])    
    for i in range(Nopoints):
        points[i,0] = centre[0]+radius*numpy.cos(ang+((i-ceil((Nopoints)/2))*q*3.14/Nopoints));
        points[i,1] = centre[1]+radius*numpy.sin(ang+((i-ceil((Nopoints)/2))*q*3.14/Nopoints));   
    #print("points in getcircular: " , points)
    #print("OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO")
    return points           

def move( point1,point2,intensity,direction ): 
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


def ang_wrapPositive(angle):
    #print("angle: " , angle)
    #for i in range(len(angle)):
    if(angle < 0):
        angle = angle + 2*pi;
    if(angle > 2*pi):
        angle = angle - 2*pi
    if(angle < -2*pi):
        angle = angle + 2*pi;
    return angle                

def SheepMovement(initpoints,vels,Rrep,Rori,Ratt,s1,dt,phi,omega,S,rs,leader_pos,leaderRrep,leaderRori,leaderRatt,destination,destinationthreshold,nos,leader_vels,destination_other):
    pos= initpoints.T
    #print("pos:")
    #print(pos)
    nol = 0;
    leader_idx = [len(initpoints[0]),len(initpoints[0])+1]
    flag = -1;
    #print("------------------------------------------------")
    #print("size, initpoints: " , len(initpoints[0]) , " N= " , N)
    dist = numpy.zeros(len(initpoints[0]) + nol)
    for i in range(len(initpoints[0])+nol):
        if(i < N):
            dist[i] = (numpy.sqrt(pow((pos[i][1] - destination[0][1]),2) + pow((pos[i][0] - destination[0][0]),2)))
        else:
            dist[i] = (numpy.sqrt(pow((leader_pos[i-N][1] - destination[0][1]),2) + pow((leader_pos[i-N][0] - destination[0][0]),2)))
    #finding all the agents not within the destination threshold    
    idx1 = numpy.where((dist > destinationthreshold))

    for i in range(len(initpoints[0])+nol):
        if(i < N):
            dist[i] = (numpy.sqrt(pow((pos[i][1] - destination_other[0][1]),2) + pow((pos[i][0] - destination_other[0][0]),2)))
        else:
            dist[i] = (numpy.sqrt(pow((leader_pos[i-N][1] - destination_other[0][1]),2) + pow((leader_pos[i-N][0] - destination_other[0][0]),2))) 
    idx2 = numpy.where((dist > destinationthreshold))
    
    idx = numpy.intersect1d(idx1[0],idx2[0]);           
    #print("idx1: " , idx1)
    #print("idx2: " ,idx2)
    #print("idx:" , idx)
    #print("dist: " , dist)
    #print("idx: ", idx[0][:] , " " , len(numpy.where(1 == 2)))    

    #print(a)

    dists = numpy.zeros(len(initpoints[0]) + nol)  # ADDING 1 FOR 1 LEADER FOR A SUB SWARM
    for i in range(len(initpoints[0])+nol):
        #print("i= " , i )
        #print("idx: " , idx)
        #checking if present agent away from destination or not, if yes then proceed 
        # for k in idx:
        #     if(i == k):
        #         flag = 1
        #         break
        #     else:
        #         flag = -1    
        # #print("Flag for ou of destinationthreshold: " , flag)
        # if(flag != -1):
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
                # if(nol >= 1):
                #     print("dists to check if leader1 is in or notused")        
                #     print(dists[N])
                #print("N= ", N , "initpoints= " , len(initpoints[0]) , "nol= " , nol , "distsSize= " , numpy.size(dists))        
                # elif(j == N and i == N):
                #     dists[j] = 0.0
                # elif(j == N):
                #     dists[j] = numpy.sqrt(pow(leader_pos[0]-pos[i,0],2) + pow(leader_pos[1]-pos[i,1],2));   
                # elif(i == N):
                #     dists[j] = numpy.sqrt(pow(pos[j,0]-leader_pos[0],2) + pow(pos[j,1]-leader_pos[1],2));
            #replace ith agent distance
            #print("dists: ", dists)
            vi = numpy.where((dists == 0))
            #print("YYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYY")
            #print(i , "  " , vi)
            dists[i] = 10000;
            
            # for k in leaderidx:
            #     if(i == k):
            #         flag = 1
            #         break
            #     else:
            #         flag = -1                            
            
            # if(flag == -1):
            #print("leaderidx: " , leaderidx , " " , leaderidx[0])
            # if(numpy.intersect1d(i,leaderidx) == i):
            #     #now different repelsion:
            #     v = numpy.where((dists <= leaderRrep))
            #v = numpy.intersect1d(v , v)
            #v =union(intersect(find(dists <= leaderRrep),leaderidx),v);
            
            v = numpy.where((dists <= Rrep))
            v12 = numpy.where(dists <= leaderRrep)   
            v13 = numpy.intersect1d(v12, leader_idx)
            #v2 = numpy.intersect1d(v13,leaderidx)
            #print("LLLLLLLLLLLLLLLLLLLLLLLLLLLLLL")
            #print("v: " , v , " " )                
            #print("leaderidx: " , leaderidx)
            v  = numpy.union1d(v[0],v13)

            # v = []
            # for j in range(len(dists)):
            #     if(j < Rrep):
            #         v.append(j)
            #print("new v: " , v , " " , numpy.size(v) , " " , )
            #di = [0,0];
            nagentFlag = 0;
            dr = [0,0]; drFlag = 0;
            #print("Flag for repulsion: " , v)
            for j in range(numpy.size(v)):
                #make sure the neighbor is inside the visible region
                #vels is the direction
                if(v[j] < N and i < N and v[j] not in ind):
                    deno = numpy.sqrt(pow(vels[v[j]][1],2)+pow(vels[v[j]][0],2))*numpy.sqrt(pow(vels[i][1],2)+pow(vels[i][0],2))
                    if(abs(numpy.arccos(numpy.dot(vels[v[j]],vels[i]))) <= phi):
                        dr = dr + (pos[v[j],:]-pos[i,:])/numpy.linalg.norm((pos[v[j],:]-pos[i,:]),axis = 0)
                        drFlag = 1;

                # elif(v[j] == N and i == N):
                #     if(abs(numpy.arccos((leader_vels[1]*vels[i][1] + leader_vels[0]*vels[i][0]))) <= phi):
                #         dr = dr + (leader_pos[:]-pos[i,:])/numpy.linalg.norm((leader_pos[:]-pos[i,:]),axis = 0)
                #         drFlag = 1;

                elif(v[j] >= N and i < N):
                    if(abs(numpy.arccos((leader_vels[v[j]-N][1]*vels[i][1] + leader_vels[v[j]-N][0]*vels[i][0]))) <= phi):
                        dr = dr + (leader_pos[v[j]-N]-pos[i,:])/numpy.linalg.norm((leader_pos[v[j]-N]-pos[i,:]),axis = 0)
                        drFlag = 1; 

                elif(v[j] < N and i >= N):
                    if(abs(numpy.arccos((vels[v[j]][1]*leader_vels[i-N][1] + vels[v[j]][0]*leader_vels[i-N][0]))) <= phi):   
                        dr = dr + (pos[v[j],:]-leader_pos[i-N])/numpy.linalg.norm((pos[v[j],:]-leader_pos[i-N]),axis = 0) 
                        drFlag = 1;     


            # % orientation - equation 2
            #checking if agent away from destination or not, if no then proceed
     
            zooFlag = 0      
            LzooFlag = 0                
            zoo = [0 ,0]
            #print("Flag for orientation: " , flag)
            #if(flag == -1):
            if(i not in leader_idx):
                dists1 = numpy.where(dists>Rrep)
                dists2 = numpy.where(dists <= Rori)

                v = numpy.intersect1d(dists1 , dists2)
                #print("v: " , v)
                #v =union(intersect(find(dists >= leaderRrep & dists<=leaderRori ),leaderidx),v)
                v11 = numpy.where(dists >= leaderRrep)
                #print("VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV")
                
                v12 = numpy.where(dists <= leaderRori)
                v13 = numpy.intersect1d(v11[0], v12[0])
                #print("v13: ", v13)
                v2 = numpy.intersect1d(v13,leader_idx)
                
                
                #---------FOR-ORIENTATION-TOWARDS-NEAREST-LEADER-------------------------------------------------------------------------------------------------
                if(numpy.size(v2) == 2): # MEANS BOTH LEADER INSIDE, BUT WE NEED ONLY 1
                    if(dists[len(initpoints[0])] <= dists[len(initpoints[0])+1]): #this means leader1
                        v2 = [len(initpoints[0])]
                    else:
                        v2 = [len(initpoints[0]) + 1]

                    #print("v2= " , v2)    
                #else:
                            
                #-------------------------------------------------------------------------------------------------------------
                if(numpy.size(v2) != 0):
                    v = numpy.union1d(v2,v2)
                    #print("i= " , i)
                else:
                    v = numpy.union1d(v2,v)    
     
                for j in range(numpy.size(v)):
                    #deno = numpy.sqrt(pow(vels[v[j]][1],2)+pow(vels[v[j]][0],2))*numpy.sqrt(pow(vels[i][1],2)+pow(vels[i][0],2))
                    if(v[j] < N and i < N):
                        #print(abs(numpy.arccos((vels[v[j]][1]*vels[i][1] + vels[v[j]][0]*vels[i][0]))))
                        deno = numpy.sqrt(pow(vels[v[j]][1],2)+pow(vels[v[j]][0],2))*numpy.sqrt(pow(vels[i][1],2)+pow(vels[i][0],2))
                        if(abs(numpy.arccos(numpy.dot(vels[v[j]],vels[i]))) <= phi):
                        #if(v[j] != N):
                            zoo[0] = zoo[0] + vels[v[j]][0]
                            zoo[1] = zoo[1] + vels[v[j]][1]
                            zooFlag = 1;    
                    elif(v[j] >= N and i < N):
                        if(abs(numpy.arccos((leader_vels[v[j]-N][1]*vels[i][1] + leader_vels[v[j]-N][0]*vels[i][0]))) <= phi):
                            zoo[0] = zoo[0] + leader_vels[v[j]-N][0]
                            zoo[1] = zoo[1] + leader_vels[v[j]-N][1]                                
                            #zooFlag = 1
                            LzooFlag = 1
                    elif(v[j] < N and i >= N):
                        if(abs(numpy.arccos((vels[v[j]][1]*leader_vels[i-N][1] + vels[v[j]][0]*leader_vels[i-N][0]))) <= phi):
                        #if(v[j] != N):
                            zoo[0] = zoo[0] + vels[v[j]][0]
                            zoo[1] = zoo[1] + vels[v[j]][1]
                            zooFlag = 1;                                

            #print("vels: " , vels)
            #print("leader_vels: " , leader_vels)                
     
            # for attraction equation 3    
            #for k in leaderidx:
            #print("Flag for ATTRACTION: " , flag)            
            # if(i in leader_idx):
            #     flag = 1
            #     #break
            # else:
            #     flag = -1                

            da = [0, 0];        
            # if(flag == -1):
            if(i not in  leader_idx): 
                dists1 = numpy.where(dists > Rori)
                dists2 = numpy.where(dists <= Ratt)
                v = numpy.intersect1d(dists2, dists1)
            
                zoaFlag = 0;
                for j in range(numpy.size(v)):
                    #print("v: " , v[0] , " " , v[1] , " " , v)
                    #deno = numpy.sqrt(pow(vels[v[j]][1],2)+pow(vels[v[j]][0],2))*numpy.sqrt(pow(vels[i][1],2)+pow(vels[i][0],2))
                    #if(v[j] == N):
                    if(v[j] < N): #EXTRA
                        deno = numpy.sqrt(pow(vels[v[j]][1],2)+pow(vels[v[j]][0],2))*numpy.sqrt(pow(vels[i][1],2)+pow(vels[i][0],2))
                        if(abs(numpy.arccos(numpy.dot(vels[v[j]],vels[i]))) <= phi):                        
                            da = da + (pos[v[j],:]-pos[i,:])/numpy.linalg.norm((pos[v[j],:]-pos[i,:]),axis = 0) # 2* part EXTRA
                            zoaFlag = 1;    
                    #NO ATTRACTION TOWARDS FELLOW AGENTS WHEN SPLITTING..................
                    # else:
                    #     if(abs(numpy.arccos((vels[v[j]][1]*vels[i][1] + vels[v[j]][0]*vels[i][0]))) <= phi):
                    #         da = da + (pos[v[j],:]-pos[i,:])/numpy.linalg.norm((pos[v[j],:]-pos[i,:]),axis = 0)
                    #         zoaFlag = 1;


                #v = v13        
                #ATTRACTING TOWARDS NEAREST NEIGHBOUR
                # dist_leader = [dists[k] for k in v13]
                # dist_leader_sorted = numpy.sort(dist_leader)
                # #print("dist_leader: " , dist_leader)
                # #print("dist_leader_sorted: " , dist_leader_sorted)
                # if(len(dist_leader) != 0):
                #     idx1 = dist_leader.index(min(dist_leader))
                #     #v = numpy.where(dist_leader)
                #     #v = dist_leader.find(dist_leader_sorted[0])
                #     #v = dist_leader_sorted[0]
                #     v = v13[idx1]
                #     const = 5 #more attraction towards leader
                #     #for j in range(numpy.size(v)):
                #     #print("OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO")
                #     #print("v: " , v)                    
                #     deno = numpy.sqrt(pow(vels[v][1],2)+pow(vels[v][0],2))*numpy.sqrt(pow(vels[i][1],2)+pow(vels[i][0],2))
                #     if(abs(numpy.arccos((vels[v][1]*vels[i][1] + vels[v][0]*vels[i][0]))) <= phi):
                #         da = da + const*(pos[v,:]-pos[i,:])/numpy.linalg.norm((pos[v,:]-pos[i,:]),axis = 0)                    
                #         zoaFlag = 1
                
            # %% Predatory Effect, not interested in this
            dpredFlag=0
            #dpred = [0, 0]
            if(nos != 0):
                #print("S size: "  , numpy.size(S))
                dpred = [0, 0]
                distp = []

                for j in range(nos):
                    distp = numpy.linalg.norm(pos[i] - S,axis = 1)
                    #distp.append(numpy.sqrt(pow(pos[i,0]-S[j,0],2) + pow(pos[i,1]-S[j,1],2)))
                    #print(distp.shape())
                    #distp = numpy.asarray(distp).reshape(1,2)
                v = numpy.where(distp < rs)
                # print("S: " , S , " " , rs)
                #print("distp: " ,  distp)
                # print("v: " , v)
                for j in range(numpy.size(v)):
                    #deno = numpy.sqrt(pow(S[v[0][j]][1],2)+pow(S[v[0][j]][0],2))*numpy.sqrt(pow(vels[i][1],2)+pow(vels[i][0],2))
                    #if(abs(numpy.arccos((S[v[0][j]][1]*vels[i][1] + S[v[0][j]][0]*vels[i][0]))) <= phi):
                    dpred = dpred + (S[v[0][j],:]-pos[i,:])/numpy.linalg.norm((S[v[0][j],:]-pos[i,:]),axis = 0)
                    dpredFlag = 1
            #print("dpred: " , dpred, " nos: " , nos)        
            # %% Leader Effect, interested in this
            #print("Flag for LEADER: " , flag)
            #print("leader_idx: " , leader_idx)
            dleaderFlag1 = 0; dleaderFlag2 = 0;
            dleader1 = [0, 0]; dleader2 = [0, 0]
            #ENTER IF LEADER
            #if(numpy   .intersect1d(leaderidx,i) == i):
            if(i in leader_idx):
                #dleader = [0, 0] was here, now put outside if
                #print("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA")
                # distp = []
                # for j in range(len(destination)):
                #     distp.append(numpy.sqrt(pow(leader_pos[0]-destination[j][0],2) + pow(leader_pos[1]-destination[j][1],2)))
                # v = numpy.where(distp > 0)
                # # #print("+++++++++++++++++++++++++++++++++++++++++++++")
                # #print(v)
                # #print(destination[v[0][0],:])
                # for j in range(numpy.size(v)):
                    #print("vels: " , vels)
                    #print("leaderidx: " , leaderidx[0][0] , " " , v[j][0], " ", v[0][j], " " , v )
                    #deno = numpy.sqrt(pow(leaderidx[v[j][0]][1],2)+pow(leaderidx[v[j][0]][0],2))*numpy.sqrt(pow(vels[i][1],2)+pow(vels[i][0],2))
                    #if(abs(numpy.arccos((leaderidx[v[j][0]][1]*vels[i][1] + leaderidx[v[j][0]][0]*vels[i][0]))) <= phi):
                #print("leader_pos= " , leader_pos)
                if(i == N):
                    dleader1 = dleader1 + (destination[0,:]-leader_pos[i-N,:])/numpy.linalg.norm((destination[0,:]-leader_pos[i-N,:]),axis = 0)        
                    dleaderFlag1 = 1
                    #print("LLLLLLLLLLLLLLLLLLLLLLLLLLLLLL")
                    #print("dleader1: " , dleader1)
                    #print("norm1: " , numpy.linalg.norm(dleader1,axis = 0))
                elif(i == N+1):
                    dleader2 = dleader2 + (destination_other[0,:]-leader_pos[i-N,:])/numpy.linalg.norm((destination_other[0,:]-leader_pos[i-N,:]),axis = 0)        
                    dleaderFlag2 = 1
                    #print("LLLLLLLLLLLLLLLLLLLLLLLLLLLLLL")
                    #print("dleader2: " , dleader2)                    
                    #print("norm2: " , numpy.linalg.norm(dleader2,axis = 0))

                # %% decision making
            di = [0, 0]    
            if(dpredFlag == 1):
                #print("i: " , i , "pos: " , pos[i])
                di = -dpred
                nagentFlag = 1;
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
                    #print("A")    
                # elif(dleaderFlag == 1):
                #     di = di + dleader
                #     nagentFlag = 1
                #     #print("B")
                elif(dleaderFlag1 == 1):
                    di = di + dleader1
                    nagentFlag = 1
                    #print("B1")
                elif(dleaderFlag2 == 1):
                    di = di + dleader2
                    nagentFlag = 1
                    #print("B2")                                    
                elif(LzooFlag == 1):
                    di[0] = di[0] + zoo[0]
                    di[1] = di[1] + zoo[1]
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
                if(numpy.intersect1d(i,leader_idx) == i):
                    di = numpy.asarray(di).reshape(1,2)
                else:    
                    di = di + numpy.random.rand(1,2)*dt
                di = di/numpy.linalg.norm(di)
                #print("normDI= " , numpy.linalg.norm(di) , " " , numpy.linalg.norm(di,axis = 0))
                #print("di: " , di , " " , di[0] , " i= ", i )
                if(i < N):
                    deno = numpy.sqrt(pow(vels[i][1],2)+pow(vels[i][0],2))*numpy.sqrt(pow(di[0][1],2)+pow(di[0][0],2))
                    dtheta = numpy.arccos(((di[0][1]*vels[i][1]) + (di[0][0]*vels[i][0]))/deno)
                    vangle = ang_wrapPositive(numpy.arctan2(vels[i][1],vels[i][0]))
                else:
                    #deno = numpy.sqrt(pow(leader_vels[1],2)+pow(leader_vels[0],2))*numpy.sqrt(pow(di[0][1],2)+pow(di[0][0],2))
                    dtheta = numpy.arccos(((di[0][1]*leader_vels[i-N][1]) + (di[0][0]*leader_vels[i-N][0])))                    
                    vangle = ang_wrapPositive(numpy.arctan2(leader_vels[i-N][1],leader_vels[i-N][0]))
                # if(vels[i][1] == 0.0 and vels[i][0] == 0.0):
                #     vangle = 0.0
                # else:    
                #     vangle = ang_wrapPositive(numpy.arctan(vels[i][1]/vels[i][0]))
                #print("vels[i][1] " , vels[i][1] , " vels[i][0] " , vels[i][0])
                
                #vangle = ang_wrapPositive(numpy.arctan2(vels[i][1],vels[i][0]))
                dangle = ang_wrapPositive(numpy.arctan2(di[0][1],di[0][0]));
                
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
                if(i < N):
                    tempdi[i][0] = numpy.cos(dangle1)
                    tempdi[i][1] = numpy.sin(dangle1)
                    temppos[i][0] = pos[i,0]+numpy.cos(dangle1)*(s)*dt 
                    temppos[i][1] = pos[i,1]+ numpy.sin(dangle1)*(s)*dt
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

def singleherdingPosition(S,Pc,X,Y,destination):
    #print("In HERDING " , S)
    #S2 = copy.deepcopy(S)
    #S2 = [0,0]
    l = numpy.shape(X)
    #Dgcm = numpy.zeros(numpy.shape(X))
    Dgcm = []
    #print("XXXXXXXXXXXX " , numpy.shape(X))
    for i in range(l[0]):  # or len if X is a list and not an array     
        Dgcm.append(numpy.sqrt(numpy.square(X[i] - destination[0]) + numpy.square(Y[i] - destination[1])))
    #idx = numpy.argmax(Dgcm,axis = 0)
    if(len(Dgcm) != 0): # NEW ADDITION , ................
        pos = Dgcm.index(max(Dgcm))
    idx = []
    for i in range(len(Dgcm)):
        if(Dgcm[i] == Dgcm[pos]):
            idx.append(i)
    #print("X and Y at POS: " , X[pos] , " " , Y[pos] , " " , S[0] , " ",S[0,0])
    if(len(idx) != 0):
        agent = numpy.zeros(2)
        agent[0] = X[pos]
        agent[1] = Y[pos]
        S,s2 = move(agent,destination,Pc,-1) #s2 is useless , also move is moveSheperd of the predator function
    #print("S after limiting its steps: " , S , " ssssssssssssssssssssssssssss " )
    return S

def stepspertimestepSheperd(currentpoints,initialpoints,steps): # to limit the step taken per unit time
#steps: steps to be taken towards currentpoints from initialpoints
    #print("currentpoints: " , currentpoints , " initials " , initialpoints,)
    difference=currentpoints-initialpoints    
    #norm=numpy.linalg.norm(difference[:],axis=0)
    norm = numpy.zeros(numpy.shape(currentpoints)[0])
    for i in range(numpy.shape(currentpoints)[0]):
        norm[i] = float(sqrt(pow(difference[i][0],2) + pow(difference[i][1],2)))
    l=numpy.shape(difference)
    #print("Difference " , difference, " DDDDDDDDDDDDDDDDDDDDDDDDDDD " , norm)
    #print("in stepspertime L: " , l ," norm: " , norm ) 
    direction=numpy.zeros((l[0],2))
    for i in range(l[0]):
        step=steps
        if norm[i]<steps:
            step=norm[i]
        if norm[i]==0:
            unitvec=[0,0]
        else:
            unitvec=[difference[i][0]/norm[i],difference[i][1]/norm[i]]
            direction[i,0]=unitvec[0]
            direction[i,1]=unitvec[1]
        #print("diff: " , difference ,"univec: " , unitvec , " norm: " , norm, " CEHK")    
        currentpoints[i,0]=initialpoints[i,0]+unitvec[0]*step
        currentpoints[i,1]=initialpoints[i,1]+unitvec[1]*step
    #print("NEW currentpts: " , currentpoints,)
    #print(" NNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNN")    
    return currentpoints,direction    

def purepredatorCouzinsHighlyParallel(Sh,initpoints,ra,Pd,Pc,destination,q,f,rhoash,rs,I,unitvec):
    mode = 6;
    tempSh = copy.deepcopy(Sh)
    #print("size sh: " , numpy.size(Sh) ," " , numpy.shape(Sh)[0])
    currentmode = []
    GCM = [numpy.mean(initpoints[0]), numpy.mean(initpoints[1])]
    rad = 180*pi/180
    q = 1.5
    ang = numpy.arctan2(GCM[1]-destination[1],GCM[0]-destination[0])
    points = getcircularpoints(GCM,rad,numpy.shape(Sh)[0]+1,ang,q)
    Sh1 = Sh
    I = []

    for i in range(numpy.shape(Sh)[0]):
    #for i in range(numpy.size(Sh)/2):
        #print("Sh: " , numpy.shape(Sh) , " " , numpy.size(Sh)/2)
        midpoint = 0.5*numpy.sum(points[i:i+1,:])
        # if(numpy.size(Sh)/2 > 1):
        #   idx = numpy.argmin(numpy.linalg.norm(Sh1-midpoint,axis = 1)) #CHECK THE OUTPUT OF idx=> CHECKED
        #   I.append(idx)
        #   Sh1[idx,0] = -1000
        #   Sh1[idx,1] = -1000      
        # if(numpy.size(Sh1)/2 == 1):
        idx = numpy.argmin(numpy.linalg.norm(Sh1-midpoint,axis = 1))
        I.append(idx)
        Sh1[idx,0] = -1000
        Sh1[idx,1] = -1000
        #print("idx:.......................")
        #print(idx)
        # I.append(idx)
        # print("idx: " , idx , " Sh1: " , Sh1 , " " , Sh1[0])
        # Sh1[idx,0] = -1000
        # Sh1[idx,1] = -1000
    #print("I: " , I)   
    for i in range(numpy.shape(Sh)[0]):
        #p = path.Path([(0,0), (0, 1), (1, 1), (1, 0)])
        p = path.Path([(GCM[0],GCM[1]), (points[i,0],points[i,1]), (points[i+1,0], points[i+1,1])])
        #print("p: " , p)
        IN = p.contains_points(initpoints.T) #PRINT AND CHECK THIS TOO
        #print("IN IN IN ")
        #print(IN)
        idx = numpy.where(IN == True)
        
        if(numpy.size(idx) != 0):
            #print("idx: " , idx , " " , idx[0])
            Xin = initpoints[0,idx[0]]
            Yin = initpoints[1,idx[0]]
            #print("Xin: " , Xin)
            agents = [initpoints[0,idx[0]],initpoints[1,idx[0]]]
            #print("agents: " , agents)
            x_mean = numpy.mean(unitvec[idx,0])
            y_mean = numpy.mean(unitvec[idx,1])
            #print("mean: " , x_mean , " " , y_mean)
            #numpy.asarray([x_mean,y_mean]).reshape(1,2)
            #print("destination: ",destination)
            # nx = x_mean - destination[0]
            # ny = y_mean - destination[1]
            # norm = numpy.linalg.norm([nx,ny])
            norm1 = numpy.linalg.norm([x_mean,y_mean])
            norm2 = numpy.linalg.norm([destination[0],destination[1]])
            headingangle = abs(numpy.arccos(numpy.dot([x_mean,y_mean],destination)/(norm1*norm2)))
            #print("unitvec:" , unitvec[idx,0] , " norm: " , norm )
            
            #print("dot: " , numpy.dot([x_mean,y_mean],destination))
            #print("agetns mean: " , numpy.mean(agents[0]) , numpy.mean(agents[1]))
            nx = numpy.mean(agents[0]) - destination[0]
            ny = numpy.mean(agents[1]) - destination[1]
            #Dist1 = sqrt(pow(nx,2) + pow(ny,2))
            Dist1 = numpy.linalg.norm([nx,ny])
            #print("Dist1: " , Dist1)
            Dist2 = 20.0 #what is this 10 ??
            tangentangle = 2.0*numpy.arcsin(Dist2/Dist1)
            #print("headingangle: " , headingangle , " tangentangle: " , tangentangle)
            #print("rs/2: " , rs/2.0)
            if(headingangle < tangentangle):
                #%         [destination,~]=move(destination,GCM,5,1);
                Sh[I[i]]  = singleherdingPosition( Sh[I[i]],rs+rs/2,Xin,Yin,destination);
                #currentmode[I[i]] = 1;
            else:
                Sh[I[i]]  = singleherdingPosition(Sh[I[i]],rs/2,Xin,Yin,destination);
                #currentmode[I[i]] = 2;
        else:
            iclose = numpy.argmin(numpy.linalg.norm(initpoints.T - Sh[I[i]] , axis = 1),axis = 0)
            #print("iclose: " , iclose)
            S1 , a = move([Sh[I[i],0],Sh[I[i],1]], [initpoints[0,iclose],initpoints[1,iclose]], rs+rs/2, -1);
            Sh[I[i]][0]=S1[0][0];
            Sh[I[i]][1]=S1[0][1];
    Sh,a = stepspertimestepSheperd(Sh,tempSh,0.2)       
    #print("Sh; " , Sh)     
    return Sh,I        



def main():
    rospy.init_node('Leader_split', anonymous=True)

    pub_marker = rospy.Publisher('leader', MarkerArray , queue_size=1)

    rate = rospy.Rate(10) # 10hz
    destination1x = [-120. , -71. ,-108. ,-140. , -63. , -82. , -78. ,-116. , -63. , -96., -119. , -71.,  -109. , -62. ,-105. , -91. ,-142. , -68. ,-128. ,-133. ,-136. , -80. ,-128. , -59.,
  -130.]
    destination1y =  [  -7.,  108. ,-124.,  -24. , 227. ,  53., -185.,   78.,   90.,   55., -192. ,  68.,
  -167., -147. , -91. , 173. ,  34. , 105. , -22., -146. , -95.  , -5. , 225. , 205.,
   -44.]

#('dest2: ',)
    destination2x = [ 181. , 198. , 169. , 218.,  179. , 171.,  223.,  187.,  240. , 212. , 232. , 239.,
   196. , 248. , 176.  ,243. , 157. , 220. , 179. , 151. , 197. , 196. , 164. , 240.,
   230.]
    destination2y = [ -81. ,-107., -228.,  234., -157., -223. ,-209. ,-141. , 145. , 138. ,  37. ,-162.,
    67. ,  80. , 249. ,  48. ,  42. ,  62.,  103. , 125. ,-151.,  186. , 187., -222.,
  -100.]

#('dest: ',)
    destinationx = [ -63.  , 16. , -57.,   33. , -10. ,  54. ,  11. ,  72.,    7. , -92. ,  47. , -85.,
   -56.  , 40. , -24. , -26. ,  15. ,   7. , -71. ,  -8.,   61. ,  57. ,  21.,  -38.,
    74.]
    destinationy = [  13. , 238. , 159.,  -76. , -20. ,-209. ,  31. , -59. ,-171.  , -6. ,-180. , -20.,
   144., -230. ,  86. ,  69., -127., -119. , -48., -211. , 120. ,  36. , 231.,  -82.,
   142.]

    MC = len(destination1x) 
    MC = 5
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

    MC = 1;
    #diff = [] ; ratio1 = [] ; ratio2 = [] ; SP = []

    it = 1
    time = numpy.zeros((MC,it))
    ratio = numpy.zeros((MC,it))    
    for mc2 in range(MC):
        for mc1 in range(it):
    
            global N, temppos, tempdi
            N = 50
            # tempdi = numpy.zeros((N,2))
            # temppos = numpy.zeros((N,2))

            
            Xstartpos = 0.0
            Ystartpos = -20.0
            threshold = 0

            nos = 2
            S1 = [1.0,-65.0]
            S2 = [-1.0, 25.0]
            #leader_pos = numpy.asarray(leader_pos1).reshape(nol,2)
            Sh = numpy.asarray([S1,S2]).reshape(nos,2)
            #leaderidx = numpy.zeros(nol)
            #leaderidx = N
            #NT = N + nol;

            #function [ X,Y,vels] = puneetsCouzins(X,Y,vels,Rrep,Rori,Ratt,s,dt,phi,omega)
            k = 15
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
            
            unitvec=numpy.zeros((N,2))
            for i in range(N):
                unitvec[i][0] = numpy.random.rand(1,1)
                unitvec[i][1] = numpy.random.rand(1,1)


            # leader_vels1 = numpy.zeros((1,2)); 
            # leader_vels2 = numpy.zeros((1,2))
            # leader_vels1[0][0] = numpy.random.rand(1,1)    
            # leader_vels1[0][1] = numpy.random.rand(1,1)    
            # leader_vels2[0][0] = numpy.random.rand(1,1)    
            # leader_vels2[0][1] = numpy.random.rand(1,1)    

            # LeaderVels = numpy.asarray([leader_vels1,leader_vels2]).reshape(nol,2)
            
            #vels_temp = vels
            #unitvec=numpy.zeros((2,N))
            #temppos = numpy.zeros((N,2))
            #tempdi = numpy.zeros((N,2))
            Rrep = 1
            Rori = Rrep + 6
            Ratt = Rori + 3
            s = 1.0
            s1 = 1.0  
            dt = 0.1    
            phi = 200*(pi)/180
            omega = 2

            ra = 2.0
            Pd = ra*log(N)
            Pc = ra
            rhoash = 1.0
            f = pow(N,0.5)
            I = []
            q = 1.5

            leaderRrep = 2    
            leaderRori = Rrep + 15 #The agents between the distance of 10(leaderRori) and 20(leaderRatt) get attracted towards them
            leaderRatt = 0 # We
            #CHANGNG DESTICATION1 with 2 and seeing the efficiency
            destination1 = [-55.0,-15.0]
            destination2 = [55.0,-15.0]
            destination = [0.0,-50.0]
            #destination = numpy.asarray(destination).reshape(1,2)
            #destination1 = numpy.asarray(destination1).reshape(1,2)
            #destination2 = numpy.asarray(destination2).reshape(1,2)

            destinationthreshold = 10.0
            rs = 15;

            M = MarkerArray()    
            l = 0;
            shape = Marker.CUBE;
            for i in range(N+ nos + 3): #to include shepherd too
                marker = Marker()
                #print("i " , i)
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

                if(i >= N and i<N+nos):
                    marker.ns = "shep"
                    marker.pose.position.x = Sh[i - N][0];
                    marker.pose.position.y = Sh[i - N][1];
                    marker.color.r = 1.0;
                    marker.color.g = 0.0;

                if(i >= N+nos):
                    marker.ns = "Point"
                    marker.type = Marker.CYLINDER
                    # marker.pose.position.x = destination[0]
                    # marker.pose.position.y = destination[1]
                    marker.scale.x = 2*destinationthreshold;
                    marker.scale.y = 2*destinationthreshold;                    
                    if(l == 0):
                        marker.pose.position.x = destination1[0]
                        marker.pose.position.y = destination1[1]
                        l = 1;
                    elif(l == 1):
                        marker.pose.position.x = destination2[0]
                        marker.pose.position.y = destination2[1]
                        l = 2;
                    elif(l == 2):
                        marker.pose.position.x = 100000
                        marker.pose.position.y = destination[1]
                        marker.scale.x = 2*(destinationthreshold+5);
                        marker.scale.y = 2*(destinationthreshold+5);                                                

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
            eaten1 = 0; eaten2 = 0; eaten3 = 0;
            entered1 = 0; entered2= 0;
            dest1 = numpy.zeros((1,2)) ; dest2 = numpy.zeros((1,2))
            Lreached1 = 0 ; Lreached2 = 0;
            swarm_split = 0;
            g1 = []; g2 = []
            group_formed = 0;
            sp1 = 0; sp2 = 0;
            entered = 0; done = 0;
            start_time = rospy.get_rostime() .to_sec()
            speed = 0.35 + mc2*(0.05)
            while not rospy.is_shutdown():

                pub_marker.publish(M)
                #print("Sh: " , Sh)
                if(eaten1 == 0 or eaten2 == 0):
                    #print("A")
                    #print("g1: " , g1)
                    #print("g2: " , g2)
                    initpoints,unitvec =  SheepMovement(initpoints,unitvec,Rrep,Rori,Ratt,s1,dt,phi,omega,Sh,rs,[],
                        leaderRrep,leaderRori,leaderRatt,numpy.asarray(destination1).reshape(1,2),destinationthreshold,nos,[],numpy.asarray(destination2).reshape(1,2))
                #Sh,I = purepredatorCouzinsHighlyParallel(Sh,initpoints,ra,Pd,Pc,destination,q,f,rhoash,rs,I,unitvec)
                    # if(eaten2 == 1):
                    #   initpoints,unitvec =  SheepMovement(initpoints,unitvec,Rrep,Rori,Ratt,s1,dt,phi,omega,Sh,rs,[],
                    #       leaderRrep,leaderRori,leaderRatt,numpy.asarray(destination1).reshape(1,2),destinationthreshold,nos,[],numpy.asarray(destination1).reshape(1,2))             
                    # if(eaten1 == 1):
                    #   initpoints,unitvec =  SheepMovement(initpoints[:,g1],unitvec,Rrep,Rori,Ratt,s1,dt,phi,omega,Sh,rs,[],
                    #       leaderRrep,leaderRori,leaderRatt,numpy.asarray(destination2).reshape(1,2),destinationthreshold,nos,[],numpy.asarray(destination2).reshape(1,2))                             
                
                else:
                    #print("B")
                    initpoints,unitvec =  SheepMovement(initpoints,unitvec,Rrep,Rori,Ratt,s1,dt,phi,omega,Sh,rs,[],
                        leaderRrep,leaderRori,leaderRatt,numpy.asarray(destination).reshape(1,2),destinationthreshold,nos,[],numpy.asarray(destination).reshape(1,2))       

                if(swarm_split == 1 and group_formed == 1 and (in1 == 0 or in2 == 0)):
                    #print("initpoints: " , initpoints[:,g1])
                    #   print("unitvec: " , unitvec[g1])
                    #print("CCCCCCCCCCCCCCCc " , "eaten2: " , eaten2)
                    Dth1 = numpy.linalg.norm(initpoints[:,g1].T - destination1 , axis = 1)
                    Dth1_ind = numpy.where(Dth1 <= destinationthreshold)
                    Dth2 = numpy.linalg.norm(initpoints[:,g2].T - destination2 , axis = 1)
                    Dth2_ind = numpy.where(Dth2 <= destinationthreshold)            
                        
                    if(0.9*len(g1) >= len(Dth1_ind[0]) and in1 == 0):
                        Sh0,I = purepredatorCouzinsHighlyParallel(Sh[0].reshape(1,2),initpoints[:,g1],ra,Pd,Pc,destination1,q,f,rhoash,rs,I,unitvec[g1])    
                        Sh[0] = Sh0
                        #print("C1")
                    elif(in1 == 0):
                        in1 = 1;    
                        sl1 = numpy.arctan2(destination[1]-destination1[1],destination[0]-destination1[0])
                        dest1[0][0] = destination1[0] - 7.5*numpy.cos(sl1)
                        dest1[0][1] = destination1[1] - 7.5*numpy.sin(sl1)
                        print("D1")
                    
                    if(0.9*len(g2) >= len(Dth2_ind[0]) and in2 == 0):
                        Sh1,I = purepredatorCouzinsHighlyParallel(Sh[1].reshape(1,2),initpoints[:,g2],ra,Pd,Pc,destination2,q,f,rhoash,rs,I,unitvec[g2])
                        Sh[1] = Sh1
                        #print("C2")
                    elif(in2 == 0): 
                        in2 = 1;
                        sl2 = numpy.arctan2(destination[1]-destination2[1],destination[0]-destination2[0])
                        dest2[0][0] = destination2[0] - 7.5*numpy.cos(sl2)
                        dest2[0][1] = destination2[1] - 7.5*numpy.sin(sl2)
                        print("D2")

                elif(eaten1 == 1 and eaten2 == 1):
                    if(entered == 0):
                        M.markers[N+nos].pose.position.x = destination[0]
                        M.markers[N+nos].pose.position.y = destination[1]
                        # M.markers[N+nos].scale.x = 2*destinationthreshold
                        # M.markers[N+nos].scale.y = 2*destinationthreshold
                        entered = 1;
                    Sh0,I = purepredatorCouzinsHighlyParallel(Sh[0].reshape(1,2),initpoints[:,g1],ra,Pd,Pc,destination,q,f,rhoash,rs,I,unitvec[g1])     
                    Sh1,I = purepredatorCouzinsHighlyParallel(Sh[1].reshape(1,2),initpoints[:,g2],ra,Pd,Pc,destination,q,f,rhoash,rs,I,unitvec[g2]) 
                    Sh[0] = Sh0
                    Sh[1] = Sh1
                

                for i in range(N + nos):
                    #if(i in leaderidx):
                    #print("i in main: " , i)
                    if(i < N):
                        M.markers[i].pose.position.x = initpoints[0][i] 
                        M.markers[i].pose.position.y = initpoints[1][i]
                    
                    elif(i == N):
                        #print("LLLLLLLLLLLLLLLLLLLLLLLLL")
                        #print(leader_pos1)
                        M.markers[i].pose.position.x = Sh[0][0]
                        M.markers[i].pose.position.y = Sh[0][1]
                        
                    elif(i == N+1):    
                        M.markers[i].pose.position.x = Sh[1][0]
                        M.markers[i].pose.position.y = Sh[1][1]        




                # s1 = 0.2
                # s2 = 0.2            
                # if(Sh[0][1] < Sh[1][1]):
                #   Sh[0][1] = Sh[0][1] + s
                #   Sh[1][1] = Sh[1][1] - s
                #   swarm_split = 1;
                # else: 
                #   swarm_split = 1;
                GCM = [numpy.mean(initpoints[0]),numpy.mean(initpoints[1])]
                print("swarm_split: " ,swarm_split , " group_formed: " , group_formed, " done:", done)
                if(swarm_split == 0):
                    #print("SS")
                    negx = numpy.where(initpoints[0] < 0.0)
                    posx = numpy.where(initpoints[0] >= 0.0)
                    if(numpy.size(initpoints[0,negx[0]]) and numpy.size(initpoints[0,posx[0]])):
                        neg_ind = numpy.argmax(initpoints[0,negx[0]]) #LEAST NEGATIVE
                        pos_ind = numpy.argmin(initpoints[0,posx[0]]) #LEAST POSITIVE

                if(Sh[0][1] <= Sh[1][1] and swarm_split == 0):
                    #print("SS")
                    S0,a = move(Sh[0],GCM,sp1,1)
                    S1,a = move(Sh[1],GCM,sp2,1)
                    #print("numpy.linalg.norm(S0 - GCM,axis = 1): " ,numpy.linalg.norm(S0 - GCM,axis = 1))
                    d0 = numpy.linalg.norm(initpoints.T - S0,axis=1)
                    d1 = numpy.linalg.norm(initpoints.T - S1,axis=1)
                    idx0 = numpy.where(d0 <= rs)
                    idx1 = numpy.where(d1 <= rs)
                    if(numpy.size(idx0) > 0):
                        sp1 = 0
                        near_swarm1 = 1;
                    else:
                        sp1 = 0.75                              
                    if(numpy.size(idx1) > 0):
                        sp2 = 0
                        near_swarm2 = 1
                    else:
                        sp2 = 0.75  

                    if(near_swarm2 == 1 and near_swarm1 == 1):                        
                        S0,a = move(Sh[0],GCM,speed,1)
                        S1,a = move(Sh[1],GCM,speed,1)  

                    Sh[0] = S0[0]
                    Sh[1] = S1[0]
                    
                elif(swarm_split == 0):
                    print("SS=1")
                    swarm_split = 1     

                if(swarm_split == 1 and group_formed == 0):
                    print("GROUP")
                    for j in range(N):
                        if(initpoints[0,j] < GCM[0]):
                            g1.append(j)
                        else:
                            g2.append(j)    
                    group_formed = 1;       

                #if(group_formed == 1):
                g1 = []; g2 = [];
                for j in range(N):
                    if(initpoints[0,j] < GCM[0]):
                        g1.append(j)
                    else:
                        g2.append(j)                    
                
                neg_x = max(initpoints[0,g1])
                pos_x = min(initpoints[0,g2])
                #print("neg_x: " , neg_x)
                #print("pos_x: " , pos_x)
                if(abs(pos_x - neg_x) > 15):
                    split_time[mc2][mc1] = rospy.get_rostime().to_sec()
                #     done = 1;
                #     break;

                if(done == 1):
                    break;        

                dist_destination1 = numpy.linalg.norm(initpoints.T - destination2,axis = 1)
                inside_threshold_destination1 = numpy.where(dist_destination1 <= destinationthreshold)         

                if(in1 == 1 and eaten1 == 0):
                    #print("Task1")
                    print(M.markers[N+nos].scale.x)
                    M.markers[N+nos].scale.x = M.markers[N+nos].scale.x - 0.02*numpy.size(inside_threshold_destination1)
                    M.markers[N+nos].scale.y = M.markers[N+nos].scale.y - 0.02*numpy.size(inside_threshold_destination1)
                    if(M.markers[N+nos].scale.x <= 0.2):
                        print("eaten1: " , eaten1)
                        eaten1 = 1;     

                dist_destination2 = numpy.linalg.norm(initpoints.T - destination2,axis = 1)
                inside_threshold_destination2 = numpy.where(dist_destination2 <= destinationthreshold)                                 

                if(in2 == 1 and eaten2 == 0):
                    #print("Task2 " ,  " " , M.markers[N+nos+1].scale.x) 
                    M.markers[N+nos+1].scale.x = M.markers[N+nos+1].scale.x - 0.02*numpy.size(inside_threshold_destination2)
                    M.markers[N+nos+1].scale.y = M.markers[N+nos+1].scale.y - 0.02*(numpy.size(inside_threshold_destination2))
                    if(M.markers[N+nos+1].scale.x <= 0.2):
                        eaten2 = 1;     


                if(in1 == 1 and (Sh[0][0] > dest1[0][0] or Sh[0][1] > dest1[0][1]) and eaten1 == 0):
                    print("Move1")
                    print("destination1: " , destination1)
                    print("dest1: " , dest1)
                    print("shep1: " , Sh[0])
                    S0,a = move(Sh[0],copy.deepcopy(dest1),1,1)
                    Sh[0] = S0[0]
                    D0 = numpy.linalg.norm(initpoints[:,g1].T - S0,axis=1)
                    print()
                    D0_ind = numpy.where(D0 < rs)
                    D0_sorted = numpy.argsort(D0)
                    print("D0_ind: " , D0_ind)
                    if(numpy.size(D0_ind) > 0):
                        print("-----------------------------")
                        #S0,a = move(Sh[0],destination1,1.0,-1)         
                        S0,a = move(Sh[0],copy.deepcopy(initpoints[:,g1[D0_sorted[0]]]),1.0,-1)         
                    Sh[0][0] = S0[0][0]
                    Sh[0][1] = S0[0][1]
                        
                if(in2 == 1 and (Sh[1][0] < dest2[0][0] or Sh[1][1] > dest2[0][1]) and eaten2 == 0):    
                    print("Move2")
                    print("destination2: " , destination2)
                    print("dest2: " , dest2)
                    print("shep2: " , Sh[1])
                    S1,a = move(Sh[1],copy.deepcopy(dest2),1,1)
                    Sh[1] = S1[0]
                    D1 = numpy.linalg.norm(initpoints[:,g2].T - S1,axis=1)
                    D1_ind = numpy.where(D1 < rs)
                    D1_sorted = numpy.argsort(D1)
                    print("D1_ind: " , D1_ind)
                    if(numpy.size(D1_ind) > 0):
                        print("++++++++++++++++++++++++++++++")
                        S1,a = move(Sh[1],copy.deepcopy(initpoints[:,g2[D1_sorted[0]]]),1.0,-1)         
                    Sh[1][0] = S1[0][0]
                    Sh[1][1] = S1[0][1]



                dist_destination = numpy.linalg.norm(initpoints.T - destination,axis = 1)
                inside_threshold_destination = numpy.where(dist_destination <= destinationthreshold)         

                if(eaten1 == 1 and eaten2 == 1):
                    inside_destination = N - numpy.size(out)
                    M.markers[N+nos+2].scale.x = M.markers[N+nos+2].scale.x - 0.2*(numpy.size(inside_threshold_destination))
                    M.markers[N+nos+2].scale.y = M.markers[N+nos+2].scale.y - 0.2*(numpy.size(inside_threshold_destination))
                    if(M.markers[N+nos+2].scale.x <= 0.2):
                        eaten3 = 1;                             
                

                count = count + 1;            
                if(count > 5000):
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