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
        ang = ang - 0.5*q*(22.0/7   )/Nopoints
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

def SheepMovement(initpoints,vels,Rrep,Rori,Ratt,s1,dt,phi,omega,S,rs,leader_pos,leaderRrep,leaderRori,leaderRatt,destination,destinationthreshold,nos,leader_vels,destination_other,nol):
    pos= initpoints.T
    #print("pos:")
    #print(pos)
    if(nol > 0):
        #nol = nos
        #nos = 0;
        #leader_idx = [len(initpoints[0]),len(initpoints[0])+1]
        leader_idx = [len(initpoints[0])]
        S = []
    else:
        #nol = 0;
        leader_idx = []
    flag = -1;
    #print("------------------------------------------------")
    #print("size, initpoints: " , len(initpoints[0]) , " N= " , N)
    dist = numpy.zeros(len(initpoints[0]) + nol)
    for i in range(len(initpoints[0])+nol):
        if(i < N):
            #print("destination: " , destination)
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
        for k in idx:
            if(i == k):
                flag = 1
                break
            else:
                flag = -1    
        #print("Flag for ou of destinationthreshold: " , flag)
        if(flag != -1):
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
                #if(abs(numpy.arccos(numpy.dot(vels[v[j],1] - vels[i,1] , vels[v[j],0] - vels[i,0]))) <= phi ):
                #print("dot: ", numpy.dot(vels[v[j],:],  vels[i,:]) )
                #print("j: " , j , " " , v[j] , " " , v[0] ," " ,(vels[v[j]][1] - vels[i][1]), " " , vels[v[j]][0] - vels[i][0])
                #print("div: " , (vels[v[j]][1] - vels[i][1])/(vels[[j]][0] - vels[i][0]))
                #print("vels: " , vels)
                #print("ANGLE: " ,numpy.arccos(vels[v[j]][1]*vels[i][1] + vels[v[j]][0]*vels[i][0]))                
                #deno = numpy.sqrt(pow(vels[v[j]][1],2)+pow(vels[v[j]][0],2))*numpy.sqrt(pow(vels[i][1],2)+pow(vels[i][0],2))
                #print("deno: " , vels[0][1] , )
                #if(abs(numpy.arccos((vels[v[j]][1]*vels[i][1] + vels[v[j]][0]*vels[i][0])/deno)) <= phi):
                #if(abs(numpy.arccos((vels[v[0][j]][1] - vels[i][1])/(vels[v[0][j]][0] - vels[i][0]))) <= phi or (vels[v[0][j]][0] == vels[i][0])):
                    #print("TTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT")
                    #print("norm: " , pos[v[0][j],:] , " " , pos[i,:], " " , numpy.linalg.norm((pos[v[0][j],:]-pos[i,:]),axis = 0)) 
                if(v[j] < N and i < N and v[j] not in ind):
                    deno = numpy.sqrt(pow(vels[v[j]][1],2)+pow(vels[v[j]][0],2))*numpy.sqrt(pow(vels[i][1],2)+pow(vels[i][0],2))
                    if(abs(numpy.arccos((vels[v[j]][1]*vels[i][1] + vels[v[j]][0]*vels[i][0])/deno)) <= phi):
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
            #print("leaderidx: " , leaderidx , " pppppppppppppp")
            #for k in leaderidx:
            if(i in leader_idx):
                flag = 1
                #break
            else:
                flag = -1  

            zooFlag = 0      
            LzooFlag = 0                
            zoo = [0 ,0]
            #print("Flag for orientation: " , flag)
            if(flag == -1):
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
                    #print("v2: " , v2 , "dists1: " , dists[len(initpoints[0])] , " d2: " , dists[len(initpoints[0]) + 1])
                    #print("v: " , v , " " )
                # #print("v2: " , v2)
                # #print("len_dest: " , len(destination) , " " , numpy.size(v) , " " , len(v))
                #v = numpy.union1d(v2,v)

                #zoo = [0 ,0]
                #zooFlag = 0

                for j in range(numpy.size(v)):
                    #deno = numpy.sqrt(pow(vels[v[j]][1],2)+pow(vels[v[j]][0],2))*numpy.sqrt(pow(vels[i][1],2)+pow(vels[i][0],2))
                    if(v[j] < N and i < N):
                        #print(abs(numpy.arccos((vels[v[j]][1]*vels[i][1] + vels[v[j]][0]*vels[i][0]))))
                        deno = numpy.sqrt(pow(vels[v[j]][1],2)+pow(vels[v[j]][0],2))*numpy.sqrt(pow(vels[i][1],2)+pow(vels[i][0],2))
                        if(abs(numpy.arccos((vels[v[j]][1]*vels[i][1] + vels[v[j]][0]*vels[i][0])/deno)) <= phi):
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
            if(i in leader_idx):
                flag = 1
                #break
            else:
                flag = -1                

            da = [0, 0];        
            if(flag == -1):  
                dists1 = numpy.where(dists > Rori)
                dists2 = numpy.where(dists <= Ratt)
                v = numpy.intersect1d(dists2, dists1)
                #v =union(intersect(find(dists >= leaderRori & dists<=leaderRatt),leaderidx),v);
                # v11 = numpy.where(dists >= leaderRori)
                # v12 = numpy.where(dists <= leaderRatt)
                # v13 = numpy.intersect1d(v11, v12)
                # v2 = numpy.intersect1d(v13,N)
                #v = numpy.union1d(v2,v)
                #v = v2 # EXTRA LINE=> CHECKING ATTRACTION TOWWRDS LEADER ONLY 
                # #print("dists : ")
                # #print(dists)
                #print("attraction v: " , v)
                #da = [0, 0];
                zoaFlag = 0;
                for j in range(numpy.size(v)):
                    #print("v: " , v[0] , " " , v[1] , " " , v)
                    #deno = numpy.sqrt(pow(vels[v[j]][1],2)+pow(vels[v[j]][0],2))*numpy.sqrt(pow(vels[i][1],2)+pow(vels[i][0],2))
                    #if(v[j] == N):
                    if(v[j] < N): #EXTRA
                        deno = numpy.sqrt(pow(vels[v[j]][1],2)+pow(vels[v[j]][0],2))*numpy.sqrt(pow(vels[i][1],2)+pow(vels[i][0],2))
                        if(abs(numpy.arccos((vels[v[j]][1]*vels[i][1] + vels[v[j]][0]*vels[i][0])/deno)) <= phi):                        
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

    return temppos.T,tempdi,leader_pos,leader_vels                    


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


def main():
    rospy.init_node('Leader', anonymous=True)

    pub_marker = rospy.Publisher('leader', MarkerArray , queue_size=1)

    rate = rospy.Rate(10) # 10hz
 
    global N, temppos, tempdi
    N = 15
    # tempdi = numpy.zeros((N,2))
    # temppos = numpy.zeros((N,2))

    
    Xstartpos = -5.0
    Ystartpos = -20.0
    threshold = 0

    nos = 2
    S1 = [1.0,-65.0]
    S2 = [-1.0, 25.0]
    #leader_pos = numpy.asarray(leader_pos1).reshape(nol,2)
    Sh = numpy.asarray([S1,S2]).reshape(nos,2)
    nol = nos
    leader_vels1 = numpy.zeros((1,2)); 
    leader_vels2 = numpy.zeros((1,2))
    leader_vels1[0][0] = numpy.random.rand(1,1)    
    leader_vels1[0][1] = numpy.random.rand(1,1)    
    leader_vels2[0][0] = numpy.random.rand(1,1)    
    leader_vels2[0][1] = numpy.random.rand(1,1)    

    LeaderVels = numpy.asarray([leader_vels1,leader_vels2]).reshape(nol,2)
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
    destination1 = [-55.0,20.0]
    destination2 = [55.0,20.0]
    destination = [0.0,40.0]
    #destination = numpy.asarray(destination).reshape(1,2)
    #destination1 = numpy.asarray(destination1).reshape(1,2)
    #destination2 = numpy.asarray(destination2).reshape(1,2)

    destinationthreshold = 10.0
    rs = 12.5;

    M = MarkerArray()    
    l = 0;
    shape = Marker.CUBE;
    for i in range(N+ nos + 2): #to include shepherd too
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
   
        
        # if(i in leaderidx):
        #     marker.color.r = 1.0;
        #     marker.color.g = 0.0;


        if(i >= N and i<N+nos):
            marker.ns = "leader"
            marker.pose.position.x = Sh[i - N][0];
            marker.pose.position.y = Sh[i - N][1];
            marker.color.r = 1.0;
            marker.color.g = 0.0;

        if(i >= N+nos):
            marker.ns = "Point"
            marker.type = Marker.CYLINDER
            # marker.pose.position.x = destination[0]
            # marker.pose.position.y = destination[1]
            if(l == 0):
                marker.pose.position.x = destination1[0]
                marker.pose.position.y = destination1[1]
                l = 1;
            elif(l == 1):
                marker.pose.position.x = destination2[0]
                marker.pose.position.y = destination2[1]
                l = 2;

            marker.color.b = 1.0;
            marker.color.r = 0.0;
            marker.scale.x = 2*destinationthreshold;
            marker.scale.y = 2*destinationthreshold;
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
    eaten1 = 0; eaten2 = 0;
    entered1 = 0; entered2= 0;
    dest1 = numpy.zeros((1,2)) ; dest2 = numpy.zeros((1,2))
    Lreached1 = 0 ; Lreached2 = 0;
    swarm_split = 0;
    g1 = []; g2 = []
    group_formed = 0;
    sp1 = 0; sp2 = 0;
    entered = 0;
    enter_once = 0;
    sub_dest1 = [0,0] ; sub_dest2 = [0,0];
    leader_front1 = 0; leader_front2 = 0;
    leader_front21 = 0; leader_front22 = 0;
    dest_reached1 = 0; dest_reached2 = 0;
    start_time = rospy.get_rostime()
    while not rospy.is_shutdown():

        pub_marker.publish(M)
        #print("Sh: " , Sh)
        if(swarm_split == 0): #FIRST
            initpoints,unitvec,a,a =  SheepMovement(initpoints,unitvec,Rrep,Rori,Ratt,s1,dt,phi,omega,Sh,rs,[],
                leaderRrep,leaderRori,leaderRatt,numpy.asarray(destination1).reshape(1,2),destinationthreshold,nos,[],numpy.asarray(destination2).reshape(1,2),0)

        elif(swarm_split == 1 and (leader_front1 == 0 and leader_front2 == 0)): #leader to come in front and the agents to ignore them
            # initpoints,unitvec,a,a =  SheepMovement(initpoints,unitvec,Rrep,Rori,Ratt,s1,dt,phi,omega,Sh,rs,[],
            #     leaderRrep,leaderRori,leaderRatt,numpy.asarray(destination1).reshape(1,2),destinationthreshold,2,[],numpy.asarray(destination2).reshape(1,2),0)
            print("leader_front1: " , leader_front1 , " leader_front2: " , leader_front2)
            if(enter_once == 0):
                ind_gcm1 = numpy.where(initpoints[0,:] < 0)
                ind_gcm2 = numpy.where(initpoints[0,:] > 0)
                gcm1 = [numpy.average(initpoints[0,ind_gcm1[0]]),numpy.average(initpoints[1,ind_gcm1[0]])]
                gcm2 = [numpy.average(initpoints[0,ind_gcm2[0]]),numpy.average(initpoints[1,ind_gcm2[0]])]

                m1 = numpy.arctan2(destination1[1]-gcm1[1],destination1[0]-gcm1[0])
                sub_dest1[0] = gcm1[0] + 5.0*numpy.cos(m1)
                sub_dest1[1] = gcm1[1] + 5.0*numpy.sin(m1)

                m2 = numpy.arctan2(destination2[1]-gcm2[1],destination2[0]-gcm2[0])
                sub_dest2[0] = gcm2[0] + 5.0*numpy.cos(m2)
                sub_dest2[1] = gcm2[1] + 5.0*numpy.sin(m2)
                enter_once = 1;

        #AFTER SPLITTING: FIRST LEADER COMES IN NEW POSITION, THEN HERDS, THEN AGAIN NEW POSITION AND THEN HERD AGAIN            
        if(leader_front1 == 1 and dest_reached1 == 0): #Leaders herd the agetns
            #print("leader_front1: " , leader_front1)
            initpoints,unitvec,S0,LeaderVels0 =  SheepMovement(initpoints,unitvec,Rrep,Rori,Ratt,s1,dt,phi,omega,[],rs,numpy.asarray(Sh[0]).reshape(1,2),leaderRrep,leaderRori,leaderRatt,numpy.asarray(destination1).reshape(1,2),destinationthreshold,0,leader_vels1,numpy.asarray(destination2).reshape(1,2),1)        
        elif(dest_reached1 == 1 and (eaten1 == 0 or leader_front21 == 0)):#move leader to new position
            #print("dest_reached1: " , dest_reached1)
            gcm1 = [numpy.average(initpoints[0,ind_gcm1[0]]),numpy.average(initpoints[1,ind_gcm1[0]])]
            m1 = numpy.arctan2(destination[1]-gcm1[1],destination[0]-gcm1[0])
            sub_dest1[0] = gcm1[0] + 5.0*numpy.cos(m1)
            sub_dest1[1] = gcm1[1] + 5.0*numpy.sin(m1)
            initpoints,unitvec,a,a = SheepMovement(initpoints,unitvec,Rrep,Rori,Ratt,s1,dt,phi,omega,[],rs,[],leaderRrep,leaderRori,leaderRatt,numpy.asarray(destination).reshape(1,2),destinationthreshold,0,leader_vels1,numpy.asarray(destination).reshape(1,2),0)
        elif(eaten1 == 1 and leader_front21 == 1):
            #print("eaten1: " , eaten1 , " leader_front21: " , leader_front21)
            initpoints,unitvec,S0,LeaderVels0 =  SheepMovement(initpoints,unitvec,Rrep,Rori,Ratt,s1,dt,phi,omega,[],rs,numpy.asarray(Sh[0]).reshape(1,2),leaderRrep,leaderRori,leaderRatt,numpy.asarray(destination).reshape(1,2),destinationthreshold,0,leader_vels1,numpy.asarray(destination).reshape(1,2),1)    


        if(leader_front2 == 1 and dest_reached2 == 0): #Leaders herd the agetns             
            initpoints,unitvec,S1,LeaderVels1 =  SheepMovement(initpoints,unitvec,Rrep,Rori,Ratt,s1,dt,phi,omega,[],rs,numpy.asarray(Sh[1]).reshape(1,2),leaderRrep,leaderRori,leaderRatt,numpy.asarray(destination2).reshape(1,2),destinationthreshold,0,leader_vels2,numpy.asarray(destination1).reshape(1,2),1)        
        elif(dest_reached2 == 1 and (eaten2 == 0 or leader_front22 == 0)):#move leader to new position and start eating   
            gcm2 = [numpy.average(initpoints[0,ind_gcm2[0]]),numpy.average(initpoints[1,ind_gcm2[0]])]
            m2 = numpy.arctan2(destination[1]-gcm2[1],destination[0]-gcm2[0])
            sub_dest2[0] = gcm2[0] + 5.0*numpy.cos(m2)
            sub_dest2[1] = gcm2[1] + 5.0*numpy.sin(m2) 
            initpoints,unitvec,a,a = SheepMovement(initpoints,unitvec,Rrep,Rori,Ratt,s1,dt,phi,omega,[],rs,[],leaderRrep,leaderRori,leaderRatt,numpy.asarray(destination).reshape(1,2),destinationthreshold,0,leader_vels2,numpy.asarray(destination).reshape(1,2),0)       
        elif(eaten2 == 1 and leader_front22 == 1):
            initpoints,unitvec,S1,LeaderVels1 =  SheepMovement(initpoints,unitvec,Rrep,Rori,Ratt,s1,dt,phi,omega,[],rs,numpy.asarray(Sh[1]).reshape(1,2),leaderRrep,leaderRori,leaderRatt,numpy.asarray(destination).reshape(1,2),destinationthreshold,0,leader_vels2,numpy.asarray(destination).reshape(1,2),1)                        

        if(leader_front1 == 1 or (eaten1 == 1 and leader_front21 == 1)):
            Sh[0][0] = S0[0][0]
            Sh[0][1] = S0[0][1]
            LeaderVels[0] = LeaderVels0[0]
        if(leader_front2 == 1 or (eaten2 == 1 and leader_front22 == 1)):    
            Sh[1][0] = S1[0][0]
            Sh[1][1] = S1[0][1]            
            LeaderVels[1] = LeaderVels1[0]

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
        #print("split: " ,swarm_split)
        if(swarm_split == 0):
            #print("SS")
            negx = numpy.where(initpoints[0] < 0.0)
            posx = numpy.where(initpoints[0] >= 0.0)
            if(numpy.size(initpoints[0,negx[0]]) and numpy.size(initpoints[0,posx[0]])):
                neg_ind = numpy.argmax(initpoints[0,negx[0]]) #LEAST NEGATIVE
                pos_ind = numpy.argmin(initpoints[0,posx[0]]) #LEAST POSITIVE



        if(swarm_split == 0):    
            S0,a = move(Sh[0],GCM,sp1,1)
            d0 = numpy.linalg.norm(initpoints.T - S0,axis=1)
            idx0 = numpy.where(d0 <= rs)
            if(numpy.size(idx0) > 0):
                #S0,a = move(Sh[0],GCM,0.25,1)
                sp1 = 0.25
            else:
                sp1 = 0.75;
            Sh[0] = S0[0]
        elif((swarm_split == 1 and leader_front1 == 0) or dest_reached1 == 1):
            #print("leader_front1: " , leader_front1)
            print("sub_dest1: " , sub_dest1 , " Sh[0]: " , Sh[0] , " gcm1:" , gcm1)
            S0,a = move(Sh[0],sub_dest1,1.5,1)
            d0 = numpy.linalg.norm(initpoints.T - Sh[0],axis = 1)
            sorted_d0 = numpy.sort(d0)
            arg_sort_d0 = numpy.argsort(d0)
            #print("sorted_d0: " , sorted_d0)
            if(sorted_d0[0] < rs):
                #print("yolo")
                S0,a = move(S0,initpoints[:,arg_sort_d0[0]],1,-1)
            Sh[0] = S0[0]
                
        if(swarm_split == 0):            
            S1,a = move(Sh[1],GCM,sp2,1)
            d1 = numpy.linalg.norm(initpoints.T - S1,axis=1)
            idx1 = numpy.where(d1 <= rs)
            if(numpy.size(idx1) > 0):                
                sp2 = 0.25
            else:
                sp2 = 0.75;                        
            Sh[1] = S1[0]    
        elif((swarm_split == 1 and leader_front2 == 0) or dest_reached2 == 1): #BOTH TIMES LEADER MOVING HERE
            #print("sub_dest2: " , sub_dest2 , " Sh[1]: " , Sh[1] , " gcm2:" , gcm2)
            S1,a = move(Sh[1],sub_dest2,1,1)
            d1 = numpy.linalg.norm(initpoints.T - Sh[1],axis = 1)
            sorted_d1 = numpy.sort(d1)
            arg_sort_d1 = numpy.argsort(d1)
            if(sorted_d1[0] < rs):
                S1,a = move(S1,initpoints[:,arg_sort_d1[0]],1,-1)            
            Sh[1] = S1[0]    



        if(swarm_split == 1 and Sh[0][0] < gcm1[0] and Sh[0][1] > gcm1[1] and leader_front1 == 0):
            leader_front1 = 1;

        if(swarm_split == 1 and Sh[1][0] > gcm2[0] and Sh[1][1] > gcm2[1]  and leader_front2 == 0):
            leader_front2 = 1;            

        #PUT THE CONDITION FOR NEW DESTINATION    
        if(dest_reached1 == 1 and Sh[0][0] > gcm1[0] and Sh[0][1] > gcm1[1] and leader_front21 == 0):
            leader_front21 = 1;    

        if(dest_reached2 == 1 and Sh[1][0] < gcm2[0] and Sh[1][1] > gcm2[1] and leader_front22 == 0):
            leader_front22 = 1;                


        if(abs(Sh[0][1] - Sh[1][1]) < 10.0  and swarm_split == 0):
            print("SS");
            swarm_split = 1

        if(dest_reached1 == 1 and eaten1 == 0): #move leader and start eating
            M.markers[N+nos].scale.x = M.markers[N+nos].scale.x - 0.2
            M.markers[N+nos].scale.y = M.markers[N+nos].scale.y - 0.2
            if(M.markers[N+nos].scale.x < 0.25):
                eaten1 = 1;

        if(dest_reached2 == 1 and eaten2 == 0): #move leader and start eating
            M.markers[N+nos+1].scale.x = M.markers[N+nos+1].scale.x - 0.2
            M.markers[N+nos+1].scale.y = M.markers[N+nos+1].scale.y - 0.2
            if(M.markers[N+nos+1].scale.x < 0.25):
                eaten2 = 1;


        if(leader_front1 == 1 and dest_reached1 == 0):
            #print("")
            dest_threshold1 = numpy.linalg.norm(initpoints[:,ind_gcm1[0]].T-destination1,axis=1)
            ind_dest_thres1 = numpy.where(dest_threshold1 <= destinationthreshold)
            if(numpy.size(ind_gcm1)*(0.9) <= numpy.size(ind_dest_thres1)):
                dest_reached1 = 1;        
        if(leader_front2 == 1 and dest_reached2 == 0):
            dest_threshold2 = numpy.linalg.norm(initpoints[:,ind_gcm2[0]].T-destination2,axis=1)        
            ind_dest_thres2 = numpy.where(dest_threshold2 <= destinationthreshold)
            if(numpy.size(ind_gcm2)*(0.9) <= numpy.size(ind_dest_thres2)):
                dest_reached2 = 1;        


        count = count + 1;
        if(count > 4000):
            print("COUNT MORE THAN " , count)
            break        

        rate.sleep()        

    stop_time = rospy.get_rostime()
    diff = stop_time - start_time
    print("diff: " , diff.to_sec())    






if __name__ == '__main__':
    #x = []
    #y = []
    try:
        main()
    except rospy.ROSInterruptException:
        pass
