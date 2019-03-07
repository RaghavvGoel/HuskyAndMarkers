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

pi =  22.0/7.0

N = 50
temppos = numpy.zeros((N,2))
tempdi = numpy.zeros((N,2))

#function angle = ang_wrapPositive(angle)
# for i = 1:length(angle)
#     if angle(i) < 0
#         angle(i) = angle(i)+2*pi;
#     end
    
#     if angle(i) > 2*pi
#         angle(i) = angle(i) - 2*pi;
#     end
    
#     if angle(i) < -2*pi
#         angle(i) = angle(i) + 2*pi;
#     end
# end

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

def SheepMovement(initpoints,vels,Rrep,Rori,Ratt,s1,dt,phi,omega,S,rs,leader_pos,leaderRrep,leaderRori,leaderRatt,destination,destinationthreshold,nol,leader_vels,destination_other,leader_idx):
    pos= initpoints.T
    #print("pos:")
    #print(pos)
    #print("SH=" , S , " shape: " , numpy.shape(S))
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

    dists = numpy.zeros(len(initpoints[0]) + nol)  # ADDING ! FOR 1 LEADER FOR A SUB SWARM
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
                # if(numpy.size(v2) == 2): # MEANS BOTH LEADER INSIDE, BUT WE NEED ONLY 1
                #     if(dists[len(initpoints[0])] <= dists[len(initpoints[0])+1]): #this means leader1
                #         v2 = [len(initpoints[0])]
                #     else:
                #         v2 = [len(initpoints[0]) + 1]

                    #print("v2= " , v2)    
                #else:
                            
                #-------------------------------------------------------------------------------------------------------------
                # if(numpy.size(v2) != 0):
                #     v = numpy.union1d(v2,v2)
                #     #print("i= " , i)
                # else:
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
                if(i == leader_idx[0]):
                    dleader1 = dleader1 + (destination[0,:]-pos[i,:])/numpy.linalg.norm((destination[0,:]-pos[i,:]),axis = 0)        
                    dleaderFlag1 = 1
                    #print("LLLLLLLLLLLLLLLLLLLLLLLLLLLLLL")
                    #print("dleader1: " , dleader1)
                    #print("norm1: " , numpy.linalg.norm(dleader1,axis = 0))
                elif(i == leader_idx[1]):
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
                    #print("B")
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


def main():
    rospy.init_node('Leader', anonymous=True)

    pub_marker = rospy.Publisher('leader', MarkerArray , queue_size=1)

    rate = rospy.Rate(10) # 10hz

    global N, temppos, tempdi
    N = 40
    # tempdi = numpy.zeros((N,2))
    # temppos = numpy.zeros((N,2))

    
    Xstartpos = 0.0
    Ystartpos = -20.0
    threshold = 0

    nol = 1
    leader_pos1 = [-1.0,5.0]
    leader_pos2 = [1.0,-45.0]
    LeaderPos = numpy.asarray(leader_pos1).reshape(nol,2)
    #LeaderPos = numpy.asarray([leader_pos1,leader_pos2]).reshape(nol,2)
    #leaderidx = numpy.zeros(nol)
    leaderidx = N
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
    
    vels=numpy.zeros((N,2))
    for i in range(N):
        vels[i][0] = numpy.random.rand(1,1)
        vels[i][1] = numpy.random.rand(1,1)

    leader_vels1 = numpy.zeros((1,2)); 
    leader_vels2 = numpy.zeros((1,2))
    leader_vels1[0][0] = numpy.random.rand(1,1)    
    leader_vels1[0][1] = numpy.random.rand(1,1)    
    leader_vels2[0][0] = numpy.random.rand(1,1)    
    leader_vels2[0][1] = numpy.random.rand(1,1)    

    #LeaderVels = numpy.asarray([leader_vels1,leader_vels2]).reshape(nol,2)
    LeaderVels = numpy.asarray([leader_vels1]).reshape(nol,2)

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
    Rrep = 1
    Rori = Rrep + 5
    Ratt = Rori + 3
    s = 1
    s1 = 1.0  
    dt = 0.1    
    phi = 200*(pi)/180
    omega = 2

    S = []

    leaderRrep = 2    
    leaderRori = Rrep + 15 #The agents between the distance of 10(leaderRori) and 20(leaderRatt) get attracted towards them
    leaderRatt = 0 # We

    destination1 = [-45.0,25.0]
    destination2 = [45.0,25.0]    
    destination = [0.0,50.0]
    destination = numpy.asarray(destination).reshape(1,2)
    destination1 = numpy.asarray(destination1).reshape(1,2)
    destination2 = numpy.asarray(destination2).reshape(1,2)

    destinationthreshold = 10.0
    rs = 15;

    M = MarkerArray()    
    l = 0;
    shape = Marker.CUBE;
    for i in range(N+ nol + 2): #to include shepherd too
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


        if(i >= N and i<N+nol):
            marker.ns = "shep"
            marker.pose.position.x = LeaderPos[i - N][0];
            marker.pose.position.y = LeaderPos[i - N][1];
            marker.color.r = 1.0;
            marker.color.g = 0.0;

        if(i >= N+nol):
            marker.ns = "Point"
            marker.type = Marker.CYLINDER
            if(l == 0):
                marker.pose.position.x = destination1[0][0]
                marker.pose.position.y = destination1[0][1]
                l = 1;
            elif(l == 1):
                marker.pose.position.x = destination2[0][0]
                marker.pose.position.y = destination2[0][1]
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
    orient1 = 0; orient2 = 0;
    entered_once = 0;
    sub_dest1 = [0.0,0.0] ; sub_dest2 = [0.0,0.0];
    swarm_split = 0;
    leader_idx = [];
    entered_once2 = 0;
    while not rospy.is_shutdown():

        pub_marker.publish(M)

        #print("initpoints before:")
        #print(initpoints) 
        #print("before: WWWWWWWWWWWWWWWW")
        #print(vels)
        #print(initpoints )
        #print("before: ",initpoints[:,1] , " lead: " , leader_pos2)
        temp_ini = initpoints[:,1]
        temp_lead = leader_pos2

        if(((near_swarm1 == 1 and near_swarm2 == 1) or swarm_split == 1) and (eaten1 == 0 or eaten2 == 0)):
            print("KKKKKKKKKKKK KKKKKKKKKKKKKK")
            print("eaten: " , eaten1 , " " , eaten2)
            #if(swarm_split == 0):
                #ORITENT BOTH
            if(entered_once == 0):
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
                print("HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH")    
                initpoints,vels =  SheepMovement(initpoints,vels,Rrep,Rrep,Ratt,1.5*s1,dt,phi,omega,LeaderPos,rs,[],leaderRrep,leaderRori,leaderRatt,numpy.asarray([0,0]).reshape(1,2),0,0,[],numpy.asarray([0,0]).reshape(1,2),leader_idx)    

            #else:    
            #if(entered_once2 == 0):
                ind_l1 = numpy.random.choice(numpy.size(ind_gcm1),1)
                ind_l2 = numpy.random.choice(numpy.size(ind_gcm2),1)
                print("ind_l1: " , ind_l1 ," size: " , numpy.size(ind_gcm1) , " " ,ind_l1[0])
                print("ind_l2: " , ind_l2," size: " , numpy.size(ind_gcm2) , " " , ind_gcm2[0][ind_l2[0]])
                leader_idx = [ind_gcm1[0][ind_l1[0]],ind_gcm2[0][ind_l2[0]]]
                #print("leader_idx: " , leader_idx)
                M.markers[leader_idx[0]].color.r = 1.0
                #M.markers[leader_idx[0]].color.g = 0.0
                M.markers[leader_idx[1]].color.r = 1.0
                #M.markers[leader_idx[1]].color.g = 0.0                
                #entered_once2 = 1;
            initpoints,vels =  SheepMovement(initpoints,vels,Rrep,Rori,Ratt,s1,dt,phi,omega,S,rs,LeaderPos,leaderRrep,leaderRori,leaderRatt,destination1,destinationthreshold,nol,LeaderVels,destination2,leader_idx)
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
        
        elif((near_swarm1 == 0 or near_swarm2 == 0 or swarm_split == 0) and (eaten1 == 0 and eaten2 == 0)):#Rori same as Rrep => swarm state
            initpoints,vels =  SheepMovement(initpoints,vels,Rrep,Rrep,Ratt,(1.5)*s1,dt,phi,omega,LeaderPos,rs,[],leaderRrep,leaderRori,leaderRatt,numpy.asarray([0,0]).reshape(1,2),0,nol,[],numpy.asarray([0,0]).reshape(1,2),leader_idx)        
            print("swarm_split: " , swarm_split)
        
        elif(eaten1 == 1 and eaten2 == 1):
            #print("LLLLLLLLLLLLLLLLLLLLLLLLLLL")
            initpoints,vels =  SheepMovement(initpoints,vels,Rrep,Rori,Ratt,s1,dt,phi,omega,S,rs,leader_idx,leaderRrep,leaderRori,leaderRatt,destination,destinationthreshold/3,nol,LeaderVels,destination,leader_idx)                        

        #print("before: ",temp_ini , " lead: " , temp_lead)
        #print("initpoints after: " , initpoints[:,1], " lead: " , leader_pos2)
        #print(initpoints)
        #print("AFTER:ZZZZZZZZZZZZZZZZZZZZZZ")
        #print(vels.T)
        #print("AAAAAAAAAAAAAAA")
        #print(vels_temp)
        #leader_pos[0] = leader_pos1;
        #leader_pos[1] = leader_pos2;

        for i in range(N + nol):
            #if(i in leaderidx):
            #print("i in main: " , i)
            if(i < N):
                M.markers[i].pose.position.x = initpoints[0][i] 
                M.markers[i].pose.position.y = initpoints[1][i]
            elif(i == N):
                #print("LLLLLLLLLLLLLLLLLLLLLLLLL")
                #print(leader_pos1)
                M.markers[i].pose.position.x = LeaderPos[0][0]
                M.markers[i].pose.position.y = LeaderPos[0][1]
                
            elif(i == N+1):    
                M.markers[i].pose.position.x = LeaderPos[1][0]
                M.markers[i].pose.position.y = LeaderPos[1][1]

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

        if(near_swarm1 == 0 and swarm_split == 0):
            D1 = numpy.linalg.norm(initpoints.T - LeaderPos[0],axis=1)
            sp1 = numpy.linalg.norm(LeaderPos[0] - GCM)*(0.3)                                                           
            #print("sp1= " , sp1)         
            # idxD1 = numpy.where(D1 < rs/5.0)            
            # if(numpy.size(idxD1) > 0):
            #     near_swarm1 = 1;
            # else:
            LeaderPos[0],a = moveLeader(LeaderPos[0],GCM,sp1*dt,1)

        # if(near_swarm2 == 0 and swarm_split == 0):
        #     D2 = numpy.linalg.norm(initpoints.T - LeaderPos[1],axis=1)
        #     sp2 = numpy.linalg.norm(LeaderPos[1] - GCM)*(0.3)
        #     #print(" sp2= " , sp2)
        #     idxD2 = numpy.where(D2 < rs/3.0)            
        #     if(numpy.size(idxD2) > 0):
        #         near_swarm2 = 1
        #     else:
        #         LeaderPos[1],a = moveLeader(LeaderPos[1],GCM,sp2*dt,1)        

        if(LeaderPos[0][1] <= Ystartpos - 14.0 and swarm_split == 0):
            swarm_split = 1;        

        # if(orient1 == 0 and swarm_split == 1 and entered_once == 1):
        #     print("GCM0: " , GCM0 , " LeaderPos: " , LeaderPos[0])
        #     LeaderPos[0],a = moveLeader(LeaderPos[0],sub_dest1,1,1)
        #     D1 = numpy.linalg.norm(initpoints.T - LeaderPos[0],axis=1)
        #     idx_D1 = numpy.where(D1 < rs/2.0)
        #     if(numpy.size(idx_D1) > 0):
        #         LeaderPos[0],a = moveLeader(LeaderPos[0],GCM0,1,-1)

        # if(orient2 == 0 and swarm_split == 1 and entered_once == 1):
        #     print("GCM1: " , GCM1 , " LeaderPos: " , LeaderPos[1])
        #     LeaderPos[1],a = moveLeader(LeaderPos[1],sub_dest2,1,1)
        #     D2 = numpy.linalg.norm(initpoints.T - LeaderPos[1],axis=1)
        #     idx_D2 = numpy.where(D2 < rs/2.0)
        #     if(numpy.size(idx_D2) > 0):
        #         LeaderPos[1],a = moveLeader(LeaderPos[1],GCM1,1,-1)                

        # if(swarm_split==1 and entered_once == 1 and LeaderPos[0][0] <= GCM0[0] and LeaderPos[0][1] >= GCM0[1] and orient1 == 0):
        #     print("PPPPPPPPPPPPPPPPPPPPPPPPP")
        #     orient1 = 1;        
        # if(swarm_split==1 and entered_once == 1 and LeaderPos[1][0] >= GCM1[0] and LeaderPos[1][1] >= GCM1[1] and orient2 == 0):
        #     orient2 = 1;                    

        #print("GCM: ", GCM , " near_swarm1: " , near_swarm1, " near2= " , near_swarm2 )
        #print("l1 " , LeaderPos[0] , " l2 " , LeaderPos[1])


        D = numpy.linalg.norm(initpoints.T - destination,axis=1)
        near_dest = numpy.where(D < destinationthreshold)
        #print("near_dest: " , numpy.size(near_dest))

        D1 = numpy.linalg.norm(initpoints.T - destination1,axis=1)
        away_dest1 = numpy.where(D1 > destinationthreshold)
        near_dest1 = numpy.where(D1 <= destinationthreshold)

        D2 = numpy.linalg.norm(initpoints.T - destination2,axis=1)
        away_dest2 = numpy.where(D2 > destinationthreshold)
        near_dest2 = numpy.where(D2 <= destinationthreshold)

        #g1 = numpy.intersect1d(away_dest1[0],grp1)
        #g2 = numpy.intersect1d(away_dest2[0],grp2)

        #print("near_dest1= " , near_dest1 , " size= " , numpy.size(near_dest1))
        #print("near_dest2= " , near_dest2)
        dist1 = numpy.linalg.norm(initpoints.T - LeaderPos[0],axis=1)
        #dist2 = numpy.linalg.norm(initpoints.T - LeaderPos[1],axis=1)
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
            if(in1 == 1 and eaten1 == 0):
                reached1 = numpy.size(near_dest1)*(1.0)
                #print("reached1= " , reached1 , " near_dest1= " , (reached1/numpy.size(g1)) , " total= " , numpy.size(near_dest1))
                M.markers[N+nol].scale.x = M.markers[N+nol].scale.x - 0.25
                M.markers[N+nol].scale.y = M.markers[N+nol].scale.y - 0.25
                if(M.markers[N+nol].scale.x < 0.1):
                    eaten1 = 1; 

            if(in2 == 1 and eaten2 == 0):
                M.markers[N+nol+1].scale.x = M.markers[N+nol+1].scale.x - 0.25
                M.markers[N+nol+1].scale.y = M.markers[N+nol+1].scale.y - 0.25
                if(M.markers[N+nol+1].scale.x < 0.1):
                    eaten2 = 1; 

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
        #         print("dest1= " , dest1," " , dest1[0][1] , " m1= " , m1 , " sin(m1)= ", numpy.sin(m1))  
        #         print("destination1= " , destination1 , " ", destination1[0][1] + 10.0 , " " , destination1[0][1] + 15*numpy.sin(m1))        
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
        #         print("dest2= " , dest2 ," " , dest1[0][1], " m2= " , m2 , "sin(m2)= ", numpy.sin(m2))    
        #     else:
        #         Lreached2 = 1;    
        #if(numpy.size(near_dest1) + numpy.size(near_dest2) > 0.9*N):
            #print("STOPPPPPPPPPPPPPPPP")
            #flag = 1;
            #break        

        count = count + 1;
        if(count > 2000):
            print("COUNT MORE THAN " , count)
            break        

        rate.sleep()        




if __name__ == '__main__':
    #x = []
    #y = []
    try:
        main()
    except rospy.ROSInterruptException:
        pass
