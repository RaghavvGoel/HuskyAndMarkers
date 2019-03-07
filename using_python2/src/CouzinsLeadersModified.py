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
N = 30
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

def SheepMovement(initpoints,vels,Rrep,Rori,Ratt,s1,dt,phi,omega,S,rs,leaderidx,leaderRrep,leaderRori,leaderRatt,destination,destinationthreshold):
    pos= initpoints.T
    #print("pos:")
    #print(pos)
    flag = -1;

    dist = numpy.zeros(len(initpoints[0]))
    for i in range(len(initpoints[0])):
        dist[i] = (numpy.sqrt(pow((pos[i][1] - destination[0][1]),2) + pow((pos[i][0] - destination[0][0]),2)))
    #finding all the agents not within the destination threshold    
    idx = numpy.where((dist > destinationthreshold))
    #print("dist: " , dist)
    #print("idx: ", idx[0][:] , " " , len(numpy.where(1 == 2)))    

    dists = numpy.zeros(len(initpoints[0]))
    for i in range(len(initpoints[0])):
        #checking if present agent away from destination or not, if yes then proceed 
        for k in idx[0]:
            if(i == k):
                flag = 1
                break
            else:
                flag = -1    
        
        if(flag != -1):
            for j in range(len(initpoints[0])):
                #dists.append(numpy.sqrt(pow(pos[j,0]-pos[i,0],2) + pow(pos[j,1]-pos[i,1],2)));
                dists[j] = numpy.sqrt(pow(pos[j,0]-pos[i,0],2) + pow(pos[j,1]-pos[i,1],2));
            
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
            #     v = numpy.intersect1d(v , leaderidx)
            #v =union(intersect(find(dists <= leaderRrep),leaderidx),v);
            #else:
            v = numpy.where((dists <= Rrep))
            v1 = numpy.where(dists <= leaderRrep)   
            v2 = numpy.intersect1d(v1,leaderidx)
            # print("LLLLLLLLLLLLLLLLLLLLLLLLLLLLLL")
            # print("v: " , v , " " )
            # print("v2: " , v2 , " " )
            # print("leaderidx: " , leaderidx)
            v  = numpy.union1d(v2,v[0])

            # v = []
            # for j in range(len(dists)):
            #     if(j < Rrep):
            #         v.append(j)
            #print("new v: " , v , " " , numpy.size(v) , " " , )
            #di = [0,0];
            nagentFlag = 0;
            dr = [0,0]; drFlag = 0;
        
            for j in range(numpy.size(v)):
                #make sure the neighbor is inside the visible region
                #vels is the direction
                #if(abs(numpy.arccos(numpy.dot(vels[v[j],1] - vels[i,1] , vels[v[j],0] - vels[i,0]))) <= phi ):
                #print("dot: ", numpy.dot(vels[v[j],:],  vels[i,:]) )
                #print("j: " , j , " " , v[j] , " " , v[0] ," " ,(vels[v[j]][1] - vels[i][1]), " " , vels[v[j]][0] - vels[i][0])
                #print("div: " , (vels[v[j]][1] - vels[i][1])/(vels[[j]][0] - vels[i][0]))
                #print("vels: " , vels)
                #print("ANGLE: " ,numpy.arccos(vels[v[j]][1]*vels[i][1] + vels[v[j]][0]*vels[i][0]))                
                deno = numpy.sqrt(pow(vels[v[j]][1],2)+pow(vels[v[j]][0],2))*numpy.sqrt(pow(vels[i][1],2)+pow(vels[i][0],2))
                #print("deno: " , vels[0][1] , )
                if(abs(numpy.arccos((vels[v[j]][1]*vels[i][1] + vels[v[j]][0]*vels[i][0])/deno)) <= phi):
                #if(abs(numpy.arccos((vels[v[0][j]][1] - vels[i][1])/(vels[v[0][j]][0] - vels[i][0]))) <= phi or (vels[v[0][j]][0] == vels[i][0])):
                    #print("TTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT")
                    #print("norm: " , pos[v[0][j],:] , " " , pos[i,:], " " , numpy.linalg.norm((pos[v[0][j],:]-pos[i,:]),axis = 0)) 
                    dr = dr + (pos[v[j],:]-pos[i,:])/numpy.linalg.norm((pos[v[j],:]-pos[i,:]),axis = 0)
                    drFlag = 1;     


            # % orientation - equation 2
            #checking if agent away from destination or not, if no then proceed
            for k in leaderidx:
                if(i == k):
                    flag = 1
                    break
                else:
                    flag = -1  

            zooFlag = 0                      
            zoo = [0 ,0]

            if(flag == -1):
                dists1 = numpy.where(dists>Rrep)
                dists2 = numpy.where(dists <= Rori)

                v = numpy.intersect1d(dists1 , dists2)
                #print("v: " , v)
                #v =union(intersect(find(dists >= leaderRrep & dists<=leaderRori ),leaderidx),v)
                v11 = numpy.where(dists >= leaderRrep)
                #print("VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV")
                
                v12 = numpy.where(dists <= leaderRori)
                v13 = numpy.intersect1d(v11, v12)
                #print("v13: ", v13)
                v2 = numpy.intersect1d(v13,leaderidx)
                #print("v2: " , v2)
                # print("v: " , v , " " )
                # print("v2: " , v2)
                # print("len_dest: " , len(destination) , " " , numpy.size(v) , " " , len(v))
                #v = numpy.union1d(v2,v)

                #zoo = [0 ,0]
                #zooFlag = 0
                zoo1 = [0,0]
                zoo2 = [0,0]
                for j in range(numpy.size(v)):
                    deno = numpy.sqrt(pow(vels[v[j]][1],2)+pow(vels[v[j]][0],2))*numpy.sqrt(pow(vels[i][1],2)+pow(vels[i][0],2))
                    if(abs(numpy.arccos((vels[v[j]][1]*vels[i][1] + vels[v[j]][0]*vels[i][0]))) <= phi):
                        zoo1[0] = zoo1[0] + vels[v[j]][0]
                        zoo1[1] = zoo1[1] + vels[v[j]][1]
                        zooFlag = 1

                for j in range(len(v2)):
                    deno = numpy.sqrt(pow(vels[v2[j]][1],2)+pow(vels[v2[j]][0],2))*numpy.sqrt(pow(vels[i][1],2)+pow(vels[i][0],2))
                    if(abs(numpy.arccos((vels[v2[j]][1]*vels[i][1] + vels[v2[j]][0]*vels[i][0]))) <= phi):
                        zoo2[0] = zoo2[0] + vels[v2[j]][0]
                        zoo2[1] = zoo2[1] + vels[v2[j]][1]
                        zooFlag = 1                    
                #zoo[0] = 0.2*(zoo1[0]) + 0.8*(zoo2[0])
                #zoo[1] = 0.2*(zoo1[1]) + 0.8*(zoo2[1])        
                zoo[0] = (zoo1[0]) + (zoo2[0])
                zoo[1] = (zoo1[1]) + (zoo2[1])                        

            # for attraction equation 3    
            for k in leaderidx:
                if(i == k):
                    flag = 1
                    break
                else:
                    flag = -1                

            da = [0, 0];
            da1 = [0,0];
            da2 = [0,0];
            zoaFlag = 0;        
            if(flag == -1):  
                dists1 = numpy.where(dists > Rori)
                dists2 = numpy.where(dists <= Ratt)
                v = numpy.intersect1d(dists2, dists1)
                #v =union(intersect(find(dists >= leaderRori & dists<=leaderRatt),leaderidx),v);
                v11 = numpy.where(dists >= leaderRori)
                v12 = numpy.where(dists <= leaderRatt)
                v13 = numpy.intersect1d(v11, v12)
                #v2 = numpy.intersect1d(v13,leaderidx)
                #v = numpy.union1d(v2,v)

                #da = [0, 0];
                #zoaFlag = 0;
                # da1 = [0,0];
                # da2 = [0,0];
                for j in range(numpy.size(v)):
                    #print("v: " , v[0] , " " , v[1] , " " , v)
                    deno = numpy.sqrt(pow(vels[v[j]][1],2)+pow(vels[v[j]][0],2))*numpy.sqrt(pow(vels[i][1],2)+pow(vels[i][0],2))
                    if(abs(numpy.arccos((vels[v[j]][1]*vels[i][1] + vels[v[j]][0]*vels[i][0]))) <= phi):
                        da1 = da1 + (pos[v[j],:]-pos[i,:])/numpy.linalg.norm((pos[v[j],:]-pos[i,:]),axis = 0)
                        zoaFlag = 1;


                #v = v13        
                #ATTRACTING TOWARDS NEAREST NEIGHBOUR
                # dist_leader = [dists[k] for k in v13]
                # dist_leader_sorted = numpy.sort(dist_leader)
                # print("dist_leader: " , dist_leader)
                # print("dist_leader_sorted: " , dist_leader_sorted)
                # if(len(dist_leader) != 0):
                #     idx1 = dist_leader.index(min(dist_leader))
                #     #v = numpy.where(dist_leader)
                #     #v = dist_leader.find(dist_leader_sorted[0])
                #     #v = dist_leader_sorted[0]
                #     v = v13[idx1]
                #     const = 5 #more attraction towards leader
                #     #for j in range(numpy.size(v)):
                #     print("OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO")
                #     print("v: " , v)                    
                for j in range(len(v13)):
                    deno = numpy.sqrt(pow(vels[v13[j]][1],2)+pow(vels[v13[j]][0],2))*numpy.sqrt(pow(vels[i][1],2)+pow(vels[i][0],2))
                    if(abs(numpy.arccos((vels[v13[j]][1]*vels[i][1] + vels[v13[j]][0]*vels[i][0]))) <= phi):
                        da2 = da2 + (pos[v13[j],:]-pos[i,:])/numpy.linalg.norm((pos[v13[j],:]-pos[i,:]),axis = 0)                    
                        zoaFlag = 1

                #da[0] = 0.2*(da1[0]) + 0.8*(da2[0])
                #da[1] = 0.2*(da1[1]) + 0.8*(da2[1])
                da[0] = da1[0]
                da[1] = da1[1]       


            # %% Predatory Effect, not interested in this
            dpredFlag=0
            #dpred = [0, 0]
            if(len(S) != 0):
                dpred = [0, 0]
                distp = []
                for j in range(len(S)):
                    distp.append(numpy.sqrt(pow(pos[i,0]-S[j,0],2) + pow(pos[i,1]-S[j,1],2)))
                v = numpy.where(distp < rs)
                for j in range(len(v)):
                    #deno = numpy.sqrt(pow(S[v[0][j]][1],2)+pow(S[v[0][j]][0],2))*numpy.sqrt(pow(vels[i][1],2)+pow(vels[i][0],2))
                    #if(abs(numpy.arccos((S[v[0][j]][1]*vels[i][1] + S[v[0][j]][0]*vels[i][0]))) <= phi):
                    dpred = dpred + (S[v[0][j],:]-pos[i,:])/numpy.linalg.norm((S[v[0][j],:]-pos[i,:]),axis = 0)
                    dpredFlag = 1

            # %% Leader Effect, interested in this
            dleaderFlag = 0
            dleader = [0, 0]
            #ENTER IF LEADER
            if(numpy.intersect1d(leaderidx,i) == i):
                #dleader = [0, 0] was here, now put outside if
                distp = []
                for j in range(len(destination)):
                    distp.append(numpy.sqrt(pow(pos[i,0]-destination[j][0],2) + pow(pos[i,1]-destination[j][1],2)))
                v = numpy.where(distp > 0)
                for j in range(len(v)):
                    #print("vels: " , vels)
                    #print("leaderidx: " , leaderidx[0][0] , " " , v[j][0], " ", v[0][j], " " , v )
                    #deno = numpy.sqrt(pow(leaderidx[v[j][0]][1],2)+pow(leaderidx[v[j][0]][0],2))*numpy.sqrt(pow(vels[i][1],2)+pow(vels[i][0],2))
                    #if(abs(numpy.arccos((leaderidx[v[j][0]][1]*vels[i][1] + leaderidx[v[j][0]][0]*vels[i][0]))) <= phi):
                    dleader = dleader + (destination[v[j][0],:]-pos[i,:])/numpy.linalg.norm((destination[v[j][0],:]-pos[i,:]),axis = 0)        
                    dleaderFlag = 1

                # %% decision making
            di = [0, 0]    
            if(dpredFlag == 1):
                di = -dpred
                nagentFlag = 1;
                s = s1*2;
            else:
                s = s1;    
                # print("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")
                # print("dr: " , dr , " dleader: " , dleader , " zoo: " , zoo , " da: ", da, " da2 " , da2)
                # print("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")
                # if(drFlag == 1 and dleaderFlag == 1):
                #     di = di - dr + dleader
                #     nagentFlag = 1

                if(drFlag == 1):
                    di = di - dr + da2;
                    nagentFlag = 1
                    #print("A")    
                elif(dleaderFlag == 1):
                    di = di + dleader + da2
                    nagentFlag = 1
                    #print("B")
                elif(zooFlag == 1 and zoaFlag == 1):
                    di[0] = di[0] + 0.5*(zoo[0]+da[0]) + da2[0]
                    di[1] = di[1] + 0.5*(zoo[1]+da[1]) + da2[1]
                    nagentFlag = 1
                    #print("C")
                elif(zooFlag == 1):
                    di[0] = di[0] + zoo[0] + da2[0];
                    di[1] = di[1] + zoo[1] + da2[1];
                    nagentFlag = 1    
                    #print("D")
                elif(zoaFlag == 1):
                    #print("di inside elif: " , di , " " , da)
                    di[0] = di[0] + da[0] + da2[0]
                    di[1] = di[1] + da[1] + da2[1]
                    nagentFlag = 1
                    #print("E " )

                #else:
                    #print("else:p: EEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE " , i)    

            if(nagentFlag > 0):
                #add noise
                #di = di + randn(1,2)*dt;
                #print("di " , di)
                di = di + numpy.random.rand(1,2)*dt
                di = di/numpy.linalg.norm(di)
                #print("di: " , di , " " , di[0] , " " , di[0][1] , " " , " " , numpy.random.rand(1,2))
                deno = numpy.sqrt(pow(vels[i][1],2)+pow(vels[i][0],2))*numpy.sqrt(pow(di[0][1],2)+pow(di[0][0],2))
                dtheta = numpy.arccos(((di[0][1]*vels[i][1]) + (di[0][0]*vels[i][0]))/deno)
                # if(vels[i][1] == 0.0 and vels[i][0] == 0.0):
                #     vangle = 0.0
                # else:    
                #     vangle = ang_wrapPositive(numpy.arctan(vels[i][1]/vels[i][0]))
                #print("vels[i][1] " , vels[i][1] , " vels[i][0] " , vels[i][0])
                vangle = ang_wrapPositive(numpy.arctan(vels[i][1]/vels[i][0]))
                dangle = ang_wrapPositive(numpy.arctan(di[0][1]/di[0][0]));
                
                #print("dangle: " , numpy.arctan(di[0][1]/di[0][0]) , " " , dangle , " " , vangle)
                R = numpy.zeros((2,2))
                #R = [numpy.cos(2*pi - vangle) , -numpy.sin(2*pi - vangle); numpy.sin(2*pi - vangle), numpy.cos(2*pi - vangle)]
                R[0][0] = numpy.cos(2*pi - vangle)
                R[0][1] = -numpy.sin(2*pi - vangle)
                R[1][0] = numpy.sin(2*pi - vangle)
                R[1][1] = numpy.cos(2*pi - vangle)
                #print("di.T: " , di.T ,)
                #print( " R: " , R)
                dth = [0 ,0]
                dth[0] = R[0][0]*di[0][0] + R[0][1]*di[0][1]
                dth[1] = R[1][0]*di[0][0] + R[1][1]*di[0][1]
                #print("dth: " , dth , " " , dth[1] , "dangle: " , dangle)
                if(dtheta > omega*dt):
                    dangle1 = vangle + numpy.sign(dth[1])*omega*dt
                else:
                    #if(dangle)    
                    dangle1 = vangle + numpy.sign(dth[1])*dangle*dt
                #temppos = [0,0]
                #tempdi  = [0,0]
                tempdi[i][0] = numpy.cos(dangle1)
                tempdi[i][1] = numpy.sin(dangle1)
                temppos[i][0] = pos[i,0]+numpy.cos(dangle1)*s*dt 
                temppos[i][1] = pos[i,1]+ numpy.sin(dangle1)*s*dt
            else:
                tempdi[i] = vels[i]
                temppos[i][0] = pos[i,0] + vels[i,0]*s*dt 
                temppos[i][1] = pos[i,1] + vels[i,1]*s*dt

            #print("CHANGE__BBBBBBBBBBBBBBBBBBBBBBBBBBBB")        
            #print("i: " , i , "x: " , numpy.cos(dangle1)*s*dt , " y: " , numpy.sin(dangle1)*s*dt)        
            #print("CCCCCCCCCCCCCCCCCCCCCCCCCCCCC")
            #print(initpoints)

        else:
            temppos[i][0] = pos[i,0]
            temppos[i][1] = pos[i,1]
            tempdi[i] = vels[i]

    return temppos.T,tempdi                    

def main():
    rospy.init_node('Leader', anonymous=True)

    pub_marker = rospy.Publisher('leader', MarkerArray , queue_size=1)

    rate = rospy.Rate(10) # 10hz

    #N = 10
    nos = 0;
    Xstartpos = 0
    Ystartpos = 0
    threshold = 0

    #function [ X,Y,vels] = puneetsCouzins(X,Y,vels,Rrep,Rori,Ratt,s,dt,phi,omega)
    k = 15
    mu = 0;
    sigma = 20;
    x = Xstartpos + numpy.random.normal(mu,sigma,(1,N))
    y = Ystartpos + numpy.random.normal(mu,sigma,(1,N))    
    initpoints=numpy.concatenate((x,y),axis=0)
    vels=numpy.zeros((N,2))
    for i in range(N):
        vels[i][0] = numpy.random.rand(1,1)
        vels[i][1] = numpy.random.rand(1,1)
    # vels=numpy.zeros((N,2))
    # for i in range(N):
    #     vels[i][0] = 1
    # #vels[1] = 1
    # vels_temp = vels
    unitvec=numpy.zeros((2,N))
    #temppos = numpy.zeros((N,2))
    #tempdi = numpy.zeros((N,2))
    Rrep = 1
    Rori = Rrep + 20
    Ratt = Rori + 40
    s = 2
    s1 = 2
    dt = 0.2    
    phi = 200*(pi)/180
    omega = 1.5

    S = []

    nol = 0
    l1 = [0, 50]
    
    #leaderidx = numpy.asarray(l1).reshape(nol,2)

    nl = 15 
    leaderidx = numpy.zeros(nl)
    print(leaderidx , " " , leaderidx[4])
    for i in range(nl):
        print(i)
        leaderidx[i] = randint(0,N);


    leaderRrep = Rrep    
    leaderRori = Rori #The agents between the distance of 10(leaderRori) and 20(leaderRatt) get attracted towards them
    leaderRatt = Ratt

    destination = [0,80]
    destination = numpy.asarray(destination).reshape(1,2)

    destinationthreshold = 5.0
    rs = 15 

    M = MarkerArray()    
    
    shape = Marker.CUBE;
    for i in range(N+nol+1): #to include shepherd too
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
   
        
        if(i in leaderidx):
            marker.color.r = 1.0;
            marker.color.g = 0.0;


        if(i >= N and i<N+nol):
            marker.ns = "shep"
            marker.pose.position.x = leaderidx[i-N][0];
            marker.pose.position.y = leaderidx[i-N][1];
            marker.color.r = 1.0;
            marker.color.g = 0.0;

        if(i >= N+nol):
            marker.ns = "Point"
            marker.type = Marker.CYLINDER
            marker.pose.position.x = destination[0][0]
            marker.pose.position.y = destination[0][1]
            marker.color.b = 1.0;
            marker.color.r = 0.0;
            marker.scale.x = 2*destinationthreshold;
            marker.scale.y = 2*destinationthreshold;
            marker.scale.z = 0.5;   

        marker.lifetime = rospy.Duration();

        M.markers.append(marker)    

    
    count = 0;    
    while not rospy.is_shutdown():

        pub_marker.publish(M)

        print("initpoints before:")
        print(initpoints.T)
        print("before: WWWWWWWWWWWWWWWW")
        #print(vels)
        initpoints,vels =  SheepMovement(initpoints,vels,Rrep,Rori,Ratt,s1,dt,phi,omega,S,rs,leaderidx,leaderRrep,leaderRori,leaderRatt,destination,destinationthreshold)
        #print("initpoints after: " , numpy.arctan(0))
        print(initpoints.T)
        print("AFTER:ZZZZZZZZZZZZZZZZZZZZZZ")
        #print(vels.T)
        print("AAAAAAAAAAAAAAA")
        #print(vels_temp)
        

        for i in range(N + nol):
            if(i in leaderidx):
                M.markers[i].pose.position.x = initpoints[0][i] 
                M.markers[i].pose.position.y = initpoints[1][i]


            elif(i < N):
                M.markers[i].pose.position.x = initpoints[0][i] 
                M.markers[i].pose.position.y = initpoints[1][i]

            if(i >= N):
                M.markers[i].pose.position.x = leaderidx[i-N][0]
                M.markers[i].pose.position.y = leaderidx[i-N][1]
                print("leader: " , leaderidx[i-N][0] , " " , destination)              

        count = count + 1;
        if(count > 1000):
            print("COUNT MORE THAN 1000")
            break        


        rate.sleep()        




if __name__ == '__main__':
    #x = []
    #y = []
    try:
        main()
    except rospy.ROSInterruptException:
        pass
