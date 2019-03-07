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

pi =  22.0/7.0
N = 50
temppos = numpy.zeros((N,2))
tempdi = numpy.zeros((N,2))

rviz_pos = [10000 , 10000]

def get_pos(msg):
    global rviz_pos
    rviz_pos[0] = msg.pose.pose.position.x
    rviz_pos[1] = msg.pose.pose.position.y



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
    print("angle: " , angle)
    #for i in range(len(angle)):
    if(angle < 0):
        angle = angle + 2*pi;
    if(angle > 2*pi):
        angle = angle - 2*pi
    if(angle < -2*pi):
        angle = angle + 2*pi;
    return angle                

def SheepMovement(initpoints,vels,Rrep,Rori,Ratt,s,dt,phi,omega):
    pos= initpoints.T
    #print("pos:")
    #print(pos)
    dists = numpy.zeros(len(initpoints[0]))
    for i in range(len(initpoints[0])):
        #dists = 
        for j in range(len(initpoints[0])):
            #dists.append(numpy.sqrt(pow(pos[j,0]-pos[i,0],2) + pow(pos[j,1]-pos[i,1],2)));
            dists[j] = numpy.sqrt(pow(pos[j,0]-pos[i,0],2) + pow(pos[j,1]-pos[i,1],2));
        #replace ith agent distance
        #print("dists: ", dists)
        vi = numpy.where((dists == 0))
        dists[i] = 10000;
        v = numpy.where((dists <= Rrep))
        # v = []
        # for j in range(len(dists)):
        #     if(j < Rrep):
        #         v.append(j)
        print("v: " , v , " " , len(v[0]))
        di = [0,0];
        nagentFlag = 0;
        dr = [0,0]; drFlag = 0;
    
        for j in range(len(v[0])):
            #make sure the neighbor is inside the visible region
            #vels is the direction
            #if(abs(numpy.arccos(numpy.dot(vels[v[j],1] - vels[i,1] , vels[v[j],0] - vels[i,0]))) <= phi ):
            #print("dot: ", numpy.dot(vels[v[j],:],  vels[i,:]) )
            print("j: " , j , " " , " " , " " , v[0][j] ," " ,(vels[v[0][j]][1] - vels[i][1]), " " , vels[v[0][j]][0] - vels[i][0])
            #print("div: " , (vels[v[0][j]][1] - vels[i][1])/(vels[v[0][j]][0] - vels[i][0]))
            #print("vels: " , vels)
            #print("ANGLE: " ,numpy.arccos(vels[v[0][j]][1]*vels[i][1] + vels[v[0][j]][0]*vels[i][0]))
            deno = numpy.sqrt(pow(vels[v[0][j]][1],2)+pow(vels[v[0][j]][0],2))*numpy.sqrt(pow(vels[i][1],2)+pow(vels[i][0],2))
            if(abs(numpy.arccos(vels[v[0][j]][1]*vels[i][1] + vels[v[0][j]][0]*vels[i][0]/(deno))) <= phi):
            #if(abs(numpy.arccos((vels[v[j]][1] - vels[i][1])/(vels[v[j]][0] - vels[i][0]))) <= phi or (vels[v[j]][0] == vels[i][0])):
                #print("TTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT")
                #print("norm: " , pos[v[j],:] , " " , pos[i,:], " " , numpy.linalg.norm((pos[v[j],:]-pos[i,:]),axis = 0)) 
                dr = dr + (pos[v[0][j],:]-pos[i,:])/numpy.linalg.norm((pos[v[0][j],:]-pos[i,:]),axis = 0)
                drFlag = 1;     
        # % orientation - equation 2
        dists1 = numpy.where(dists>Rrep)
        dists2 = numpy.where(dists <= Rori)
        v = numpy.intersect1d(dists1 , dists2)
        print("v: " , v , len(v))
        zoo = [0 ,0]
        zooFlag = 0

        for j in range(len(v)):
            deno = numpy.sqrt(pow(vels[v[j]][1],2)+pow(vels[v[j]][0],2))*numpy.sqrt(pow(vels[i][1],2)+pow(vels[i][0],2))
            if(abs(numpy.arccos((vels[v[j]][1]*vels[i][1] + vels[v[j]][0]*vels[i][0]))) <= phi):
                zoo = zoo + vels[v[j]][:]
                zooFlag = 1


        dists1 = numpy.where(dists > Rori)
        dists2 = numpy.where(dists <= Ratt)
        v = numpy.intersect1d(dists1, dists2)
        #v = numpy.where(dists <= Ratt)
        print("vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv")
        print("v: " , v )
        #print("v[0] " , v[0] , " " , v[1])
        da = [0, 0];
        zoaFlag = 0;
        #v = v.reshape(1,numpy.size(v))
       # print("after reshape: " , v[0] , " " ,v[1])
        for j in range(numpy.size(v)):
            deno = numpy.sqrt(pow(vels[v[j]][1],2)+pow(vels[v[j]][0],2))*numpy.sqrt(pow(vels[i][1],2)+pow(vels[i][0],2))
            if(abs(numpy.arccos((vels[v[j]][1]*vels[i][1] + vels[v[j]][0]*vels[i][0]))) <= phi):
                da = da + (pos[v[j],:]-pos[i,:])/numpy.linalg.norm((pos[v[j],:]-pos[i,:]),axis = 0)
                print("da= " , (pos[v[j],:]-pos[i,:])/numpy.linalg.norm((pos[v[j],:]-pos[i,:]),axis = 0))
                print("2222da= " , (pos[v[j],:]-pos[i,:])/numpy.linalg.norm((pos[v[j],:]-pos[i,:])))
                zoaFlag = 1;

        # %% decision making
        if(drFlag == 1):
            di = -dr
            nagentFlag = 1;
        elif(zooFlag == 1 and zoaFlag == 1):
            di = 0.5*(zoo+da)
            nagentFlag = 1;
        elif(zooFlag == 1):
            di = zoo
            nagentFlag = 1
        elif(zoaFlag == 1):
            di = da
            nagentFlag = 1
        else:
            print("else:p: EEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE " , i)    

        if(nagentFlag > 0):
            #add noise
            #di = di + randn(1,2)*dt;
            di = di + numpy.random.rand(1,2)*dt
            A1 = numpy.linalg.norm((di),axis = 0)
            A2 = numpy.linalg.norm(di)
            A3 = di/A1
            A4 = di/A2
            print("QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ")
            print("A1: " , A1, " A2= " , A2 , " di=" , di - numpy.asarray([1,1]).reshape(1,2))
            print("A3= " , A3 , " A4= " , A4)
            di = di/numpy.linalg.norm(di)

            print("di: " , di , " " , di[0] , " " , di[0][1] , " " , " " , numpy.random.rand(1,2))
            deno = numpy.sqrt(pow(vels[i][1],2)+pow(vels[i][0],2))*numpy.sqrt(pow(di[0][1],2)+pow(di[0][0],2))
            dtheta = numpy.arccos(((di[0][1]*vels[i][1]) + (di[0][0]*vels[i][0]))/deno)
            # if(vels[i][1] == 0.0 and vels[i][0] == 0.0):
            #     vangle = 0.0
            # else:    
            #     vangle = ang_wrapPositive(numpy.arctan(vels[i][1]/vels[i][0]))
            print("vels[i][1] " , vels[i][1] , " vels[i][0] " , vels[i][0])
            vangle = ang_wrapPositive(numpy.arctan2(vels[i][1],vels[i][0]))
            dangle = ang_wrapPositive(numpy.arctan2(di[0][1],di[0][0]));
            
            print("dangle: " , numpy.arctan2(di[0][1],di[0][0]) , " " , dangle , " " , vangle)
            R = numpy.zeros((2,2))
            #R = [numpy.cos(2*pi - vangle) , -numpy.sin(2*pi - vangle); numpy.sin(2*pi - vangle), numpy.cos(2*pi - vangle)]
            R[0][0] = numpy.cos(2*pi - vangle)
            R[0][1] = -numpy.sin(2*pi - vangle)
            R[1][0] = numpy.sin(2*pi - vangle)
            R[1][1] = numpy.cos(2*pi - vangle)
            print("di.T: " , di.T ,)
            print( " R: " , R)
            dth = [0 ,0]
            dth[0] = R[0][0]*di[0][0] + R[0][1]*di[0][1]
            dth[1] = R[1][0]*di[0][0] + R[1][1]*di[0][1]
            print("dth: " , dth , " " , dth[1] , "dangle: " , dangle)
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

    #print("BBBBBBBBBBBBBBBBBBBBBBBBBBBB")        
    #print(temppos.T)        
    #print("CCCCCCCCCCCCCCCCCCCCCCCCCCCCC")
    #print(initpoints)
    return temppos.T,tempdi                    

def main():
    rospy.init_node('Leader', anonymous=True)

    pub_marker = rospy.Publisher('leader', MarkerArray , queue_size=1)
    sub_cursor = rospy.Subscriber('/initialpose' , PoseWithCovarianceStamped , get_pos)

    rate = rospy.Rate(10) # 10hz

    #N = 10
    nos = 0;
    Xstartpos = 0
    Ystartpos = 0
    threshold = 0

    #function [ X,Y,vels] = puneetsCouzins(X,Y,vels,Rrep,Rori,Ratt,s,dt,phi,omega)
    k = 15
    #x=Xstartpos+numpy.random.rand(1,N)*k - (k/2) #swarm initialization
    #y=Ystartpos+numpy.random.rand(1,N)*k - (k/2)  #gives random values from [0,1)
    mu = 0;
    sigma = 5;
    x = Xstartpos + numpy.random.normal(mu,sigma,(1,N))
    y = Ystartpos + numpy.random.normal(mu,sigma,(1,N))
    initpoints=numpy.concatenate((x,y),axis=0)
    vels=numpy.zeros((N,2))
    for i in range(N):
        vels[i][0] = numpy.random.rand(1,1)
        vels[i][1] = numpy.random.rand(1,1)

    # for i in range(N):
    #     vels[i][0] = 1
    #vels[1] = 1
    vels_temp = vels
    unitvec=numpy.zeros((2,N))
    #temppos = numpy.zeros((N,2))
    #tempdi = numpy.zeros((N,2))
    global rviz_pos
    Rrep = 2
    Rori = Rrep + 0# was 5
    Ratt = Rrep + 10
    s = 2
    dt = 0.2    
    phi = 360*(pi)/180
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

    

    while not rospy.is_shutdown():

        pub_marker.publish(M)

        print("initpoints before:")
        #print(initpoints)
        print("before: WWWWWWWWWWWWWWWW")
        #print(vels)
        initpoints,vels =  SheepMovement(initpoints,vels,Rrep,Rori,Ratt,s,dt,phi,omega)
        #print("initpoints after: " , numpy.arctan(0))
        #print(initpoints)
        print("AFTER:ZZZZZZZZZZZZZZZZZZZZZZ")
        #print(vels.T)
        print("AAAAAAAAAAAAAAA")
        #print(vels_temp)

        for i in range(N):
            M.markers[i].pose.position.x = initpoints[0][i] 
            M.markers[i].pose.position.y = initpoints[1][i] 


        #MOVE to CLICKED POINT
        #D =     


        rate.sleep()        




if __name__ == '__main__':
    #x = []
    #y = []
    try:
        main()
    except rospy.ROSInterruptException:
        pass
