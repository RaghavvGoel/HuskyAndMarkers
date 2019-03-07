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


#CAN CHnge this to moveshepehrd, choose as convinient
def move( point1,point2,intensity,direction ):    
    if numpy.shape(point1)[0]!=0 and numpy.shape(point2)[0]!=0:
        difference=numpy.asarray(point1)-numpy.asarray(point2)
        #difference=point1-point2
        norm=numpy.linalg.norm(difference,axis=0)
        l=numpy.shape(point2)
        if numpy.ndim(point1)>1:
            for i in range(l[1]):
                if norm[i]==0:
                    unitvec=[0,0]
                else:
                    unitvec=[difference[0][i]/norm[i],difference[1][i]/norm[i]]
                point1[0,i]=point1[0,i]-direction*(intensity)*unitvec[0]
                point2[0,i]=point2[0,i]+direction*(intensity)*unitvec[0]
                point1[1,i]=point1[1,i]-direction*(intensity)*unitvec[1]
                point2[1,i]=point2[1,i]+direction*(intensity)*unitvec[1]
        else:
            if norm==0:
                unitvec=[0,0]
            else:
                unitvec=[difference[0]/norm,difference[1]/norm]
            point1[0]=point1[0]-direction*(intensity)*unitvec[0]
            point2[0]=point2[0]+direction*(intensity)*unitvec[0]
            point1[1]=point1[1]-direction*(intensity)*unitvec[1]
            point2[1]=point2[1]+direction*(intensity)*unitvec[1]
    return point1,point2


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


# def sheperdingEffect(initpoints,Sh,rs,rhos):
#     pos = numpy.concatenate((X,Y),axis=0)
#     if(numpy.size(X) > 0):
#         for j in range(numpy.size(Sh)):
#             S = [Sh[j,0], Sh[j,1]]
#             print("shapes: ",numpy.shape(pos.T) , " " , numpy.shape(S))
#             D = numpy.linalg.norm(initpoints.T - numpy.asarray(S).reshape(1,2) , axis =1)
#             idx = numpy.where(D < rs)
#             if(numpy.size(idx) != 0):
#                 point1 = [initpoints[0,idx].T , initpoints[1,idx].T]
#                 points2 = S
#                 for i in range(numpy.size(x[idx])):
#                     points1[i,:],a = move(points1[i,:],points2,rhos,-1)
#                 X[idx] = points1[:,0]
#                 Y[idx] = points1[:,1]
#         return X[idx], Y[idx]       

def sheperdingEffect(initpoints,Shepherd,rs,rhos): #the fear of the predator
    Shepherdsize=numpy.shape(Shepherd)
    shepherdingpoints=initpoints
    for j in range(Shepherdsize[0]):
        if numpy.size(Shepherd)==2:
            S=Shepherd
        else:
            S=Shepherd[j]
        D=numpy.linalg.norm(shepherdingpoints.T - S[j,:],axis=1)
        closeagents=numpy.where((D<(rs)))                                    
        S=numpy.reshape(numpy.repeat(S,numpy.size(closeagents)),(2,numpy.size(closeagents)))
        N=numpy.shape(shepherdingpoints)
        NS,shepherdingpoints[:,closeagents[0]]=move(S,shepherdingpoints[:,closeagents[0]],rhos,-1)
    #return shepherdingpoints[0],shepherdingpoints[1]
    return shepherdingpoints

def initPhysico(initpoints,V1, deltaT,leader_pos,leader_vels,rs,dps,rhos,ra,destination,destinationthreshold):
    #D=numpy.linalg.norm(initpoints.T-agent1,axis=1)
    #temp = numpy.zeros(1,numpy.size(X))
    #pos = numpy.concatenate((X,Y),axis=0)
    #pos = initpoints.T
    S = []

    leaderidx = len(initpoints[0])
    temp_initpoints = copy.deepcopy(initpoints)
    
    DIST = numpy.linalg.norm(initpoints.T - numpy.asarray(destination).reshape(1,2),axis=1)    
    idx = numpy.where(DIST > destinationthreshold)

    temp_ind = []
    for i in range(len(initpoints[0])):
        temp_ind.append(i)

    # if(numpy.size(S) != 0):
    #     print("Shperd present")
    #else:
        #idxgravity = numpy.intersect1d(numpy.setdiff1d(temp,leaderidx),idx)
    idxgravity = numpy.intersect1d(temp_ind,idx)
        #idxshepherd = []        
    
    #NO USE OF LEADERIDX AS WE HAVE EXTERNAL LEADERS AND NOT WITHIN THE SWARM
    if(numpy.size(leaderidx) != 0 and numpy.size(leader_pos) == 0):
        Xleader = initpoints[0,leaderidx];
        Yleader = initpoints[1,leaderidx];
    if(numpy.size(idxgravity) != 0):        
        #print("idxgravity[0]: " , idxgravity[0])
        Xgravity = initpoints[0,idxgravity];
        Ygravity = initpoints[1,idxgravity];
        V = V1[idxgravity,:]    
        #D = [Xgravity.T,Ygravity.T];
        #D = numpy.concatenate((Xgravity,Ygravity),axis=0)
    n = numpy.size(Xgravity)
    D = numpy.zeros((numpy.size(Xgravity),2))
    for k in range(numpy.size(Xgravity)):
        D[k][0] = Xgravity[k]
        D[k][1] = Ygravity[k]

    m = 1.0; # mass of agent
    p = 2.0; # degree to the distance between the two agents
    R = 10.0;  #Range of repulsive forces (in metres)
    C = 50; # range of forces applicable (in metres)
    cf = 0.2; # coefficient of friction for self-stabilisation, restricting the maximum velocity
    Fmax = 1; # Maximum force that can be experienced by each agent
    #Gravitational constant specific to the hexagonal formation  
    #G = Fmax * R^p * (2.0 - 1.5^(1-p))^(p/(1-p));
    G = Fmax*(pow(R,p))*(2.0 - pow(pow(1.5,1-p),p/(1-p)))

# %% Inter-Agent
    F = []
    Fx = []
    Fy = []
    if(numpy.size(idxgravity) != 0):
        #print("D: " , D , " " , D[0])
        for i in range(numpy.shape(D)[0]):
            #print("D_shape: " , numpy.shape(D))
            #print("i of D: " , i)
            dist = D - D[i] #vector distance(B-A) between points
            d = numpy.linalg.norm(D - D[i],axis = 1) #magnitude of distance(|B-A|) between points
            rows1 = numpy.where(d<=C)
            #rows2 = numpy.where(d!=R) # why are we exclusing agents lying at a dist R ?? maybe so that they can move when they come at R
            rows3 = numpy.where(d > 0)
            rows = numpy.intersect1d(rows1[0],rows3[0])
            #rows = numpy.intersect1d(rows2[0],rows)
            dist = dist[rows,:]
            #print("d: " , d)
            d = d[rows]
            repRows = numpy.where(d<R)
            #print("d[repRows]: BEFORE " , d)
            d[repRows] = -d[repRows] # for repulsive force, negating
            #print("d[repRows]: AFTER " , d)
            #scalar = pow(G*m*m*d,-(p+1)) #assuming p = 2, otherwise sign for repulsion would create problems
            scalar = numpy.zeros((numpy.shape(d)[0],1))
            for k in range(numpy.shape(d)[0]):
                #scalar[k] = pow(G*m*m*d[k],-(p+1)) # = G*d[k])^(-3) = 1/(-6.25*D[k])^3 
                scalar[k] = pow(d[k],-1) #making force inverse to distance
            # print("i: " , i)
            #print("dist: before " , dist)
            #print("scalar: " , scalar)    
            #NOT SCALING AS THE DELTAS COMING TOO SMALL
            dist = dist*scalar #getting projections of scalar along points                    
            Fx.append(numpy.sum(dist[:,0]))
            Fy.append(numpy.sum(dist[:,1]))
        deltaV = numpy.zeros((numpy.size(Fx),2))
        # print("Fx: " , Fx , " " , numpy.size(Fx))
        # print("Fx: " , Fy , " " , numpy.size(Fy))
        # for i in range(numpy.size(Fx)):    
        #     deltaV[i] = sqrt(pow(Fx[i],2)+pow(Fy[i],2)) * deltaT
        deltaV[:,0] = Fx
        deltaV[:,1] = Fy
        deltaV = deltaV*deltaT
        # print("deltaV: " , deltaV)
        # print("V Shape: " , numpy.shape(V))
        # print("deltaV Shape: " , numpy.shape(deltaV))
        V = ((V + deltaV)*(1.0 - cf))

        for i in range(numpy.shape(V)[0]):
            if(numpy.linalg.norm(V[i,:]) >= 1):
                V[i,0] = V[i,0]/numpy.linalg.norm(V[i,:])
                V[i,1] = V[i,1]/numpy.linalg.norm(V[i,:])       

        deltaD = V*deltaT
        # print("delta D: " , deltaD)
        D = D + deltaD
        initpoints[0][idxgravity] = D[:,0]
        initpoints[1][idxgravity] = D[:,1]
        V1[idxgravity,:] = V    

# %% With Leader (LEADER SEPARATE)
    #leader_pos = []
    #if(numpy.size(leaderidx) != 0):

    if(numpy.size(leader_pos) != 0):
        print("LLLLLLLLLLLLLLLLLLLLLLLLL")
        Xgravity = initpoints[0,idxgravity]
        Ygravity = initpoints[1,idxgravity]
        V = V1[idxgravity,:]
        #D = [Xgravity.T,Ygravity.T]
        D = numpy.zeros((numpy.size(Xgravity),2))
        for k in range(numpy.size(Xgravity)):
            D[k][0] = Xgravity[k]
            D[k][1] = Ygravity[k]        
        # if(numpy.size(leaderidx) != 0):
        #     leader = numpy.zeros((numpy.size(Xleader),2))
        #     for k in range(numpy.size(Xleader)):
        #         leader[k][0] = Xleader[k]
        #         leader[k][1] = Yleader[k]
            #dist = leader - D[]    
        #leader = [Xleader.T , Yleader.T]
        F = []
        for i in range(numpy.shape(D)[0]):
            dist = leader_pos-D[i]
            d = numpy.linalg.norm(leader_pos - D[i,:], axis = 1);
            rows = numpy.where(d <= ra)
            print("rows: " , rows)
            dist = dist[rows,:]
            d = d[rows]  
            #print("rows leader pull " , rows)
            print("d[rows]: " , d)          
            #scalar = pow(G*m*m*d,-(p+1))
            scalar = numpy.zeros((numpy.shape(d)[0],1))
            for k in range(numpy.shape(d)[0]):
                #scalar[k] = pow(G*m*m*d[k],-(p+1))
                scalar[k] = pow(d[k],-1) #making force inverse to distance
            dist = dist*scalar;
            print("dist*scalar= " , dist)
            #dist=normr(dist.*scalar);% getting projections of scalar along points
            for j in range(numpy.size(rows)):
                dist[j][0] = dist[j][0]/numpy.linalg.norm(dist[j]);
            #F[i,:] = numpy.sum(dist)
            F.append(numpy.sum(dist))
            print("F(leader): "  , F) 
        #deltaV = (F * deltaT);
        for i in range(numpy.size(F)):
            print("kkkkkkkkkkkkkkkkkkk")
            deltaV[i] = F[i]*deltaT
            print("deltaV: " , deltaV[i])
        #V = normr((V + deltaV)*(1.0 - cf));

        V = ((V+deltaV)*(1.0 - cf))
        print("V(leader: " , V)
        # for j in range(numpy.shape(V)[0]):
        #     V[j,0] = V[j,0]/numpy.linalg.norm(V[j,:])
        #     V[j,1] = V[j,1]/numpy.linalg.norm(V[j,:])
        deltaD = V*deltaT
        print("deltaD: " , deltaD)
        D = D + deltaD
        print("D leader wala")
        print(D)
        initpoints[0,idxgravity] = D[:,0] 
        initpoints[1,idxgravity] = D[:,1]
        V1[idxgravity,:] = V
    
    # print("Xshepherd: " , Xshepherd)
    # if(numpy.shape(S)[0] != 0 and numpy.size(Xshepherd) != 0):
    #     #Xshepherd , Yshepherd = sheperdingEffect(pos,S,rs,rhos)
    #     print("Shepherd")
    #     shepherdingpoints = sheperdingEffect(initpoints[:,idxshepherd],S,rs,rhos)
    #     initpoints[0,idxshepherd] = shepherdingpoints[0]
    #     initpoints[1,idxshepherd] = shepherdingpoints[1]
    #     initpoints,V = stepspertimestep(initpoints,temp_initpoints,dps*deltaT)
    #     V = V.T
    #     #print("V shape: " , numpy.shape(V1) , " " , numpy.shape(V))
    #     print(V[idxshepherd])
    #     V1[idxshepherd,0] = V[idxshepherd,0]
    #     V1[idxshepherd,1] = V[idxshepherd,1]

    return initpoints,V1    



def main():
    rospy.init_node('Physicometics', anonymous=True)

    pub_marker = rospy.Publisher('Physico', MarkerArray , queue_size=1)
    #sub_cursor = rospy.Subscriber('/initialpose' , PoseWithCovarianceStamped , get_pos)

    rate = rospy.Rate(10) # 10hz

    nos = 1;
    Xstartpos = 0
    Ystartpos = 0
    threshold = 0
    N = 2
    #function [ X,Y,vels] = puneetsCouzins(X,Y,vels,Rrep,Rori,Ratt,s,dt,phi,omega)
    k = 15
    # x=Xstartpos+numpy.random.rand(1,N)*k - (k/2) #swarm initialization
    # y=Ystartpos+numpy.random.rand(1,N)*k - (k/2)  #gives random values from [0,1)
    mu = 0;
    sigma = 2
    x = Xstartpos + numpy.random.normal(mu,sigma,(1,N))
    y = Ystartpos + numpy.random.normal(mu,sigma,(1,N))

    initpoints=numpy.concatenate((x,y),axis=0)
    V1=numpy.zeros((N,2))
    # for i in range(N):
    #     V1[i][0] = numpy.random.rand(1,1)
    #     V1[i][1] = numpy.random.rand(1,1)

    # for i in range(N):
    #     vels[i][0] = 1
    #vels[1] = 1
    vels_temp = V1
    unitvec=numpy.zeros((2,N))
    #temppos = numpy.zeros((N,2))
    #tempdi = numpy.zeros((N,2))
    global rviz_pos
    Rrep = 2
    Rori = Rrep + 2# was 5
    Ratt = Rrep + 10
    s = 1
    dt = 0.2    
    phi = 360*(pi)/180
    omega = 1.0

    dps = 1.0
    ra = 10.0
    rhos = 1.0 #strength of repulsion
    deltaT = 0.1
    rs = 15.0
    destinationthreshold = 10.0
    
    L1 = [0.0,-15]
    leader_pos = numpy.asarray(L1).reshape(1,2)
    leader_vels = numpy.zeros((1,2))
    leader_vels[0][0] = numpy.random.rand(1,1)
    leader_vels[0][1] = numpy.random.rand(1,1)

    #leader_pos = 
    leaderidx = []

    destination = [0,50]

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
            marker.pose.position.x = leader_pos[i-N][0];
            marker.pose.position.y = leader_pos[i-N][1];
            marker.color.r = 1.0;
            marker.color.g = 0.0;

        if(i >= N+nos):
            marker.ns = "Point"
            marker.type = Marker.CYLINDER
            marker.pose.position.x = destination[0]
            marker.pose.position.y = destination[1]
            marker.color.b = 1.0;
            marker.color.g = 0.0;
            marker.scale.x = 2*destinationthreshold;
            marker.scale.y = 2*destinationthreshold;
            marker.scale.z = 0.5;   

        M.markers.append(marker)    

    

    while not rospy.is_shutdown():

        pub_marker.publish(M)

        initpoints,V1 = initPhysico(initpoints,V1, deltaT,leader_pos,leader_vels,rs,dps,rhos,ra,destination,destinationthreshold)
        print("yolo")
        #print("x: " , x)
        #print("y: " , y)
        for i in range(N + nos):
            #if(i in leaderidx):
            #print("i in main: " , i)
            #print("x: " , x)
            if(i < N):
                #print("x[0][i]: " , x[i])
                M.markers[i].pose.position.x = initpoints[0][i] 
                M.markers[i].pose.position.y = initpoints[1][i]
            
            elif(i == N):
                #print("LLLLLLLLLLLLLLLLLLLLLLLLL")
                #print(leader_pos1)
                M.markers[i].pose.position.x = leader_pos[0][0]
                M.markers[i].pose.position.y = leader_pos[0][1]
                
            elif(i == N+1):    
                M.markers[i].pose.position.x = leader_pos[1][0]
                M.markers[i].pose.position.y = leader_pos[1][1]        

    rate.sleep()            

if __name__ == '__main__':
    #x = []
    #y = []
    try:
        main()
    except rospy.ROSInterruptException:
        pass
