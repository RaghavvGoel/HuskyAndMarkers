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

def moveSheperd( point1,point2,intensity,direction): 
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
        D=numpy.linalg.norm(shepherdingpoints.T - S,axis=1)
        closeagents=numpy.where((D<(rs)))                                    
        S=numpy.reshape(numpy.repeat(S,numpy.size(closeagents)),(2,numpy.size(closeagents)))
        N=numpy.shape(shepherdingpoints)
        NS,shepherdingpoints[:,closeagents[0]]=move(S,shepherdingpoints[:,closeagents[0]],rhos,-1)
    #return shepherdingpoints[0],shepherdingpoints[1]
    return shepherdingpoints

def initPhysico(initpoints,V1, deltaT,S,leaderidx,rs,dps,rhos,ra,destination,destinationthreshold,destination1,destination2,swarm_split,join):
    #D=numpy.linalg.norm(initpoints.T-agent1,axis=1)
    #temp = numpy.zeros(1,numpy.size(X))
    #pos = numpy.concatenate((X,Y),axis=0)
    #pos = initpoints.T
    #S = []

    #leaderidx = len(initpoints[0])
    # if(swarm_split == 1):
    #     DIST1 = numpy.linalg.norm(initpoints.T - (destination1),axis=1)    
    #     idx1 = numpy.where(DIST1 > destinationthreshold)

    #     DIST2 = numpy.linalg.norm(initpoints.T - (destination2),axis=1)    
    #     idx2 = numpy.where(DIST2 > destinationthreshold)            

    #     #idx = numpy.intersect1d(idx1[0],idx2[0])

    # #else:
    # DIST = numpy.linalg.norm(initpoints.T - numpy.asarray(destination).reshape(1,2),axis=1)    
    # idx = numpy.where(DIST > destinationthreshold)

    if(swarm_split == 1 and join == 0):
        DIST1 = numpy.linalg.norm(initpoints.T - (destination1),axis=1)    
        idx1 = numpy.where(DIST1 > destinationthreshold)

        DIST2 = numpy.linalg.norm(initpoints.T - (destination2),axis=1)    
        idx2 = numpy.where(DIST2 > destinationthreshold)      

        IDX = numpy.intersect1d(idx1[0],idx2[0])      
    else:
        IDX = []    
        #idx = numpy.intersect1d(idx1[0],idx2[0])

    #else:
    #if(eaten1 == 1 and eaten2 == 1):
    
    DIST = numpy.linalg.norm(initpoints.T - numpy.asarray(destination).reshape(1,2),axis=1)    
    if(join == 0):    
        idx = numpy.where(DIST >= 0.0)
    else:
        idx = numpy.where(DIST > 15.0)    
    temp_initpoints = copy.deepcopy(initpoints)

    temp_ind = []
    for i in range(len(initpoints[0])):
        temp_ind.append(i)

    if(numpy.size(S) != 0):
        #print("Shperd present")
        idxshepherd = []
        for i in range(numpy.shape(S)[0]):
            d = numpy.linalg.norm(initpoints.T-S[i],axis=1)
            # print("dist: " , dist)
            # print("d: " , d)
            idx_rs = numpy.where(d <= rs)
            print("idx_rs: " , idx_rs)
            if(numpy.size(idxshepherd) == 0):
                idxshepherd = idx_rs[0]
                print("idxshepherd after first intersect1d")
                print(idxshepherd)
            else:
                idxshepherd = numpy.union1d(idxshepherd,idx_rs[0])
                idxshepherd = numpy.intersect1d(idxshepherd,idx[0])
                print("idxshepherd: " , idxshepherd)
        idxgravity = numpy.intersect1d(numpy.setdiff1d(temp_ind,idxshepherd),idx)    
        # if(swarm_split == 1):
        #     idx = numpy.intersect1d(idx1[0],idx2[0])
        #     idxgravity = numpy.intersect1d(numpy.setdiff1d(temp_ind,idxshepherd),idx)                
    else:
        idxshepherd = []
        idxgravity = numpy.intersect1d(numpy.setdiff1d(temp_ind,leaderidx),idx)
    #idxgravity = numpy.intersect1d(temp_ind,idx)
        #idxshepherd = []        
    
    Xshepherd = [] ; Yshepherd = [];
    if(numpy.size(idxshepherd) != 0):
        #print("Xshepherd assigned")
        Xshepherd = initpoints[0][idxshepherd[0]] ; Yshepherd = initpoints[1][idxshepherd[0]]

    if(numpy.size(leaderidx) != 0):
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

    scale = 5.0        
    m = 1.0; # mass of agent
    p = 2.0; # degree to the distance between the two agents
    R = 10.0/scale;  #Range of repulsive forces (in metres)
    C = 50.0/scale; # range of forces applicable (in metres)
    cf = 0.2; # coefficient of friction for self-stabilisation, restricting the maximum velocity
    Fmax = 2; # Maximum force that can be experienced by each agent
    #Gravitational constant specific to the hexagonal formation  
    #G = Fmax * R^p * (2.0 - 1.5^(1-p))^(p/(1-p));
    G = Fmax*(pow(R,p))*pow(2.0 - pow(1.5,1-p),p/(1-p))

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
            rows2 = numpy.where(d!=R) # why are we exclusing agents lying at a dist R ?? maybe so that they can move when they come at R
            rows3 = numpy.where(d > 0)
            rows = numpy.intersect1d(rows1[0],rows3[0])
            rows = numpy.intersect1d(rows2[0],rows)
            dist = dist[rows,:]
            #print("d: " , d)
            d = d[rows]
            repRows = numpy.where(d<R)
            #print("d[repRows]: BEFORE " , d)
            d[repRows[0]] = -d[repRows[0]] # for repulsive force, negating
            #print("d[repRows]: AFTER " , d)
            #scalar = pow(G*m*m*d,-(p+1)) #assuming p = 2, otherwise sign for repulsion would create problems
            scalar = numpy.zeros((numpy.shape(d)[0],1))
            for k in range(numpy.shape(d)[0]):
                #scalar[k] = pow(G*m*m*d[k],-(p+1)) # = G*d[k])^(-3) = 1/(-6.25*D[k])^3 
                scalar[k] = pow(d[k],-3) #making force inverse to distance
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

    if(numpy.size(leaderidx) != 0):
        #print("LLLLLLLLLLLLLLLLLLLLLLLLL")        
        Xgravity = initpoints[0,idxgravity]
        Ygravity = initpoints[1,idxgravity]
        V = V1[idxgravity,:]
        #D = [Xgravity.T,Ygravity.T]
        D = numpy.zeros((numpy.size(Xgravity),2))
        for k in range(numpy.size(Xgravity)):
            D[k][0] = Xgravity[k]
            D[k][1] = Ygravity[k]        
        if(numpy.size(leaderidx) != 0):
            leader = numpy.zeros((numpy.size(Xleader),2))
            for k in range(numpy.size(Xleader)):
                leader[k][0] = Xleader[k]
                leader[k][1] = Yleader[k]
            #dist = leader - D[]    
        #leader = [Xleader.T , Yleader.T]
        F = []
        Fx = []
        Fy = []
        for i in range(numpy.shape(D)[0]):
            dist = leader - D[i]
            d = numpy.linalg.norm(leader - D[i,:], axis = 1);
            rows = numpy.where(d <= ra) 
            # no repulsion from the leader implies overlapping
            #we meed to add repuslion here as leader not present in inter agent part which has repulsion
            
            #rows2 = numpy.where(d >= 2.0)
            #rows3 = numpy.where(d <= ra)
            #rows = numpy.intersect1d(rows2[0],rows3[0])
            #print("rows: " , rows)
            dist = dist[rows[0],:]
            d = d[rows[0]]  
            rowsRep = numpy.where(d < 2.0) # taking 2m for repulsion
            d[rowsRep[0]] = -d[rowsRep[0]] 
            #print("rows leader pull " , rows)
            #print("d[rows]: " , d)          
            #scalar = pow(G*m*m*d,-(p+1))
            scalar = numpy.zeros((numpy.shape(d)[0],1))
            for k in range(numpy.shape(d)[0]):
                #scalar[k] = pow(G*m*m*d[k],-(p+1))
                scalar[k] = pow(d[k],-3) #making force inverse to distance
                #dist[k] = dist[k]*scalar[k]
            # print("scalar: " , scalar , " size " , numpy.size(scalar) , " " , numpy.shape(scalar))
            # print("dist shape before changing " , numpy.shape(dist))
            dist = dist*scalar;
            # print("dist*scalar= " , dist)
            # print("size dist: " , numpy.size(dist), " " )
            # print("dist shape: " , numpy.shape(dist), " ")
            #dist=normr(dist.*scalar);% getting projections of scalar along points
            for j in range(numpy.size(rows)):
                dist[j][0] = dist[j][0]/numpy.linalg.norm(dist[j]);
            #F[i,:] = numpy.sum(dist)
            #F.append(numpy.sum(dist))
            Fx.append(numpy.sum(dist[:,0]))
            Fy.append(numpy.sum(dist[:,1]))
            #print("F(leader): "  , F) 
        #deltaV = (F * deltaT);
        deltaV = numpy.zeros((numpy.size(Fx),2))
        #print("Fx: " , Fx)
        #print("Fy: " , Fy)
        deltaV[:,0] = Fx
        deltaV[:,1] = Fy
        deltaV = deltaV*deltaT #as deltaV = F*dT
        # for i in range(numpy.size(F)):
        #     print("kkkkkkkkkkkkkkkkkkk")
        #     deltaV[i] = F[i]*deltaT
        #     print("deltaV: " , deltaV[i])
        #V = normr((V + deltaV)*(1.0 - cf));

        V = ((V+deltaV)*(1.0 - cf))
        #print("V(leader: " , V)
        # for j in range(numpy.shape(V)[0]):
        #     V[j,0] = V[j,0]/numpy.linalg.norm(V[j,:])
        #     V[j,1] = V[j,1]/numpy.linalg.norm(V[j,:])
        deltaD = V*deltaT
        #print("deltaD: " , deltaD)
        D = D + deltaD
        #print("D leader wala")
        #print(D)
        initpoints[0,idxgravity] = D[:,0] 
        initpoints[1,idxgravity] = D[:,1]
        V1[idxgravity,:] = V

    if(numpy.shape(S)[0] != 0):
        #Xshepherd , Yshepherd = sheperdingEffect(pos,S,rs,rhos)
        #print("Shepherd")
        shepherdingpoints = sheperdingEffect(initpoints[:,idxshepherd],S,rs,rhos)
        initpoints[0,idxshepherd] = shepherdingpoints[0]
        initpoints[1,idxshepherd] = shepherdingpoints[1]
        initpoints,V = stepspertimestep(initpoints,temp_initpoints,dps*deltaT)
        V = V.T
        # print("V1 shape: " , numpy.shape(V1) , " " , numpy.shape(V))
        # print(" idxshepherd ",idxshepherd)
        V1[idxshepherd,0] = V[idxshepherd,0]
        V1[idxshepherd,1] = V[idxshepherd,1]        
    
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
    net = [];

    destination1x = [-19., -24., -26., -43.,-61., -55., -69., -67.]
    destination1y = [-40.,  -8., -18., -17.,-32.,  -2., -10., -32.]
    destination2x = [ 14.,  66.,  61.,  15.,67.,   4.,  69.,   5.]
    destination2y = [ 47.,  30.,  41.,   7.,52.,  59.,  35.,  48.]
    destinationx = [ 39.,   6., -54.,  59., 37.,  58.,  36.,  37.]
    destinationy = [ 22.,  67.,  66.,  39.,18.,  28., -47.,  18.]
    
    MC = len(destination1x) 
    start_time = numpy.zeros((1,MC))
    stop_time = numpy.zeros((1,MC))
    split_time = numpy.zeros((1,MC))
    diff = numpy.zeros((1,MC))
    dest1_arr = numpy.zeros((2,MC))
    dest2_arr = numpy.zeros((2,MC))
    dest_arr = numpy.zeros((2,MC))
    split_diff = numpy.zeros((1,MC))


    for mc1 in range(MC):
        nos = 2;
        Xstartpos = 0
        Ystartpos = 0
        threshold = 0
        N = 10 # 1 is nos 
        #function [ X,Y,vels] = puneetsCouzins(X,Y,vels,Rrep,Rori,Ratt,s,dt,phi,omega)
        k = 15
        # x=Xstartpos+numpy.random.rand(1,N)*k - (k/2) #swarm initialization
        # y=Ystartpos+numpy.random.rand(1,N)*k - (k/2)  #gives random values from [0,1)
        mu = 0;
        sigma = 2
        nod = 3
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
        s = 1.5
        dt = 0.2    
        phi = 360*(pi)/180
        omega = 1.5

        S1 = [0.0,-25.0]
        S2 = [0.0,25.0]
        #Sh = numpy.asarray(S1).reshape(1,2)
        Sh = numpy.asarray([S1,S2]).reshape(nos,2)

        dps = 2.0
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
        leaderidx = numpy.random.randint(N,size=1)

        #destination = [0,50]
        

        # destination = numpy.asarray(destination).reshape(1,2)
        # destination1 = numpy.asarray(destination1).reshape(1,2)
        # destination2 = numpy.asarray(destination2).reshape(1,2)    
        destination = numpy.asarray([destinationx[mc1],destinationy[mc1]]).reshape(1,2)
        destination1 = numpy.asarray([destination1x[mc1],destination1y[mc1]]).reshape(1,2)
        destination2 = numpy.asarray([destination2x[mc1],destination2y[mc1]]).reshape(1,2)    

        M = MarkerArray()    
        l = 0;
        shape = Marker.CUBE;
        for i in range(N+nos+nod): #to include shepherd too
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
                # marker.pose.position.x = x[0][leaderidx[0]]
                # marker.pose.position.y = y[0][leaderidx[0]]
                marker.pose.position.z = 2.0
                marker.color.r = 1.0;
                marker.color.g = 0.0;

            if(i >= N+nos):
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
                    marker.scale.x = 2*(destinationthreshold+5.0);
                    marker.scale.y = 2*(destinationthreshold+5.0);                                                        
                marker.color.b = 1.0;
                marker.color.g = 0.0;

                marker.scale.z = 0.5;   

            M.markers.append(marker)    

        eaten1 = 0 ; eaten2 = 0; eating3 = 0; eaten3 = 0;
        near_dest1 = [] ; near_dest2 = []    
        swarm_split = 0;
        entered_once = 0;
        sp = 0.2
        spl = 0.01
        entered_once2 = 0;
        count = 0;
        size_dec = 0.2
        join = 0;
        start_time[0][mc1] = rospy.get_rostime().to_sec()
        while not rospy.is_shutdown():

            pub_marker.publish(M)

            initpoints,V1 = initPhysico(initpoints,V1, deltaT,Sh,leaderidx,rs,dps,rhos,ra,destination,destinationthreshold,destination1,destination2,
                swarm_split,join)
            print("yolo")
            print("eaten1:" , eaten1 , " eaten2:" , eaten2 )
            print("swarm_split:" , swarm_split , " join: " , join)
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
                
                elif(swarm_split == 0):
                    leaderidx = []
                    M.markers[i].pose.position.x = Sh[i-N][0]
                    M.markers[i].pose.position.y = Sh[i-N][1]                    
                elif(swarm_split == 1):
                    Sh = []
                    if(entered_once == 0):
                        ind_l1 = numpy.random.choice(numpy.size(grp1),1)
                        ind_l2 = numpy.random.choice(numpy.size(grp2),1)
                        leaderidx = [grp1[0][ind_l1[0]],grp2[0][ind_l2[[0]]]]
                        M.markers[leaderidx[0]].color.r = 1.0
                        M.markers[leaderidx[1]].color.r = 1.0
                        entered_once = 1;
                    # print("grp1: " , grp1)
                    # print("leader chosen: " , grp1[0][ind_l1[0]])    
                    #sp = 0.00125
                    if(eaten1 == 0):
                        S0,a = moveSheperd(initpoints[:,leaderidx[0]],copy.deepcopy(destination1),spl,1)
                    elif(eaten1 == 1 and eaten2 == 1):
                        S0,a = moveSheperd(initpoints[:,leaderidx[0]],copy.deepcopy(destination),spl,1)   
                    if(eaten2 == 0):
                        S1,a = moveSheperd(initpoints[:,leaderidx[1]],copy.deepcopy(destination2),spl,1)
                    elif(eaten1 == 1 and eaten2 == 1):
                        S1,a = moveSheperd(initpoints[:,leaderidx[1]],copy.deepcopy(destination),spl,1)   
                    initpoints[0,leaderidx[0]] = S0[0][0]
                    initpoints[1,leaderidx[0]] = S0[0][1]    
                    initpoints[0,leaderidx[1]] = S1[0][0]
                    initpoints[1,leaderidx[1]] = S1[0][1] 
                    #print("S0[0] " , S0[0])
                    #print("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@")
                    #print("S1[1] " , S1[0])                                   
                    #leaderidx[1] = numpy.random.randint(grp2,size=1)
                    # if(i == N):
                    # M.markers[i].pose.position.x = initpoints[0][leaderidx[0]]
                    # M.markers[i].pose.position.y = initpoints[1][leaderidx[0]]
                    
                    #interesting that only 0.005 here is making it go so fast
                    #initpoints[1][leaderidx[0]] = initpoints[1][leaderidx[0]] + 0.005

            # print("Sh: " , Sh , " " , numpy.shape(Sh))
            # print("swarm_split: " , swarm_split)


            if(swarm_split == 0):
                if(abs(Sh[0][1] - Sh[1][1]) < rs):
                    swarm_split = 1;
                    grp1 = numpy.where(initpoints[0,:] < 0)
                    grp2 = numpy.where(initpoints[0,:] > 0)

            if(swarm_split == 0):
                #sp = 0.0025
                gcm = [numpy.average(initpoints[0,:]),numpy.average(initpoints[1,:])]
                S0,a = moveSheperd(Sh[0],gcm,sp,1)
                S1,a = moveSheperd(Sh[1],gcm,sp,1)
                #print("S0: ", S0 , numpy.shape(S0) , " " , numpy.size(S0)
                Sh[0][0] = S0[0][0]
                Sh[0][1] = S0[0][1]
                Sh[1][0] = S1[0][0]
                Sh[1][1] = S1[0][1]            
            
            dist_shep1 = numpy.linalg.norm(initpoints.T - S0,axis = 1)
            dist_shep2 = numpy.linalg.norm(initpoints.T - S1,axis = 1)
            near_shep1 = numpy.where(dist_shep1 <= rs)
            near_shep2 = numpy.where(dist_shep2 <= rs)
            if(numpy.size(near_shep1) > 0 or  numpy.size(near_shep2) > 0 and entered_once2 == 0):
                #start_time = rospy.get_rostime()
                entered_once2 = 1;


            #Start eating
            #if(swarm_split == 1 and eaten1 == 0 and numpy.size(near_dest1) >= 0.9*numpy.size(grp1)):
            #sp = 0.001;    
            if(swarm_split == 1 and eaten1 == 0):
                M.markers[N+nos].scale.x = M.markers[N+nos].scale.x - size_dec/(numpy.size(grp1))*(numpy.size(near_dest1)) 
                M.markers[N+nos].scale.y = M.markers[N+nos].scale.y - size_dec/(numpy.size(grp1))*(numpy.size(near_dest1))
                if(M.markers[N+nos].scale.y < 0.2):
                    eaten1 = 1;

            if(swarm_split == 1 and eaten2 == 0):
                M.markers[N+nos+1].scale.x = M.markers[N+nos+1].scale.x - size_dec/(numpy.size(grp2))*(numpy.size(near_dest2)) 
                M.markers[N+nos+1].scale.y = M.markers[N+nos+1].scale.y - size_dec/(numpy.size(grp2))*(numpy.size(near_dest2))
                if(M.markers[N+nos+1].scale.y < 0.2):
                    eaten2 = 1;

                
            dist1 = numpy.linalg.norm(initpoints.T - destination1,axis = 1)            
            near_dest1 = numpy.where(dist1 <= destinationthreshold)

            dist2 = numpy.linalg.norm(initpoints.T - destination2,axis = 1)            
            near_dest2 = numpy.where(dist2 <= destinationthreshold)

            dist = numpy.linalg.norm(initpoints.T - destination,axis = 1)            
            near_dest = numpy.where(dist <= destinationthreshold/3.0)
            if(numpy.size(near_dest) >= 0.9*(N)):
                eating3 = 1;

            if(eating3 == 1):
                M.markers[N+nos+2].scale.x = M.markers[N+nos+2].scale.x - size_dec/N*(numpy.size(near_dest))
                M.markers[N+nos+2].scale.y = M.markers[N+nos+2].scale.y - size_dec/N*(numpy.size(near_dest))
                if(M.markers[N+nos+2].scale.y < 0.2):
                    eaten3 = 1;                                   

            #count = count + 1;    
                        
            if(eaten3 == 1):
                break;

            rate.sleep()
                
        # stop_time2 = rospy.get_rostime().to_sec()
        # net.append(stop_time2 - start_time[0][mc1])
        #print("Whole: " , diff3 , " sec: " , diff3.to_sec())    
        #print("count: " , count)        
    # print("TIME: " , net)
    # print("dest1: " , )    
        print("MCMCMCMCMCMCMCMCMCMCMMMCMCM")
        print("mc1 : " ,mc1)    
        stop_time[0][mc1] = rospy.get_rostime().to_sec()
        split_diff[0][mc1] = split_time[0][mc1] - start_time[0][mc1]
        diff[0][mc1] = stop_time[0][mc1] - start_time[0][mc1]
        #print("diff: " , diff)
    print("++++++++++++++++++++++++++++++++++++++")
    #diff = stop_time - start_time
    #split_diff = stop_time - split_time
    print("FINAL diff: " , diff)
    print("FINAL SPLIT DIFF" , split_diff)
    print("Final Destinations: " )    

if __name__ == '__main__':
    #x = []
    #y = []
    try:
        main()
    except rospy.ROSInterruptException:
        pass
