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
        #print("SSSSSSSSSSSSSSSSSSSSSSSssssSSSs")    
        #print("j: " , j , " S: " , S , " Sh[j]: " , Shepherd[j])    
        D=numpy.linalg.norm(shepherdingpoints.T - S,axis=1)
        closeagents=numpy.where((D<=(rs)))                                    
        S=numpy.reshape(numpy.repeat(S,numpy.size(closeagents)),(2,numpy.size(closeagents)))
        N=numpy.shape(shepherdingpoints)
        NS,shepherdingpoints[:,closeagents[0]]=move(S,shepherdingpoints[:,closeagents[0]],rhos,-1)
    #return shepherdingpoints[0],shepherdingpoints[1]
    return shepherdingpoints

def initPhysico(initpoints,V1, deltaT,S,leaderidx,rs,dps,rhos,ra,destination,destinationthreshold,destination1,destination2,swarm_split,eaten1,eaten2,join,g1_reached3,g2_reached3):
    #D=numpy.linalg.norm(initpoints.T-agent1,axis=1)
    #temp = numpy.zeros(1,numpy.size(X))
    #pos = numpy.concatenate((X,Y),axis=0)
    #pos = initpoints.T
    #S = []

    #leaderidx = len(initpoints[0])
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

    # if(g2_reached3 == 1):
    #     S = numpy.asarray([S[0]]).reshape(1,2)
    # if(g1_reached3 == 1):   
    #     S = numpy.asarray([S[1]]).reshape(1,2)
        
    if(numpy.size(S) != 0):
        #print("Shperd present")
        idxshepherd = []
        for i in range(numpy.shape(S)[0]):
            d = numpy.linalg.norm(initpoints.T-S[i],axis=1)
            # print("dist: " , dist)
            # print("d: " , d)
            idx_rs = numpy.where(d <= rs)
            #print("idx_rs: " , idx_rs)
            if(numpy.size(idxshepherd) == 0):
                idxshepherd = idx_rs[0]
                idxshepherd = numpy.intersect1d(idxshepherd,idx[0])
                if(numpy.size(IDX) != 0):
                   idxshepherd = numpy.intersect1d(idxshepherd,IDX)
                #print("idxshepherd after first intersect1d")
                #print(idxshepherd)
            else:
                
                # if(numpy.size(IDX) != 0):
                #     idxshepherd = numpy.intersect1d(idxshepherd,IDX)
                # else:
                #print("idxshepherd: initially " , idxshepherd)
                idxshepherd = numpy.union1d(idxshepherd,idx_rs[0])
                idxshepherd = numpy.intersect1d(idxshepherd,idx[0])        
                #print("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAa")
                #print("idxshepherd: " , idxshepherd)
                #print("join:" , join)
                #print("idx: " , idx)
        #if(eaten1 == 1 and eaten2 == 1):        
        idxgravity = numpy.intersect1d(numpy.setdiff1d(temp_ind,idxshepherd),idx)
        # else:
        #     idxgravity = numpy.intersect1d(numpy.setdiff1d(temp_ind,idxshepherd),IDX)    
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
            #rows2 = numpy.where(d!=R) # why are we exclusing agents lying at a dist R ?? maybe so that they can move when they come at R
            rows3 = numpy.where(d > 0)
            rows = numpy.intersect1d(rows1[0],rows3[0])
            #rows = numpy.intersect1d(rows2[0],rows)
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
                scalar[k] = Fmax*pow(d[k],-3) #making force inverse to distance
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
    leaderidx = []

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

    nos = 2;
    Xstartpos = 0
    Ystartpos = 0
    threshold = 0
    N = 40 # 1 is nos 
    #function [ X,Y,vels] = puneetsCouzins(X,Y,vels,Rrep,Rori,Ratt,s,dt,phi,omega)
    k = 15
    # x=Xstartpos+numpy.random.rand(1,N)*k - (k/2) #swarm initialization
    # y=Ystartpos+numpy.random.rand(1,N)*k - (k/2)  #gives random values from [0,1)
    mu = 0;
    sigma = 2

    # destination1x = [-61., -55., -69., -67.]
    # destination1y = [-32.,  -2., -10., -32.]
    # destination2x = [67.,   4.,  69.,   5.]
    # destination2y = [52.,  59.,  35.,  48.]
    # destinationx = [37.,  58.,  36.,  37.]
    # destinationy = [18.,  28., -47.,  18.]
    destination1x = [-63., -42., -11., -11., -86., -30.,  -5.,  -8., -19., -17., -92.,-73., -14., -60., -97., -63., -67., -85., -31., -51., -76., -61.,-46., -80., -22.]

    destination1y = [-91., -66., -75.,  -5., -70., -46.,  92., -70., -66.,  -2., -66.,-62., -62., -89., -26., -95., -66., -28., -18., -42.,  -6., -98.,-55., -67., -91.]

    destination2x = [ 23.,   0.,  46.,  45.,  79.,   2.,  24.,  98.,  92.,  96.,  97.,63.,  73.,  76.,  55.,  72.,  25.,  40.,  58.,  90.,  23.,  95.,57.,  88.,  59.]

    destination2y =[ 27.,  98.,  75.,  95.,  85.,  53.,  12.,   4.,  94.,  53.,  16.,80.,  42.,   0.,  19.,  46.,  74.,  36.,  55.,  47.,  89.,  92.,52.,  39.,  27.]

    destinationx =[ 60., -82.,  14.,  81., -23.,  98.,  51.,  70.,  65., -51., -37.,38.,  39.,  31.,  30.,  36.,   0.,   9.,  96.,  52.,  66.,  43.,-84.,  34.,  95.]

    destinationy  = [ 96.,   1.,  28., -90.,  35.,  85., -24.,  74., -29.,  88.,  62.,-98.,  87.,  87.,  87., -32.,   7.,  89.,  86.,  91.,  55.,  53.,99.,  69.,  70.]

    
    MC = len(destination1x) 
    #MC = 25
    start_time = numpy.zeros((1,MC))
    stop_time = numpy.zeros((1,MC))
    split_time = numpy.zeros((1,MC))
    diff = numpy.zeros((1,MC))
    dest1_arr = numpy.zeros((2,MC))
    dest2_arr = numpy.zeros((2,MC))
    dest_arr = numpy.zeros((2,MC))
    split_diff = numpy.zeros((1,MC))
    not_complete = numpy.zeros((1,MC))
    agent_sep = numpy.zeros((1,MC))


# ('dest1: ', array([[-19., -24., -26., -43.,-61., -55., -69., -67.],
#                            [-40.,  -8., -18., -17.,-32.,  -2., -10., -32.]]))
# ('dest2: ', array([[ 14.,  66.,  61.,  15.,67.,   4.,  69.,   5.],
#                           [ 47.,  30.,  41.,   7.,52.,  59.,  35.,  48.]]))
# ('dest: ', array([[ 39.,   6., -54.,  59., 37.,  58.,  36.,  37.],
#                         [ 22.,  67.,  66.,  39.,18.,  28., -47.,  18.]]))


    for mc1 in range(MC):
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

        S1 = [0.0,-20.0]
        S2 = [0.0,20.0]
        #Sh = numpy.asarray(S1).reshape(1,2)
        Sh = numpy.asarray([S1,S2]).reshape(nos,2)

        dps = 1.0
        ra = 10.0
        rhos = 1.0/2.0 #strength of repulsion
        deltaT = 0.2
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
        # destination1 = [-45.0,25.0]
        # destination2 = [45.0,25.0]    
        # destination = [0.0,50.0]

        # A = 100
        # dx1 = numpy.random.randint(A,size = 1)
        # dy1 = numpy.random.randint(A,size = 1)
        # dx2 = numpy.random.randint(A,size = 1)
        # dy2 = numpy.random.randint(A,size = 1)
        # dx2 = dx2*(-1)
        # dy2 = dy2*(-1)

        # while(abs(dx2 - dx1) < 20):
        #     print("x2 in while: " , dx2)
        #     dx2 = numpy.random.randint(A,size = 1)
        # while(abs(dy2 - dy1) < 20):
        #     print("dy2 in while: " , dy2)
        #     dy2 = numpy.random.randint(A,size = 1)            
        # dx3 = numpy.random.randint(2*A,size = 1)
        # dy3 = numpy.random.randint(2*A,size = 1)
        # dx3 = dx3 - A; dy3 = dy3 - A;
        # while(abs(dx3 - dx1) < 25 or abs(dx3 - dx2) < 25):
        #     print("x3 in while: " , dx3)
        #     dx3 = numpy.random.randint(A,size = 1)
        # while(abs(dy3 - dy1) < 25 or abs(dy3 - dy2) < 25):
        #     print("dy3 in while: " , dy3)
        #     dy3 = numpy.random.randint(A,size = 1)            
        
        
        # if(dx1 > dx2):
        #     destination2 = [dx1[0],dy1[0]]
        #     destination1 = [dx2[0],dy2[0]]
        # else:
        #     destination1 = [dx1[0],dy1[0]]
        #     destination2 = [dx2[0],dy2[0]]                      
        
        # destination=[dx3[0],dy3[0]] #destination

        
        # dest1_arr[0][mc1] = destination1[0]
        # dest1_arr[1][mc1] = destination1[1]

        # dest2_arr[0][mc1] = destination2[0]
        # dest2_arr[1][mc1] = destination2[1]
        
        # dest_arr[0][mc1] = dx3[0]
        # dest_arr[1][mc1] = dy3[0]
        destination = numpy.asarray([destinationx[mc1],destinationy[mc1]]).reshape(1,2)
        destination1 = numpy.asarray([destination1x[mc1],destination1y[mc1]]).reshape(1,2)
        destination2 = numpy.asarray([destination2x[mc1],destination2y[mc1]]).reshape(1,2)    

        nod = 3;
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
                else:
                    marker.pose.position.x = destination[0][0]
                    marker.pose.position.y = destination[0][1]
                    marker.scale.x = 2*(destinationthreshold+5);
                    marker.scale.y = 2*(destinationthreshold+5);
                    marker.color.g = 1.0;                                                            
                marker.color.b = 1.0;
                marker.color.r = 0.0;

                marker.scale.z = 0.5;   

            M.markers.append(marker)    

        eaten1 = 0 ; eaten2 = 0; eaten3 = 0; eating3 = 0;
        near_dest1 = [] ; near_dest2 = []    
        swarm_split = 0;
        entered_once = 0;
        reached1 = 0; reached2 = 0;
        #sp = 0.001
        sp = 0.10
        sp1 = sp
        size_dec= 0.2
        join = 0;
        g1_reached3 = 0; g2_reached3 = 0;
        #start_time2 = rospy.get_rostime()
        count =  0;
        start_time[0][mc1] = rospy.get_rostime().to_sec()
        separated_agents = []
        while not rospy.is_shutdown():

            pub_marker.publish(M)

            initpoints,V1 = initPhysico(initpoints,V1, deltaT,Sh,leaderidx,rs,dps,rhos,ra,destination,destinationthreshold,destination1,destination2,
                swarm_split,eaten1,eaten2,join,g2_reached3,g1_reached3)
            print("yolo")
            print("eaten1: " , eaten1 , " eaten2: " , eaten2 , " reached1 ", reached1, "reached2: " , reached2)
            print("swarm_split: " , swarm_split , " join " , join)
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
                else:
                    #leaderidx = []
                    M.markers[i].pose.position.x = Sh[i-N][0]
                    M.markers[i].pose.position.y = Sh[i-N][1]   

            if(swarm_split == 1):
                #Sh = []
                if(entered_once == 0):
                    ind_l1 = numpy.random.choice(numpy.size(grp1),1)
                    ind_l2 = numpy.random.choice(numpy.size(grp2),1)
                    #leaderidx = [grp1[0][ind_l1[0]],grp2[0][ind_l2[[0]]]]
                    #M.markers[leaderidx[0]].color.r = 1.0
                    #M.markers[leaderidx[1]].color.r = 1.0
                    entered_once = 1;
                #if()    
                for k in range(numpy.size(initpoints[0])):
                    DD1 = numpy.linalg.norm(initpoints.T - initpoints[:,k].T,axis = 1)
                    far_from1 = numpy.where(DD1 < 10)
                    if(numpy.size(far_from1) < N*0.1):
                        separated_agents = far_from1
                        grp1 = numpy.setdiff1d(grp1,separated_agents)
                        grp2 = numpy.setdiff1d(grp2,separated_agents)
                            
                # DD1 = numpy.linalg.norm(initpoints.T - Sh[0],axis = 1)
                # DD2 = numpy.linalg.norm(initpoints.T - Sh[1],axis = 1)
                # far_from1 = numpy.where(DD1 > 3*rs)
                # far_from2 = numpy.where(DD2 > 3*rs)
                # separated_agents = numpy.intersect1d(far_from1[0],far_from2[0])
                # grp1 = numpy.setdiff1d(grp1,separated_agents)
                # grp2 = numpy.setdiff1d(grp2,separated_agents)
                #print("grp1: " , grp1)
                # grp1 = numpy.intersect1d(grp1[0],grp1[0])
                # grp2 = numpy.intersect1d(grp2[0],grp2[0])
                if(numpy.size(separated_agents) > 0):
                    agent_sep[0][mc1] = numpy.size(separated_agents)
                #print("grp1: " , grp1 , " grp1[0] " , grp1[0])
                    print("separated_agents: " , separated_agents)
                #grp1[0] = new_grp1
                #grp2[0] = new_grp2

                gcm1 = [numpy.average(initpoints[0,grp1]),numpy.average(initpoints[1,grp1])]
                gcm2 = [numpy.average(initpoints[0,grp2]),numpy.average(initpoints[1,grp2])]
                
                # print("grp1: " , grp1)
                # print("leader chosen: " , grp1[0][ind_l1[0]])    
                
                #sp = 0.001
                r = rs
                dr = 2;
                if(eaten1 == 0 and reached1 == 0):
                    #S0,a = moveSheperd(initpoints[:,leaderidx[0]],copy.deepcopy(destination1),sp,1)
                    #pos1 is the point on the line of destination1 and gcm1
                    m1 = numpy.arctan2(destination1[0][1] - gcm1[1],destination1[0][0] - gcm1[0])
                    pos1 = [0.0,0.0]
                    pos1[0] = gcm1[0] - (r-dr)*numpy.cos(m1)
                    pos1[1] = gcm1[1] - (r-dr)*numpy.sin(m1)
                    print("pos1: " , pos1)
                    S0,a = moveSheperd(Sh[0],pos1,sp,1)
                elif(reached1 == 1 and (eaten1 == 0 or eaten2 == 0)):
                    m1 = numpy.arctan2(destination[0][1] - gcm1[1],destination[0][0] - gcm1[0])
                    pos1 = [0.0,0.0]
                    pos1[0] = gcm1[0] - 2*r*numpy.cos(m1)
                    pos1[1] = gcm1[1] - 2*r*numpy.sin(m1)                    
                    S0,a = moveSheperd(Sh[0],pos1,sp,1)
                    n1 = numpy.linalg.norm(initpoints.T - S0)
                    if(numpy.size(n1) > 0):
                        S0,a = moveSheperd(S0,gcm1,sp/2,-1) #numpy.asarray(gcm1).reshape(1,2)                   
                    print("pos1: " , pos1)                        
                elif((eaten1 == 1 and eaten2 == 1) and reached1 == 1 and g1_reached3 == 0):
                    join = 1;
                    #S0,a = moveSheperd(initpoints[:,leaderidx[0]],copy.deepcopy(destination),sp,1)   
                    m1 = numpy.arctan2(destination[0][1] - gcm1[1],destination[0][0] - gcm1[0])
                    #sp1 = 0.004
                    pos1 = [0.0,0.0]                    
                    pos1[0] = gcm1[0] - (r-dr)*numpy.cos(m1)
                    pos1[1] = gcm1[1] - (r-dr)*numpy.sin(m1)                    
                    S0,a = moveSheperd(Sh[0],pos1,sp1,1)
                    print("pos1: " , pos1)
                elif(g1_reached3 == 1):
                    pos1 = [0,-100];
                    S0,a = moveSheperd(Sh[0],pos1,sp1,1)        
                
                if(eaten2 == 0 and reached2 == 0):
                    #S1,a = moveSheperd(initpoints[:,leaderidx[1]],copy.deepcopy(destination2),sp,1)
                    m2 = numpy.arctan2(destination2[0][1] - gcm2[1],destination2[0][0] - gcm2[0])
                    pos2 = [0.0,0.0]
                    pos2[0] = gcm2[0] - (r-dr)*numpy.cos(m2)
                    pos2[1] = gcm2[1] - (r-dr)*numpy.sin(m2)                    
                    #print("pos2: " , pos2)
                    S1,a = moveSheperd(Sh[1],pos2,sp,1)
                elif(reached2 == 1 and (eaten1 == 0 or eaten2 == 0)):
                    m2 = numpy.arctan2(destination[0][1] - gcm2[1],destination[0][0] - gcm2[0])
                    pos2 = [0.0,0.0]
                    pos2[0] = gcm2[0] - 2*r*numpy.cos(m2)
                    pos2[1] = gcm2[1] - 2*r*numpy.sin(m2)                    
                    S1,a = moveSheperd(Sh[1],pos2,sp,1)
                    n2 = numpy.linalg.norm(initpoints.T - S1)
                    if(numpy.size(n2) > 0):
                        S1,a = moveSheperd(S1,gcm2,sp/2,-1)
                    #print("pos1: " , pos1)                                            
                elif(eaten1 == 1 and eaten2 == 1 and reached2 == 1 and g2_reached3 == 0):
                    #S1,a = moveSheperd(initpoints[:,leaderidx[1]],copy.deepcopy(destination),sp,1)  
                    #sp1 = 0.004 
                    join = 1;
                    m2 = numpy.arctan2(destination[0][1] - gcm2[1],destination[0][0] - gcm2[0])
                    pos2 = [0.0,0.0]
                    pos2[0] = gcm2[0] - (r-dr)*numpy.cos(m2)
                    pos2[1] = gcm2[1] - (r-dr)*numpy.sin(m2)
                    print("pos2: " , pos2)                    
                    S1,a = moveSheperd(Sh[1],pos2,sp1,1)
                elif(g2_reached3 == 1):
                    pos2 = destination;
                    pos2 = [0,100]
                    S1,a = moveSheperd(Sh[1],pos2,sp1,1)                   

                Sh[0][0] = S0[0][0]
                Sh[0][1] = S0[0][1]
                Sh[1][0] = S1[0][0]
                Sh[1][1] = S1[0][1]                     
                # initpoints[0,leaderidx[0]] = S0[0][0]
                # initpoints[1,leaderidx[0]] = S0[0][1]    
                # initpoints[0,leaderidx[1]] = S1[0][0]
                # initpoints[1,leaderidx[1]] = S1[0][1] 
                # print("S0[0] " , S0[0])                
                # print("S1[1] " , S1[0])
                # print("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@")                                   
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
                    split_time[0][mc1] = rospy.get_rostime().to_sec()
                    grp1 = numpy.where(initpoints[0,:] < 0)
                    #grp1 = numpy.intersect1d(grp1[0],grp1[0])
                    
                    grp2 = numpy.where(initpoints[0,:] > 0)
                    #grp2 = numpy.intersect1d(grp2[0],grp2[0])

            if(swarm_split == 0):
                #sp = 0.002
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
            # if(numpy.size(near_shep1) > 0 or  numpy.size(near_shep2)):
            #     start_time = rospy.get_rostime().to_sec()


            #Start eating
            #if(swarm_split == 1 and eaten1 == 0 and numpy.size(near_dest1) >= 0.9*numpy.size(grp1)):  
            #size_dec = 0.005      
            if(swarm_split == 1 and eaten1 == 0):
                M.markers[N+nos].scale.x = M.markers[N+nos].scale.x - size_dec/(numpy.size(grp1))*(numpy.size(near_dest1))         
                M.markers[N+nos].scale.y = M.markers[N+nos].scale.y - size_dec/(numpy.size(grp1))*(numpy.size(near_dest1))
                if(M.markers[N+nos].scale.y < 0.2):
                    eaten1 = 1;

            if(entered_once ==1 and numpy.size(near_dest1) >= 0.9*numpy.size(grp1)):
                reached1 = 1;


            if(swarm_split == 1 and eaten2 == 0):
                M.markers[N+nos+1].scale.x = M.markers[N+nos+1].scale.x - size_dec/(numpy.size(grp2))*(numpy.size(near_dest2)) #*(numpy.size(near_dest2))         
                M.markers[N+nos+1].scale.y = M.markers[N+nos+1].scale.y - size_dec/(numpy.size(grp2))*(numpy.size(near_dest2))          #*(numpy.size(near_dest2))
                if(M.markers[N+nos+1].scale.y < 0.2):
                    eaten2 = 1;

            if(entered_once == 1 and numpy.size(near_dest2) >= 0.9*numpy.size(grp2)):
                reached2 = 1;        

                
            dist1 = numpy.linalg.norm(initpoints.T - destination1,axis = 1)            
            near_dest1 = numpy.where(dist1 <= destinationthreshold)

            dist2 = numpy.linalg.norm(initpoints.T - destination2,axis = 1)            
            near_dest2 = numpy.where(dist2 <= destinationthreshold)

            dist = numpy.linalg.norm(initpoints.T - destination,axis = 1)            
            near_dest = numpy.where(dist <= (destinationthreshold + 5.0))
            if(numpy.size(near_dest) >= 0.9*(N)):
                eating3 = 1; #START EATING3
            
            if(eaten1 == 1 and eaten2 == 1):
                g1 = numpy.intersect1d(grp1,near_dest[0])    
                if(numpy.size(g1) >= 0.9*numpy.size(grp1)):
                    g1_reached3 = 1;

                g2 = numpy.intersect1d(grp2,near_dest[0])    
                if(numpy.size(g2) >= 0.9*numpy.size(grp2)):
                    g2_reached3 = 1;
            
            if(eating3 == 1 and eaten3 == 0  and eaten1 == 1 and eaten2 == 1): #and numpy.size(near_dest) > 0
                #START EATING : 
                M.markers[N+nos+2].scale.x = M.markers[N+nos+2].scale.x - size_dec/N*(numpy.size(near_dest))         
                M.markers[N+nos+2].scale.y = M.markers[N+nos+2].scale.y - size_dec/N*(numpy.size(near_dest))                
                if(M.markers[N+nos+2].scale.x < 0.1):
                    eaten3 = 1;
            # if(numpy.size(near_dest) >= 0.9*(N)):
            #     stop_time = rospy.get_rostime()
            #     print("time taken:" ,stop_time - start_time)
            #     print("STOPPPPPPPPPP")
            #     break;     

            count = count + 1;
            print("COUNT : " , count)
            if(count > 10000):
                not_complete[0][mc1] = 1;
                break;
            if(eaten3 == 1):
                print("one complete");                
                break;                                   


            rate.sleep()            

        #stop_time2 = rospy.get_rostime()`
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
    print("++++++++++++++++++++++++++++++++++++++")
    print("FINAL SPLIT DIFF" , split_diff)
    print("++++++++++++++++++++++++++++++++++++++")
    print("not_complete: " , not_complete)
    print("++++++++++++++++++++++++++++++++++++++")
    print("separated_agents: " , agent_sep)
    

if __name__ == '__main__':
    #x = []
    #y = []
    try:
        main()
    except rospy.ROSInterruptException:
        pass
