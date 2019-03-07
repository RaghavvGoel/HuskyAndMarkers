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

def inertialEffect( initpoints,direction,h ):
    inertiapoints=numpy.zeros((2,numpy.size(direction)/2))
    print direction
    if numpy.size(direction)!=0:
        inertiapoints=initpoints+h*direction
    return inertiapoints

def inertialEffect( initpoints,direction,h ):
    inertiapoints=initpoints
    print direction
    if numpy.size(direction)!=0:
        for i in range(len(initpoints[0])):
            inertiapoints[0][i] = initpoints[0][i] + h*direction[0][i]
            inertiapoints[1][i] = initpoints[1][i] + h*direction[1][i]
    return inertiapoints    

def clusteringEffect(initpoints,n,c,rs,Shepherd,swarm_split): #the clustering of the sheep
    Shepherdsize=numpy.shape(Shepherd)
    clusterpoints=initpoints
    for j in range(Shepherdsize[0]):
        if numpy.size(Shepherd)==2:
            S=Shepherd
        else:
            S=Shepherd[j]
        #print("IIIIIIIIIIIIIIIIIIIIIIIII")
        #print(initpoints)
        #print(initpoints.T)    
        D=numpy.linalg.norm(initpoints.T-S,axis=1)
        #print("initpoints: " , initpoints[:,1:10])
        #print("init.T ", initpoints.T[1:10] ,)
        #print(" D in cluster: " , D[1:10]);
        #print("CCCCCCCCCCCCCCCCCCCCCCCCCCCCC")
        #print("")
        cl = [i for i in D if i < rs]
        #print("cl: ")
        #print(cl)
        #print("CLCLCLCCLCLCLCCLCLCCLCLCLLL")
        closeagents=numpy.where((D>(rs)))  # Check why this is happening
        closeagents_nearLeader = numpy.where((D<=(rs)))
        #print("closeagents: " , closeagents, " " , numpy.size(closeagents))
        if(numpy.size(closeagents_nearLeader) > 0):
            for i in range(numpy.size(closeagents)):
                #print("FFFFFFFFFFFFFFFFFFFFFFFFFFFFFF")
                agent1=initpoints[:,closeagents[0][i]] 
                D=numpy.linalg.norm(initpoints.T-agent1,axis=1)
                sortedD=numpy.argsort(D)
                orderedD = numpy.sort(D)
                #indices of all the nieghbours within 5m of an agent
                D_less_than_10 = numpy.where((orderedD < 7.5))
                #if(swarm_split != 1):
                n = len(D_less_than_10[0])
                #else:
                    #print("clustering: n: " ,n)    
                #sortedDindex=numpy.where((sortedD<=3))
                #if(len(D_less_than_10) < 5):
                    #n = len(D_less_than_10)
                #else:
                    #n = 5    
                #if(toggle == 0):
                #sortedDindex=numpy.where((sortedD<n)) #nearest neighbours which are inside a radius with center as the agent
                sortedDindex = sortedD[:n]
                sortedDindex = numpy.asarray(sortedDindex).reshape(1,n)
                #print("sortedD: " , sortedDindex )
                #print("orderedD: " , orderedD)
                #print("D_less_than_10: ", len(D_less_than_10) )
                #sortedDindex=numpy.where(())
                agent2=(initpoints[:,sortedDindex[0]]).mean(1)
                clusterpoints[:,closeagents[0][i]],agent2=move(agent1,agent2,c,1)
    #print("clusterpoints: " , clusterpoints.T[1:10])        
    return clusterpoints

def angularnoiseEffect( initpoints,e,p ):   # The noise function
    X=initpoints[0,:]
    Y=initpoints[1,:]
    l=numpy.shape(X)
    errorpoints=initpoints
    r=numpy.random.rand(1,l[0])
    for i in range(l[0]):
        if p>r[0][i]:
            X[i]=X[i]+e*numpy.random.randint(-1,1)
            Y[i]=Y[i]+e*numpy.random.randint(-1,1)
    errorpoints[0]=X
    errorpoints[1]=Y
    return errorpoints

def selfRepellingEffect( initialpoints,ra,rhoa):
    l=numpy.shape(initialpoints)
    repelledpoints=initialpoints
    for i in range(l[1]):
        D =numpy.linalg.norm(initialpoints.T-initialpoints[:,i],axis=1)
        closeagents=numpy.where((D>0) & (D<ra)) 
        if numpy.size(closeagents)!=0:
            agent1=numpy.reshape(numpy.repeat(initialpoints[:,i],numpy.size(closeagents)),(2,numpy.size(closeagents)))
            agent1,agent2=move(agent1,initialpoints[:,closeagents[0]],rhoa,-1)
            repelledpoints[:,i] = agent1.mean(1)
    return repelledpoints

def sheperdingEffect(initpoints,Shepherd,rs,rhos): #the fear of the predator
    Shepherdsize=numpy.shape(Shepherd)
    shepherdingpoints=initpoints
    for j in range(Shepherdsize[0]):
        if numpy.size(Shepherd)==2:
            S=Shepherd
        else:
            S=Shepherd[j]
        D=numpy.linalg.norm(shepherdingpoints.T-S,axis=1)
        closeagents=numpy.where((D<(rs)))                                    
        S=numpy.reshape(numpy.repeat(S,numpy.size(closeagents)),(2,numpy.size(closeagents)))
        N=numpy.shape(shepherdingpoints)
        NS,shepherdingpoints[:,closeagents[0]]=move(S,shepherdingpoints[:,closeagents[0]],rhos,1) #changed -1 to +1
    return shepherdingpoints

# def sheepmovement(initpoints,e,p,ra,rhoa,dps,Shepherd,direction,n,c,rs,rhos,h):
#     clusterpoints=clusteringEffect(initpoints,n,c,rs,Shepherd)
#     errorpoints=angularnoiseEffect(initpoints,e,p) # The effect of noise    <----working
#     repelledpoints=selfRepellingEffect(initpoints,ra,rhoa)         #<--working
#     fearpoints=sheperdingEffect(initpoints,Shepherd,rs,rhos)
#     inertiapoints=inertialEffect(initpoints[:],direction[:],h)
#     allpoints=repelledpoints+fearpoints+errorpoints+clusterpoints+inertiapoints
#     initpoints,direction=stepspertimestep(allpoints,initpoints,dps)   #<--working
    #return initpoints

def sheepmovements(initpoints,n,c,rs,Shepherd,ra,rhoa,rhos,unitvec,h,e,dps,mode,destination,p,swarm_split):
    #print("IN SHEEPMOVEMENT >>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
    #print("sheperd: " , Shepherd)
    copypoints = copy.deepcopy(initpoints)
    if(n != 1):
        if(swarm_split == 0): #swarm_split is the joined variable of the main
            initpoints = clusteringEffect(initpoints,n,c,rs,Shepherd,swarm_split)
            print("clustering effect")
    idx1 = []
    X = initpoints[0,:]
    Y = initpoints[1,:]
    for i in range(len(X)):
        dist = sqrt(pow(initpoints[0][i] - destination[0],2) + pow(initpoints[1][i] - destination[1],2))
        if(dist >= 10):
            idx1.append(i)
    initpoints1 = sheperdingEffect(initpoints,Shepherd,rs,rhos)
    for i in range(len(initpoints[0])):
        if(i not in idx1):
            initpoints1[0][i] = initpoints[0][i]
            initpoints1[1][i] = initpoints[1][i]        

    #print("initpoints " , initpoints )
    #print("inipoints1 " , initpoints1)        
    initpoints = abs(initpoints - initpoints1)
    initpoints[0] = initpoints[0] + initpoints[1]
    idx = []
    for i in range(len(initpoints[0])):
        if(initpoints[0][i] != 0):
            idx.append(i)
    #idx = idx.intersect(idx1)
    idx.sort()
    idx1.sort()
    #print("idx: " , idx , " idx1: " , idx1 , " " , len(idx) , " " , len(idx1))
    j = 0; k = 0;
    temp =[]
    while(j < len(idx) and k < len(idx1)):
        if(idx[j] == idx1[k]):
            temp.append(j)
            j = j+1; k=k+1;
        elif(idx[j] > idx1[k]):
            k = k + 1
        else:
            j = j + 1
    #print("before intertial initpoints")
    #print(initpoints)        
    if(len(idx)!=0 and len(unitvec)!= 0):
        initpoints1[:,idx] = inertialEffect(initpoints1[:,idx],unitvec[:,idx],h)
    initpoints = initpoints1
    #print("AFTER IT")
    #print(initpoints)
    #r = numpy.random.randint(len(X))
    #for i in range()    
    initpoints = angularnoiseEffect(initpoints,e,p)

    initpoints[:,idx1] = selfRepellingEffect(initpoints[:,idx1],ra,rhos)
    idx = []
    for i in range(len(initpoints[0])):
        dist = sqrt(pow(initpoints[0][i] - destination[0],2) + pow(initpoints[1][i] - destination[1],2))
        if(dist >= 10):
            idx.append(i)
    if(len(idx) != 0):
        initpoints[:,idx],unitvec1 = stepspertimestep(initpoints[:,idx],copypoints[:,idx],dps)
        unitvec[:,idx] = unitvec1[:,:]
    #print("idx: " , idx)
    #print("idx1: " , idx1)
    if(len(idx) != 0):
        initpoints1 = initpoints[:,idx]
        R = numpy.random.rand(2,len(idx))
        initpoints1 = initpoints1 + R
        idx1 = []
        for i in range(len(initpoints1[0])):
            dist = sqrt(pow(initpoints1[0][i] - destination[0],2) + pow(initpoints1[1][i] - destination[1],2))
            if(dist > 10):
                idx1.append(i)
        if(len(idx1)!= 0 ):
            for i in idx1:
                initpoints1[:,i] = initpoints[:,idx[i]]
        initpoints[:,idx] = initpoints1            
    else:
        initpoints,unitvec = stepspertimestep(initpoints,copypoints,dps)    


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

def leadermovement(leader_pos,initpoints,destination,rs,rrep,step):
    temp_leader_pos = copy.deepcopy(leader_pos)
    D = numpy.linalg.norm(initpoints.T - leader_pos,axis = 1)
    idx_att = numpy.where(D <= 10.0)
    idx_rep = numpy.where(D <= 1.5)
    if(numpy.size(idx_rep) > 0): #let's change from 0 to sth  else ????
        #leader_pos , a = moveLeader(leader_pos,destination,1,1)
        print("idx_rep: " , idx_rep[0])
        leader_pos , a = moveLeader(leader_pos,destination,1.25,-1) #idx[0] implies the nearest agent to the leader
    else:
        leader_pos , a = moveLeader(leader_pos,destination,1,1)
    #leader_pos, a = stepspertimestepLeader(leader_pos,temp_leader_pos,1.5)     
    return leader_pos         

def moveLeader( point1,point2,intensity,direction): 
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

def stepspertimestepLeader(currentpoints,initialpoints,steps): # to limit the step taken per unit time
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
    rospy.init_node('sheep_sheperd', anonymous=True)
    e=0.3 # strength of error movement
    p=0.05 # probability of an error
    Xstartpos=0 #swarm initial position
    Ystartpos=-20
    h = 0.5 #strength of inertial force
    N = 4 # number of agents
    N1 = copy.deepcopy(N)
    ra=2.0  # distance for self repulsion
    mindisShep = 6
    n=numpy.ceil(0.1*N1)
    Pd = ra*log(N)
    Pc = ra
    threshold = 10.0
    dps=1 # speed of agents
    
    f = pow(N,0.5)
    rhoa=2.0# strength of self repulsion 
    rhos=1.0#strength of predatory repulsion 
    q = 1.8
    #I = []
    rhoash = 1.0
    #Shepherd=[-0,100]
    L1 = [-5,10.0]
    #S1 = [3,110]
    L2 = [5,10.0]
    #S2 = [60,90]
    L3 = [-60,90]
    L4 = [30,100]
    nol = 2 #Number of sheperd
    #Sh = numpy.asarray([S1,S2,S3,S4]).reshape(nos,2)
    #Sh = numpy.asarray([S1,S2,S3]).reshape(nos,2) #Sheperd Array    
    #leader_pos = numpy.asarray(L1).reshape(nol,2)
    leader_pos = numpy.asarray([L1,L2]).reshape(nol,2)

    leader_vels = numpy.zeros((1,2))
    leader_vels[0][0] = numpy.random.rand(1,1)
    leader_vels[0][1] = numpy.random.rand(1,1)

    rs = 12.5  #radius of influence
    rrep = 2 # radius of repulsion
    c=1.05 #strength of clustering
    nod = 2 #no. of destination
    destination=[0,70] #destination
    destination1 = [-25,50]
    destination2 = [25,50]
    k = 30 #Spreading the swarm
    x=Xstartpos+numpy.random.rand(1,N) #swarm initialization
    y=Ystartpos+numpy.random.rand(1,N)  #gives random values from [0,1)
    initpoints=numpy.concatenate((x,y),axis=0)
    direction=numpy.zeros((2,N))
    unitvec=numpy.zeros((2,N))

    mode1 = 2;
    mode2 = 2
    alpha = 1
    ang = 0
    mode = 2
    nin = 0 
    method = 1 
    predmasterslave = 3 
    ds = 1.5
    decrad = 0.9
    I = []
    I1 = []
    I2 = []
    
    destinationthreshold = 10.0
    pub_marker = rospy.Publisher('multi_sheep_sheperd', MarkerArray , queue_size=1)
    rate = rospy.Rate(10) # 10hz

    M = MarkerArray()    
    l = 0;
    shape = Marker.CUBE;
    for i in range(N+nol+2): #to include shepherd too
        marker = Marker()
        print("i " , i)
        marker.header.frame_id = "/multi_sheep_sheperd";
        marker.header.stamp = rospy.get_rostime();
        

        marker.ns = "mul_sheperd";
        
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

        if(i >= N and i<N+nol):
            marker.ns = "sheperd"
            marker.pose.position.x = leader_pos[i-N][0];
            marker.pose.position.y = leader_pos[i-N][1];
            marker.color.r = 1.0;
            marker.color.g = 0.0;

        if(i >= N+nol):
            marker.ns = "Point"
            marker.type = Marker.CYLINDER
            if(l == 0):
                marker.pose.position.x = destination1[0]
                marker.pose.position.y = destination1[1]
                l = 1;
            elif(l == 1):
                marker.pose.position.x = destination2[0]
                marker.pose.position.y = destination2[1]
                l = 2;                
            marker.color.b = 1.0;
            marker.color.g = 0.0;
            marker.scale.x = 2*destinationthreshold;
            marker.scale.y = 2*destinationthreshold;
            marker.scale.z = 0.5;   

        M.markers.append(marker)    
    count = 0    
    switch = 1; # change back to 1
    crossed_swarm = 0;
    enter_once = 0
    swarm_split = 0;        
    step = 1.5
    outthreshold = []
    for j in range(N):
        outthreshold.append(j)
    enter_once0 = 0; enter_once1 = 0;
    enter_once_grping = 0; done = 0;
    reached1 = 0 ; reached2 = 0; reached = 0;
    joined = 0;
    eaten1 = 0 ; eaten2 = 0;
    while not rospy.is_shutdown():

        pub_marker.publish(M)

        if(joined == 0):
            sheepmovements(initpoints,n,c,rs,leader_pos,ra,rhoa,rhos,unitvec,h,e,dps,mode,destination,p,swarm_split)
        #if(reached1 == 1 and reached2 == 1):
            print("joined: " , joined)
        #elif(joi)    
        print("size outthreshold: " , numpy.size(outthreshold))
        if(numpy.size(initpoints[0])*(0.1) > numpy.size(outthreshold) or joined == 1):
            #SWARM SHOULD EXPAND i.e leaders should move away
            #To expand the swarm we need the swarm outsie the destination threshold, so putting someother destination
            sheepmovements(initpoints,n,c,rs,[],ra,rhoa,rhos,unitvec,h,e,dps,mode,destination1,p,joined)
            joined = 1;
            print("joined: " , joined)

            #FEED ONN, then join


        if(enter_once0 == 1 and enter_once_grping == 0):
            leader0 = leadermovement(leader_pos[0],initpoints[:,outthreshold],destination1,rs,rrep,step)
            leader_pos[0] = leader0[0]
        elif(enter_once_grping == 1 and numpy.size(g1) > 0 and reached == 0):    
            leader0 = leadermovement(leader_pos[0],initpoints[:,g1[0]],destination1,rs,rrep,step)
            leader_pos[0] = leader0[0]
            print("SSSSSSSSSSSSSSSSSSSSSsSSSSSSSS")
            print("size g1: " , numpy.size(g1))
        elif(enter_once_grping == 1 and numpy.size(g1) <= 0):
            reached1 = 1    

        if(enter_once1 == 1 and enter_once_grping == 0):    
            leader1 = leadermovement(leader_pos[1],initpoints[:,outthreshold],destination2,rs,rrep,step)            
            leader_pos[1] = leader1[0]
        elif(enter_once_grping == 1 and numpy.size(g2) > 0 and reached == 0):    
            leader1 = leadermovement(leader_pos[1],initpoints[:,g2[0]],destination2,rs,rrep,step)            
            leader_pos[1] = leader1[0]
            print("size g2: " , numpy.size(g2))
        elif(enter_once_grping == 1 and numpy.size(g2) <= 0):
            reached2 = 1          

        if(reached == 1 and joined == 0):
            D0 = numpy.linalg.norm(initpoints.T - destination1,axis = 1)
            #NEW grp1
            grp1 = numpy.where(D0 <= destinationthreshold)

            D1 = numpy.linalg.norm(initpoints.T - destination2,axis = 1)
            #NEW grp2
            grp2 = numpy.where(D1 <= destinationthreshold)

        if(eaten1 == 1 and eaten2 == 1 and joined == 0):
            leader0 = leadermovement(leader_pos[0],initpoints[:,grp1[0]],destination,rs,rrep,step) 
            leader1 = leadermovement(leader_pos[1],initpoints[:,grp2[0]],destination,rs,rrep,step)
            leader_pos[0] = leader0[0]
            leader_pos[1] = leader1[0]
        
        # if(numpy.size(initpoints[0])*(0.1) > numpy.size(outthreshold) and joined == 1):
        #     print("moving away from dest")
        #     leader0,a = moveLeader(leader_pos[0],destination,1.5,-1)
        #     leader1,a = moveLeader(leader_pos[1],destination,1.5,-1)
        #     leader_pos[0] = leader0[0]
        #     leader_pos[1] = leader1[0]                  

        for i in range(N + nol):
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


        # if(enter_once == 0):        
        #     D = numpy.linalg.norm(initpoints.T - leader_pos,axis = 1)
        #     away_leader = numpy.where(D > rs)
        #     sort_D = numpy.argsort(D)        
        #     if(numpy.size(away_leader) > N/2.0):
        #         A1 = initpoints[:,sort_D[N/2 - 1]]
        #         print("A1: " , A1)
        #         leader_pos , a = moveLeader(leader_pos,A1,0.5,1)
        #     else:
        #         enter_once = 1;    
        
        if(reached1 == 1 and eaten1 == 0):
            #EATTT
            M.markers[N+nol].scale.x = M.markers[N+nol].scale.x - 0.25
            M.markers[N+nol].scale.y = M.markers[N+nol].scale.y - 0.25
            if(M.markers[N+nol].scale.x < 0.1):
                eaten1 = 1 

        if(reached2 == 1 and eaten2 == 0):
            #EATTT
            M.markers[N+nol+1].scale.x = M.markers[N+nol+1].scale.x - 0.25
            M.markers[N+nol+1].scale.y = M.markers[N+nol+1].scale.y - 0.25
            if(M.markers[N+nol+1].scale.x < 0.1):
                eaten2 = 1 


        if(enter_once0 == 0):        
            D0 = numpy.linalg.norm(initpoints.T - leader_pos[0],axis = 1)
            away_leader = numpy.where(D0 > rs)
            sort_D0 = numpy.argsort(D0)        
            if(numpy.size(away_leader) > N/2.0):
                A1 = initpoints[:,sort_D0[N/2 - 1]]
                #print("A1: " , A1)
                leader0 , a = moveLeader(leader_pos[0],A1,0.25,1)
                leader_pos[0] = leader0[0]
            else:
                enter_once0 = 1;            

        if(enter_once1 == 0):        
            D1 = numpy.linalg.norm(initpoints.T - leader_pos[1],axis = 1)
            away_leader = numpy.where(D1 > rs)
            sort_D1 = numpy.argsort(D1)        
            if(numpy.size(away_leader) > N/2.0):
                A1 = initpoints[:,sort_D1[N/2 - 1]]
                #print("A1: " , A1)
                leader1 , a = moveLeader(leader_pos[1],A1,0.25,1)
                leader_pos[1] = leader1[0]
            else:
                enter_once1 = 1;                            

        

        dist = []
        flag = 0;

        # for i in range(N):
        #     dist.append(sqrt(pow(initpoints[0][i] - destination[0],2) + pow(initpoints[1][i] - destination[1],2)))
        #     if(sqrt(pow(initpoints[0][i] - destination[0],2) + pow(initpoints[1][i] - destination[1],2)) <= threshold):
        #         flag = flag + 1
        #grp = []    
        if(abs(leader_pos[0][0] - leader_pos[1][0]) > 20 and enter_once_grping == 0):
            #FORM GROUPS
            dist_leader1 = numpy.linalg.norm(initpoints.T - leader_pos[0],axis = 1)
            dist_leader2 = numpy.linalg.norm(initpoints.T - leader_pos[1],axis = 1)
            grp1 = numpy.where(dist_leader1 < dist_leader2)
            grp2 = numpy.where(dist_leader1 >= dist_leader2)
            enter_once_grping = 1
            #if(dist_leader2 > dist_leader1):


        # dist_dest = numpy.linalg.norm(initpoints.T - destination,axis = 1)
        # idx_dest = numpy.where(dist_dest <= destinationthreshold)
        # if(numpy.size(idx_dest) > 0.9*N):
        #     print("RRREEEAACCHHEDD")
        #     break

        if(enter_once_grping == 1 and done == 0):    
            # dist_dest1 = numpy.linalg.norm(initpoints.T - destination1,axis = 1)
            # idx_dest1 = numpy.where(dist_dest1 <= destinationthreshold)
            # #if(numpy.size(idx_dest1) > 0.9*numpy.size(grp1)):
            #     #print("RRREEEAACCHHEDD")
            #     #break
            # dist_dest2 = numpy.linalg.norm(initpoints.T - destination2,axis = 1)
            # idx_dest2 = numpy.where(dist_dest2 <= destinationthreshold)

            done = 1
            #if(numpy.size(idx_dest2) > 0.9*numpy.size(grp2)):
                #print("RRREEEAACCHHEDD")
                #break                        
            #if(numpy.size(idx_dest1)+numpy.size(idx_dest2) > 0.9*(numpy.size(grp1)+numpy.size(grp2))):


        outthreshold = []        
        for i in range(N):
            dist.append(sqrt(pow(initpoints[0][i] - destination[0],2) + pow(initpoints[1][i] - destination[1],2)))
            if(sqrt(pow(initpoints[0][i] - destination[0],2) + pow(initpoints[1][i] - destination[1],2)) > threshold):
                #flag = flag + 1
                outthreshold.append(i)

        outthreshold1 = []        
        for i in range(N):
            dist.append(sqrt(pow(initpoints[0][i] - destination1[0],2) + pow(initpoints[1][i] - destination1[1],2)))
            if(sqrt(pow(initpoints[0][i] - destination1[0],2) + pow(initpoints[1][i] - destination1[1],2)) > destinationthreshold):
                #flag = flag + 1
                outthreshold1.append(i)

        outthreshold2 = []        
        for i in range(N):
            dist.append(sqrt(pow(initpoints[0][i] - destination2[0],2) + pow(initpoints[1][i] - destination2[1],2)))
            if(sqrt(pow(initpoints[0][i] - destination2[0],2) + pow(initpoints[1][i] - destination2[1],2)) > destinationthreshold):
                #flag = flag + 1
                outthreshold2.append(i)                                
                #print("Deleting " , i  ," " )
        if(done == 1):
            print("DDDDDDDDDDDDDDDDDDDDDDDDDDDDDD")
            g1 = numpy.intersect1d(grp1[0],outthreshold1)
            g1 = numpy.asarray(g1).reshape((1,numpy.size(g1)))
            print("g1: " , g1)
            #print("size grp1: " , numpy.size(grp1))

            g2 = numpy.intersect1d(grp2[0],outthreshold2)
            g2 = numpy.asarray(g2).reshape((1,numpy.size(g2))) 
            print("g2: " , g2)           

        dist_dest1 = numpy.linalg.norm(initpoints.T - destination1,axis = 1)
        idx_dest1 = numpy.where(dist_dest1 <= destinationthreshold)
        #if(numpy.size(idx_dest1) > 0.9*numpy.size(grp1)):
            #print("RRREEEAACCHHEDD")
            #break
        dist_dest2 = numpy.linalg.norm(initpoints.T - destination2,axis = 1)
        idx_dest2 = numpy.where(dist_dest2 <= destinationthreshold)

        if(done == 1 and numpy.size(idx_dest1)+numpy.size(idx_dest2) > 0.9*(numpy.size(grp1)+numpy.size(grp2))):
            print("RRREEEAACCHHEDD")
            reached = 1;
            #break

        if(count >= 1000):
            print("STOPPPPPPPPPPPPPP " , "count is: " , count)
            break        

        rate.sleep()
            
if __name__ == '__main__':
    #x = []
    #y = []
    try:
        main()
    except rospy.ROSInterruptException:
        pass
