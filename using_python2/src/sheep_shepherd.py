#!/usr/bin/env python
# license removed for brevity
import rospy
import array
from std_msgs.msg import String
from visualization_msgs.msg import *
import numpy
from math import *
import matplotlib.pyplot as plt
import copy

# def callback(msg):
#     message = "recieved  " + msg.data
#     print(message)
def singlesheperdMovement( S,X,Y,ra,Pd,Pc,destination,f):
    print("singlesheperdMovement in Sh : " , S , "  " , numpy.shape(X) , " |||||||||||||||||")
    D = numpy.zeros(numpy.shape(X)[0])
    idx = []
    S2 = copy.deepcopy(S)
    l = numpy.shape(X)
    for i in range(l[0]):
        D[i] = numpy.sqrt(numpy.square(X[i] - S[0]) + numpy.square(Y[i] - S[1]))
        if(D[i] < ra):
            idx.append(i)
    currentmode = 2 # 
    if(len(idx) == 0):
        Xgcm = sum(X)/numpy.shape(X)[0]
        Ygcm = sum(Y)/numpy.shape(Y)[0]
        Dgcm = []
        ind = []
        for i in range(l[0]):  # or len if X is a list and not an array     
            Dgcm.append(numpy.sqrt(numpy.square(X[i] - Xgcm) + numpy.square(Y[i] - Ygcm)))
            if(Dgcm[i] > f):
                ind.append(i)
        if(len(ind) == 0):   
            print("entering singleherdingPosition: " , S , " HHHHHHHHHHHHHHHHHH")         
            singleherdingPosition(S,Pc,X,Y,destination)
            print("AFTR HERDING: " , S , " hhhhhhhhhhhhhhhhh")
            currentmode = 1;
        else:
            pos = Dgcm.index(max(Dgcm))
            last = [X[pos],Y[pos]]
            print("BEFOR COLLECTING: " , S , " cccccccccccccccccc")
            singlecollectingPosition(S,last,Xgcm,Ygcm,Pd)
            S,s2 = stepspertimestepSheperd(S,S2,2) #dps = 1    
            print("AFTR COLLECTING: " , S , " CCCCCCCCCCCCCCCCCCCCCCCCCC")
            currentmode = 2     

        #S,s2 = stepspertimestepSheperd(S,S2,1) #dps = 1    
        #print("Destination: " , destination, " DDDDDDDDDDDDDDDDDDDDDDDDDDD")            
        print("S: " , S , "  s2 " , " ZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZ")
        #A = numpy.asarray(S)
        #A[0] = 1000
        #print("will S change : " , S , "  A: " , A) 
        #return S

def singleherdingPosition(S,Pc,X,Y,destination):
    print("In HERDING")
    S2 = copy.deepcopy(S)
    #S2 = [0,0]
    l = numpy.shape(X)
    #Dgcm = numpy.zeros(numpy.shape(X))
    Dgcm = []
    for i in range(l[0]):  # or len if X is a list and not an array     
        Dgcm.append(numpy.sqrt(numpy.square(X[i] - destination[0]) + numpy.square(Y[i] - destination[1])))
    #idx = numpy.argmax(Dgcm,axis = 0)
    pos = Dgcm.index(max(Dgcm))
    idx = []
    for i in range(len(Dgcm)):
        if(Dgcm[i] == Dgcm[pos]):
            idx.append(i)
    
    if(len(idx) != 0):
        # agent = numpy.zeros(2)
        # agent[0] = X[pos]
        # agent[1] = Y[pos]
        #agent = numpy.asarray([X[pos],Y[pos]])
        #print("agent " , agent)
        #S1 = S
        S[0] = X[pos]
        S[1] = Y[pos]
        print("Entering Move pt1: " , S , " pt2: " , destination )
        #difference=numpy.asarray(S)-numpy.asarray(destination)
        #norm=numpy.linalg.norm(difference,axis=0)
        #print("difference, " , difference , " norm " , norm)
        S,s2 = move(S,destination[:],Pc,-1) #s2 is useless

        #S,s2 = move(S,S,0,0)
        #S = S1
        print("S after move : " , S ,"  ", " SSSSSSSSSSSSSSSSSSSSS " ) 
    S,s2 = stepspertimestepSheperd(S,S2,2)   
    #numpy.asarray(S1)
    print("S after limiting its steps: " , S , " ssssssssssssssssssssssssssss " )
    #return S

def singlecollectingPosition(S,last,Xgcm,Ygcm,Pd):
    print("In COLLECTING")
    agent = numpy.zeros(2)
    agent[0] = Xgcm
    agent[1] = Ygcm
    last = numpy.asarray(last)
    print("last: ", last , " numpy.asarray([Xgcm,Ygcm])  " , agent)
    l,s2 = move(last,agent,Pd,-1) #s2 is useless
    S,s2 = move(S,l,1,1) 

def stepspertimestepSheperd(currentpoints,initialpoints,steps): # to limit the step taken per unit time
#steps: steps to be taken towards currentpoints from initialpoints
    #difference=numpy.asarray(currentpoints)-numpy.asarray(initialpoints)
    difference = [0,0]   
    difference[0] = currentpoints[0] - initialpoints[0]
    difference[1] = currentpoints[1] - initialpoints[1]
    norm=numpy.linalg.norm(difference,axis=0)
    l=numpy.shape(difference)
    #print("in stepper  L : " , l , " " , norm, " llolololooloolooololo " , currentpoints)
    direction=numpy.zeros((2,1))
    #for i in range(1):
    step=steps
    if norm<steps:
        step=norm
    if norm==0:
        unitvec=[0,0]
    else:
        unitvec=[difference[0]/norm,difference[1]/norm]
        direction[0]=unitvec[0]
        direction[1]=unitvec[1]
    currentpoints[0] = initialpoints[0]+unitvec[0]*step
    currentpoints[1] = initialpoints[1]+unitvec[1]*step
    print("currentpoints " , currentpoints, "  " , initialpoints," poppppopopopopo")
    return currentpoints,direction

def inertialEffect( initpoints,direction,h ):
    inertiapoints=numpy.zeros((2,numpy.size(direction)/2))
    print direction
    if numpy.size(direction)!=0:
        inertiapoints=initpoints+h*direction
    return inertiapoints    

def clusteringEffect(initpoints,n,c,rs,Shepherd): #the clustering of the sheep
    Shepherdsize=numpy.shape(Shepherd)
    clusterpoints=initpoints
    for j in range(Shepherdsize[0]):
        if numpy.size(Shepherd)==2:
            S=Shepherd
        else:
            S=Shepherd[j]
        D=numpy.linalg.norm(initpoints.T-S,axis=1)
        closeagents=numpy.where((D<(rs)))  # Check why this is happening
        for i in range(numpy.size(closeagents)):
            agent1=initpoints[:,closeagents[0][i]] 
            D=numpy.linalg.norm(initpoints.T-agent1,axis=1)
            sortedD=numpy.argsort(D)
            sortedDindex=numpy.where((sortedD<=n-1))
            #sortedDindex = sortedD[,n-1]
            agent2=(initpoints[:,sortedDindex[0]]).mean(1)
            clusterpoints[:,closeagents[0][i]],agent2=move(agent1,agent2,c,1)
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
        NS,shepherdingpoints[:,closeagents[0]]=move(S,shepherdingpoints[:,closeagents[0]],rhos,-1)
    return shepherdingpoints



def sheepmovement(initpoints,e,p,ra,rhoa,dps,Shepherd,direction,n,c,rs,rhos,h):
    clusterpoints=clusteringEffect(initpoints,n,c,rs,Shepherd)
    errorpoints=angularnoiseEffect(initpoints,e,p) # The effect of noise    <----working
    repelledpoints=selfRepellingEffect(initpoints,ra,rhoa)         #<--working
    fearpoints=sheperdingEffect(initpoints,Shepherd,rs,rhos)
    inertiapoints=inertialEffect(initpoints[:],direction[:],h)
    allpoints=repelledpoints+fearpoints+errorpoints+clusterpoints+inertiapoints
    initpoints,direction=stepspertimestep(allpoints,initpoints,dps)   #<--working
    #return initpoints

def stepspertimestep(currentpoints,initialpoints,steps): # to limit the step taken per unit time
#steps: steps to be taken towards currentpoints from initialpoints
    difference=currentpoints-initialpoints
    norm=numpy.linalg.norm(difference,axis=0)    
    l=numpy.shape(difference)
    print("in stepsper L : " , l , "  dodododododdodo")
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

def move( point1,point2,intensity,direction ):
    if numpy.shape(point1)[0]!=0 and numpy.shape(point2)[0]!=0:
        difference=numpy.asarray(point1)-numpy.asarray(point2)
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
                #print("norm : " , norm , " OOOOOOOOOOOOOOOOOOOOOOOOOOO")
                unitvec=[difference[0]/norm,difference[1]/norm]
            point1[0]=point1[0]-direction*(intensity)*unitvec[0]
            point2[0]=point2[0]+direction*(intensity)*unitvec[0]
            point1[1]=point1[1]-direction*(intensity)*unitvec[1]
            point2[1]=point2[1]+direction*(intensity)*unitvec[1]
    return point1,point2


def main():
    rospy.init_node('sheep_sheperd', anonymous=True)
    #pub = rospy.Publisher('chatter', String, queue_size=10)
    #sub = rospy.Subscriber("chatter",String,callback)
    e=0.3 # strength of error movement
    p=0.05 # probability of an error
    Xstartpos=0 #swarm initial position
    Ystartpos=0
    
    N=40   # number of agents
    ra=2.0  # distance for self repulsion
    n=numpy.ceil(0.9*N)
    Pd = ra*log(N)
    Pc = ra
    threshold = 5
    dps=1 # speed of agents
    h = 0.5
    f = (N^(1/2))
    rhoa=2.0# strength of self repulsion 
    rhos=1.0#strength of predatory repulsion 
    Shepherd=[-0,100]
    rs=30   #radius of influence
    c=1.05 #strength of clustering
    destination=[0,100] #destination
    x=Xstartpos+numpy.random.rand(1,N) #swarm initialization
    y=Ystartpos+numpy.random.rand(1,N) #gives random values from [0,1)
    initpoints=numpy.concatenate((x,y),axis=0)
    direction=numpy.zeros((2,N))


    pub_marker = rospy.Publisher('sheep_sheperd', MarkerArray , queue_size=1)

    #N = 100    

    rate = rospy.Rate(10) # 10hz

    M = MarkerArray()    
    #x = []
    #y = []


    shape = Marker.CUBE;
    for i in range(N+1): #to include shepherd too
        marker = Marker()
        print("i " , i)
        marker.header.frame_id = "/sheep_sheperd";
        marker.header.stamp = rospy.get_rostime();
        

        marker.ns = "shp";
        
        marker.id = i;
        
        marker.type = shape;
        
        marker.action = Marker.ADD;

        if(i != N):
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

        if(i == N):
            marker.ns = "sheperd"
            marker.pose.position.x = Shepherd[0];
            marker.pose.position.y = Shepherd[1];
            marker.color.r = 1.0;
            marker.color.g = 0.0;


        #x.append(marker.pose.position.x)
        #y.append(marker.pose.position.y)


        M.markers.append(marker)
        print("id = " , M.markers[i].id , "ini pos " , x[0][i%N] , " , " , y[0][i%N])

        m=numpy.size(initpoints[0])
        print("this will be the new N " , m)
    #for i in range(3):
        #M.markers[i].action = marker.ADD
    count = 0;    

    while not rospy.is_shutdown():
        #hello_str = "hello world %s" % rospy.get_time()
        #rospy.loginfo(hello_str)
        #print("hello_str")
        #pub.publish(hello_str)
        #print(M.markers[0].id , " " , M.markers[1].id , "  " , M.markers[2].id , "tolo")
        pub_marker.publish(M)
        #print(M.markers[0].id , " " , M.markers[1].id , "  " , M.markers[2].id)
        N=numpy.size(initpoints[0])
        Pd=ra*numpy.log(N)
        Pc=ra
        f=ra*numpy.sqrt(N)
        #plotpoints(initpoints,Shepherd,direction,rs)
        sheepmovement(initpoints,e,p,ra,rhoa,dps,Shepherd,direction,n,c,rs,rhos,    h)
        singlesheperdMovement(Shepherd,initpoints[0],initpoints[1],ra,Pd,Pc,destination,f)
        rospy.Duration(0.05)
        #plt.pause(0.05) --- similar to ros::Duration(0.05) or rospy.Duration(0.05)
        #plt.clf() --- clears the plot screen blah blah
        
        #print(initpoints)
        print("yolo")
        print("sheperd:" , Shepherd)
        print("oloy")

        for i in range(N):
            M.markers[i].pose.position.x = initpoints[0][i] 
            M.markers[i].pose.position.y = initpoints[1][i] 

        M.markers[N].pose.position.x = Shepherd[0];
        M.markers[N].pose.position.y = Shepherd[1];
            #x[0][i] = M.markers[i].pose.position.x
            #y[0][i] = M.markers[i].pose.position.y
            #print(i , " x " , M.markers[i].pose.position.x , " id " , M.markers[i].id)
        flag = 0    
        for i in range(N):
            if(numpy.sqrt(numpy.square(initpoints[0][i] - destination[0]) + numpy.square(initpoints[1][i] - destination[1])) < threshold):
                flag = flag + 1
        if(flag >= n):
            print("STOPPPPPPPPPPPPPP " , "count is: " , count)
            break        

        # count = count + 1;
        # if(count > 1000):
        #     print("Count more than 1000")
        #     break    
        rate.sleep()




if __name__ == '__main__':
    #x = []
    #y = []
    try:
        main()
    except rospy.ROSInterruptException:
        pass
