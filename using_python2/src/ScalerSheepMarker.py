#!/usr/bin/env python
# license removed for brevity
import rospy
import array
from std_msgs.msg import String
from visualization_msgs.msg import *
import numpy
import matplotlib.pyplot as plt
import copy
from math import *

# def callback(msg):
#     message = "recieved  " + msg.data
#     print(message)
Shepherd = []
nos = 0
#X1 => initpoints1
# function [ X,Y,unitvec ] = sheepmovements(X,Y,n,c,rs,S,ra,rhoa,rhos,unitvec,h,e,initpoints,dps,mode,destination)
# %SHEEPMOVEMENTS Summary of this function goes here
# %   Detailed explanation goes here
# else
#     if n~=1
#         [X,Y] = clusteringEffect(X,Y,n,c,rs,S);
#     end
# end
# idx1=find(pdist2([X' Y'],destination)>=10);
# [X1,Y1] = sheperdingEffect(X,Y,S,rs,rhos);
# X1(~idx1)=X(~idx1);
# Y1(~idx1)=Y(~idx1);
# X=abs(X-X1);
# Y=abs(Y-Y1);
# X=X+Y;
# idx=find(X~=0);
# idx=intersect(idx,idx1);
# if ~isempty(idx) && ~isempty(unitvec) 
#     [X1(idx),Y1(idx)] = inertialEffect(X1(idx),Y1(idx),unitvec(idx,:),h);
# end
# X=X1;
# Y=Y1;
# r=randi(size(X,2),1);
# idx=find(r<0.05);
# if ~isempty(idx)
#     [X(idx),Y(idx)] = angularnoiseEffect(X(idx),Y(idx),e);
# end
# [X(idx1),Y(idx1)] = selfRepellingEffect(X(idx1),Y(idx1),ra,rhoa);
# idx=find(pdist2([X' Y'],destination)>=10);
# if ~isempty(idx)
#     [X(idx),Y(idx),unitvec1] = stepspertimestep(X(idx),Y(idx),initpoints(idx,:),dps);
#     unitvec(idx,1)=unitvec1(:,1);
#     unitvec(idx,2)=unitvec1(:,2);
#     idx=setdiff(n,idx);
#     if idx
#         X1=X(idx);
#         Y1=Y(idx);
#         R=rand(2,length(idx));
#         X1=X1+R(1,:);
#         Y1=Y1+R(2,:);
#         idx1=find(pdist2([X1' Y1'],destination)>10);
#         if idx1
#             X1(idx1)=X(idx(idx1));
#             Y1(idx1)=Y(idx(idx1));
#         end
#         X(idx)=X1;
#         Y(idx)=Y1;
#     end
# else
#     [X,Y,unitvec] = stepspertimestep(X,Y,initpoints,dps);
# end
# % [X,Y] = placeHerd(X,Y);
# end

def sheepmovements(initpoints,n,c,rs,Shepherd,ra,rhoa,rhos,unitvec,h,e,dps,mode,destination,p,dist_threshold):
    copypoints = copy.deepcopy(initpoints)
    if(n != 1):
        initpoints = clusteringEffect(initpoints,n,c,rs,Shepherd)
    idx1 = []
    X = initpoints[0,:]
    Y = initpoints[1,:]
    for i in range(len(X)):
        dist = sqrt(pow(initpoints[0][i] - destination[0],2) + pow(initpoints[1][i] - destination[1],2))
        if(dist >= dist_threshold):
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
        if(dist >= dist_threshold):
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
            if(dist > dist_threshold):
                idx1.append(i)
        if(len(idx1)!= 0 ):
            for i in idx1:
                initpoints1[:,i] = initpoints[:,idx[i]]
        initpoints[:,idx] = initpoints1            
    else:
        initpoints,unitvec = stepspertimestep(initpoints,copypoints,dps)    


# def sheepmovement(initpoints,e,p,ra,rhoa,dps,Shepherd,direction,n,c,rs,rhos):
#     clusterpoints=clusteringEffect(initpoints,n,c,rs,Shepherd)
#     errorpoints=angularnoiseEffect(initpoints,e,p) # The effect of noise    <----working
#     repelledpoints=selfRepellingEffect(initpoints,ra,rhoa)         #<--working
#     fearpoints=sheperdingEffect(initpoints,Shepherd,rs,rhos)
#     allpoints=repelledpoints+fearpoints+errorpoints+clusterpoints
#     initpoints,direction=stepspertimestep(allpoints,initpoints,dps)   #<--working

def shepherdmovement(Shepherd,initpoints,ra,Pd,Pc,destination,f,rs):
    shepherdcircle=plt.Circle(Shepherd,rs, color='r', alpha=0.2, lw=5)
    sheeppoints=initpoints
    GCM=initpoints.mean(1)

    
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



    #return initpoints

def stepspertimestep(currentpoints,initialpoints,steps): # to limit the step taken per unit time
#steps: steps to be taken towards currentpoints from initialpoints
    difference=currentpoints-initialpoints
    norm=numpy.linalg.norm(difference,axis=0)
    l=numpy.shape(difference)
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
        currentpoints[0][i]=currentpoints[0][i]+unitvec[0]*step
        currentpoints[1][i]=currentpoints[1][i]+unitvec[1]*step
    return currentpoints,direction

def move( point1,point2,intensity,direction ):
    if numpy.shape(point1)[0]!=0 and numpy.shape(point2)[0]!=0:
        difference=point1-point2
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

def callback(msg):
    print("DONT ENTER THIS")
    if(len(Shepherd) == 0):
        Shepherd.append(msg.pose.position.x)
        Shepherd.append(msg.pose.position.y)
    else:    
        Shepherd[0] = msg.pose.position.x
        Shepherd[1] = msg.pose.position.y


def callback_multi_sheperd(msg):
    global Shepherd , nos

    nos = numpy.shape(msg.markers)[0]
    nos = nos - 1
    #print("nos: " , nos  )
    #print(msg)

    #print("LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL")
    if(len(Shepherd) == 0):
        for i in  range(nos):
            Shepherd.append([msg.markers[i].pose.position.x, msg.markers[i].pose.position.y])
    else:
        for i in range(nos):
            Shepherd[i] = ([msg.markers[i].pose.position.x, msg.markers[i].pose.position.y])

def main():
    rospy.init_node('sheep_marker', anonymous=True)
    #pub = rospy.Publisher('chatter', String, queue_size=10)
    #sub = rospy.Subscriber("chatter",String,callback)
    scalar = 10.0;
    e=0.3 # strength of error movement
    p=0.05 # probability of an error
    Xstartpos=50 #swarm initial position
    Ystartpos=0
    h = 0.5
    mode = 0
    threshold = 10.0/scalar
    N=10# number of agents
    
    n=numpy.ceil(0.9*N)
    dps=1/scalar # speed of agents
    ra=2.0/10.0  # distance for self repulsion
    rhoa=2.0# strength of self repulsion 
    rhos=1.0#strength of predatory repulsion 
    #Shepherd=[-0,0]
    rs=10/scalar   #radius of influence
    c=1.05 #strength of clustering
    destination=[70,70] #destination
    destination[0] = destination[0]/scalar
    destination[1] = destination[1]/scalar

    dist_threshold = threshold/scalar
    
    x=Xstartpos+numpy.random.rand(1,N) #swarm initialization
    y=Ystartpos+numpy.random.rand(1,N) #gives random values from [0,1)
    initpoints=numpy.concatenate((x,y),axis=0)
    direction=numpy.zeros((2,N))
    unitvec = numpy.zeros((2,N))

    pub_marker = rospy.Publisher('marker_sheep', MarkerArray , queue_size=1)
    #sub_marker = rospy.Subscriber("marker_sheperd",Marker,callback)
    sub_marker_array = rospy.Subscriber("rover_sheperd", MarkerArray, callback_multi_sheperd)
    #N = 100    

    rate = rospy.Rate(10) # 10hz

    M = MarkerArray()    
    #x = []
    #y = []


    shape = Marker.CUBE;
    for i in range(N):
        marker = Marker()
        print("i " , i)
        marker.header.frame_id = "/my_marker_sheep";
        marker.header.stamp = rospy.get_rostime();
        
        marker.ns = "sheep";
        
        marker.id = i;
        
        marker.type = shape;
        
        marker.action = Marker.ADD;

        
        marker.pose.position.x = x[0][i];            

        marker.pose.position.y = y[0][i];
        
        marker.pose.position.z = 0;

        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        
        marker.scale.x = 0.3;
        marker.scale.y = 0.3;
        marker.scale.z = 0.3;
        
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
   
        marker.lifetime = rospy.Duration();

        #x.append(marker.pose.position.x)
        #y.append(marker.pose.position.y)


        M.markers.append(marker)
        print("id = " , M.markers[i].id , "ini pos " , x[0][i] , " , " , y[0][i])

        m=numpy.size(initpoints[0])
        print("this will be the new N " , m)
    #for i in range(3):
        #M.markers[i].action = marker.ADD

    count = 0    
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
        sheepmovements(initpoints,n,c,rs,Shepherd,ra,rhoa,rhos,unitvec,h,e,dps,mode,destination,p,dist_threshold)
        #shepherdmovement(Shepherd,initpoints,ra,Pd,Pc,destination,f,rs)
        rospy.Duration(0.05)
        #plt.pause(0.05) --- similar to ros::Duration(0.05) or rospy.Duration(0.05)
        #plt.clf() --- clears the plot screen blah blah
        #print(initpoints)
        print("yolo")
        print("Shepherd : " , Shepherd)

        for i in range(N):
            M.markers[i].pose.position.x = initpoints[0][i] 
            M.markers[i].pose.position.y = initpoints[1][i] 
            #x[0][i] = M.markers[i].pose.position.x
            #y[0][i] = M.markers[i].pose.position.y
            #print(i , " x " , M.markers[i].pose.position.x , " id " , M.markers[i].id)
        dist = []    
        flag = 0
        
        for i in range(N):
            dist.append(sqrt(pow(initpoints[0][i] - destination[0],2) + pow(initpoints[1][i] - destination[1],2)))
            if(sqrt(pow(initpoints[0][i] - destination[0],2) + pow(initpoints[1][i] - destination[1],2)) <= threshold):
                flag = flag + 1

        outthreshold = []        
        for i in range(N):
            dist.append(sqrt(pow(initpoints[0][i] - destination[0],2) + pow(initpoints[1][i] - destination[1],2)))
            if(sqrt(pow(initpoints[0][i] - destination[0],2) + pow(initpoints[1][i] - destination[1],2)) > threshold):
                #flag = flag + 1
                outthreshold.append(i)
        print("flag: " , flag)        
        if(flag >= numpy.ceil(0.9*N)):
            print("STOPPPPPPPPPPPPPP " , "count is: " , count)
            break        
        count = count + 1;
        if(count > 1000):            
            break    

        rate.sleep()




if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
