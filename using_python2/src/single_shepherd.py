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

X = []
Y = []
N = 0;


#count = 0
# function [ S,currentmode ] = singlesheperdMovement( S,X,Y,ra,Pd,Pc,destination,f)
# %SHEPERDMOVEMENT Summary of this function goes here
# %   Detailed explanation goes here
# D = pdist2([X' Y'],S,'euclidean');
# [idx]=find(D<ra);
# currentmode=2;
# if isempty(idx)
#     XGCM=sum(X)/length(X);
#     YGCM=sum(Y)/length(Y);
#     plot(XGCM,YGCM,'b*')
#     DGCM=pdist2([X' Y'],[XGCM YGCM],'euclidean');
#     [idx]=find(DGCM>f);
#     if isempty(idx)
#         S=singleherdingPosition(S,Pd,X,Y,destination);
#         currentmode=1;
#     else
#         [~,idx]=max(DGCM);
#         last=[X(idx) Y(idx)];
# %         S=singleherdingPosition(S,Pc,X,Y,destination);
#         S=singlecollectingPosition(S,last,XGCM,YGCM,Pc);
#         currentmode=2;
#     end
# end
# end

def singlesheperdMovement( S,X,Y,ra,Pd,Pc,destination,f):
	D = []
	idx = []
	l = numpy.shape(X)
	for i in range(l[0]):
		D.append(numpy.sqrt(numpy.square(X[i] - S[0]) + numpy.square(Y[i] - S[1])))
		if(D[i] < ra):
			idx.append(i)
	currentmode = 2 
	if(len(idx) == 0 and len(X) != 0):
		Xgcm = sum(X)/len(X)
		Ygcm = sum(Y)/len(Y)
		Dgcm = []
		ind = []
		for i in range(l[0]):  # or len if X is a list and not an array		
			Dgcm.append(numpy.sqrt(numpy.square(X[i] - Xgcm) + numpy.square(Y[i] - Ygcm)))
			if(Dgcm[i] > f):
				ind.append(i)
		if(len(ind) == 0):
			S = singleherdingPosition(S,Pc,X,Y,destination)			
			currentmode = 1;
		else:
			pos = Dgcm.index(max(Dgcm))
			last = [X[pos],Y[pos]]
			S = singlecollectingPosition(S,last,Xgcm,Ygcm,Pd)
			currentmode = 2				
	#return S		
	print("S in single movement : " , S)
	#return S

# function [S] = singleherdingPosition(S,Pc,X,Y,destination)
# %HERDINGPOS Summary of this function goes here
# %   Detailed explanation goes here
# DGCM=pdist2([X' Y'],destination,'euclidean');
# [~,idx]=max(DGCM);
# if ~isempty(idx)
#     [S,~]=move([X(idx),Y(idx)],destination,Pc,-1);
# end
# end
def singleherdingPosition(S,Pc,X,Y,destination):
    print("In HERDING")
    l = numpy.shape(X)
    print("l " , l )
    print("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$")
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
        agent = numpy.asarray([X[pos],Y[pos]])
        #print("agent " , agent)
        S,s2 = move(agent,destination,Pc,-1) #s2 is useless
        print("New Sheperd position: " , S)
        #return S

# function [ S ] = singlecollectingPosition( S,last,XGCM,YGCM,Pd)
# %COLLECTINGPOSITION Summary of this function goes here
# %   Detailed explanation goes here
# plot(S(1),S(2),'k+')
# [l,~]=move(last,[XGCM,YGCM],Pd,-1);
# [S,~]=move(S,l,1,1);
# end
def singlecollectingPosition(S,last,Xgcm,Ygcm,Pd):
    print("In COLLECTING")
    agent = numpy.zeros(2)
    agent[0] = Xgcm
    agent[1] = Ygcm
    last = numpy.asarray(last)
    #print("last: ", last , " numpy.asarray([Xgcm,Ygcm])  " , agent)
    l,s2 = move(last,agent,Pd,-1) #s2 is useless
    S,s2 = move(S,l,1,1) 
    #return S

# function [ point1,point2] = move( point1,point2,intensity,direction )
# %MOVE -1 -> away from the two points, +1 towards
# if ~isempty(point1) && ~isempty(point2)
#     if direction==1
#         dif=point1-point2;
#         unitvec=dif/norm(dif);
#         point1=point1-(intensity)*unitvec;
#         point2=point2+(intensity)*unitvec;
#     else
#         dif=point1-point2;
#         unitvec=dif/norm(dif);
#         point1=point1+(intensity)*unitvec;
#         point2=point2-(intensity)*unitvec;
#     end
# end
# end
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
	#X = []
	#Y = []
	#print("x[0] msg: " , (msg.markers[0].pose.position.x) , "blah blah " , numpy.shape(msg.markers)[0])
	N = numpy.shape(msg.markers)[0]
	if(len(X) == 0):
		for i in range(numpy.shape(msg.markers)[0]):
			X.append(msg.markers[i].pose.position.x)
			Y.append(msg.markers[i].pose.position.y)
	else:		
		for i in range(numpy.shape(msg.markers)[0]):
			X[i] = 	msg.markers[i].pose.position.x
			Y[i] = 	msg.markers[i].pose.position.y
			

def main():
	rospy.init_node('single_shepherd_marker', anonymous=True)
	#X = []
	#Y = []
	r = rospy.Rate(10)
	#sub = MarkerArray()
	sub = rospy.Subscriber("marker_sheep",MarkerArray,callback)
	#pub_array = rospy.Publisher('array_sheperd',MarkerArray,queue_size=1)
	pub = rospy.Publisher('marker_sheperd', Marker , queue_size=1)

	#print("sub has the : " , sub.markers[0].pos.position.x)
	#print("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&")
	Xstartpos=0 #swarm initial position
	Ystartpos=0

	N=5  # number of agents
	Pd = 1.5
	Pc = 1.5
	ra=2.0  # distance for self repulsion
	f = ra*(N^(2/3))
	rs = 30
	#X = Xstartpos+10*numpy.random.rand(1,N) #swarm initialization
	#Y = Ystartpos+10*numpy.random.rand(1,N)  
	#initpoints=numpy.concatenate((X,Y),axis=0)
	#print(initpoints[0])
	S = [30,30]
	S2 = [-30,-30]
	#Sh = numpy.asarray([S,S2]).reshape(2,2)
	#We can resize to any size depending on number of Sheperds
	destination=[0,100] #destination
	q = 1.8
	rhoash = 0
	I = []

	#M = MarkerArray()

	shape = Marker.CUBE
	#for i in range(numpy.shape(Sh)[0]):
	marker = Marker()
	marker.header.frame_id = "/my_marker_sheep";
	marker.header.stamp = rospy.get_rostime();

	marker.ns = "sheep";

	marker.id = N;

	marker.type = shape;

	marker.action = Marker.ADD;


	marker.pose.position.x = S[0];            

	marker.pose.position.y = S[1];

	marker.pose.position.z = 0;

	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;

	marker.scale.x = 1.0;
	marker.scale.y = 1.0;
	marker.scale.z = 1.0;

	marker.color.r = 1.0;
	marker.color.g = 0.0;
	marker.color.b = 0.0;
	marker.color.a = 1.0;

	marker.lifetime = rospy.Duration();
	#M.markers.append(marker)

	while not rospy.is_shutdown():
		pub.publish(marker)
		#pub_array.publish(M)
		print("yolo")
		print("X ")
		print(X)
		print("Y") 
		print(Y)
		print("Sheperd" , S )
		singlesheperdMovement(S,X,Y,ra,Pd,Pc,destination,f)
		#purepredator(Sh,X,Y,ra,Pd,Pc,destination,q,f,rhoash,rs,I)
		#for i in range(numpy.shape(Sh)[0]):	
		marker.pose.position.x = S[0]
		marker.pose.position.y = S[1]
			
		#M.markers.append(marker)


		#print("Sheperd after function call" , Sh)
		r.sleep()

if __name__ == '__main__':
    #X = []
    #Y = []	
    try:
        main()
    except rospy.ROSInterruptException:
        pass




