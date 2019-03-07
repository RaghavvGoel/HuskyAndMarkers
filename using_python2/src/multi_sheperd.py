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

pi = 22/7

def singlecollectingPosition(S,last,Xgcm,Ygcm,Pd):
    print("In COLLECTING")
    agent = numpy.zeros(2)
    agent[0] = Xgcm
    agent[1] = Ygcm
    last = numpy.asarray(last)
    print("last: ", last , " numpy.asarray([Xgcm,Ygcm])  " , agent)
    l,s2 = move(last,agent,Pd,-1) #s2 is useless
    S,s2 = move(S,l,1,1) 

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
    print("In HERDING" , " " , type(S))
    l = numpy.shape(X)
    print("l " , l )
    print("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$")
    #Dgcm = numpy.zeros(numpy.shape(X))
    Dgcm = []
    for i in range(l[0]):  # or len if X is a list and not an array     
        Dgcm.append(numpy.sqrt(numpy.square(X[i] - destination[0]) + numpy.square(Y[i] - destination[1])))
    #idx = numpy.argmax(Dgcm,axis = 0)
    #if(l[0] != 0):
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

# function [ S ] = singlecollectingPositionwithin(S,Pc,X,Y,destination)
# %SINGLECOLLECTINGPOSITIONWITHIN Summary of this function goes here
# %   Detailed explanation goes here
# DGCM=pdist2([X' Y'],destination,'euclidean');
# [~,idx]=max(DGCM);
# if ~isempty(idx)
#     [S,~]=move([X(idx),Y(idx)],destination,Pc,-1);
# end
# end
def singlecollectingPositionwithin(S,Pc,X,Y,destination):
	DGCM = []
	for i in range(numpy.shape(numpy.transpose(X))[0]):
		DGCM.append(numpy.sqrt(numpy.square(X[i] - destination[0]) + numpy.square(Y[i] - destination[1])))
	#if(len(X) != 0):
	idx = []
	idx.append(DGCM.index(max(DGCM)))
	print("idx :  " , idx , " iiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiii")
	if(len(idx) != 0):
		pt = [X[idx[0]],Y[idx[0]]]
		pt = numpy.asarray(pt)
		S,s2 = move(pt,destination,Pc,-1)	
	#return S

# function [ points ] = getcircularpoints( centre,radius,Nopoints,ang,q)
# %GETCIRCULARPOINTS Summary of this function goes here
# %   Detailed explanation goes here
# i=1:Nopoints;
# if rem(Nopoints,2)==0
#     ang=ang-0.5*q*pi/Nopoints;
# end
# points(:,1)=centre(1)+radius*cos(ang+((i-ceil((Nopoints)/2))*q*pi/Nopoints));
# points(:,2)=centre(2)+radius*sin(ang+((i-ceil((Nopoints)/2))*q*pi/Nopoints));
# end
def getcircularpoints(centre,radius,Nopoints,ang,q):
	#Nopoints = no. of sheperds + 1 
	print("Nopoints: " , Nopoints , "  " , numpy.zeros([1,2]))
	print("^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^")
	print("center " , centre , " rad " , radius , " ang " , ang)
	if(Nopoints%2 == 0):
		ang = ang - 0.5*q*3.14/Nopoints
	points = numpy.zeros([Nopoints,2])	
	for i in range(Nopoints):
		points[i,0] = centre[0]+radius*numpy.cos(ang+((i-ceil((Nopoints)/2))*q*pi/Nopoints));
		points[i,1] = centre[1]+radius*numpy.sin(ang+((i-ceil((Nopoints)/2))*q*pi/Nopoints));	
	print("points in getcircular: " , points , "cos " , numpy.cos(1.57))
	print("Returning back..............................")
	return points			


def stepspertimestepSheperd(currentpoints,initialpoints,steps): # to limit the step taken per unit time
#steps: steps to be taken towards currentpoints from initialpoints
    difference=currentpoints-initialpoints
    norm=numpy.linalg.norm(difference,axis=0)
    l=numpy.shape(difference)
    print("in stepper  L : " , l , " " , norm, " llolololooloolooololo")
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
    currentpoints[0]=currentpoints[0]+unitvec[0]*step
    currentpoints[1]=currentpoints[1]+unitvec[1]*step
    print("currentpoints " , currentpoints , " poppppopopopopo")
    return currentpoints,direction

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

# function [ Sh] = formcircle( Sh,centre,radius,ang,q )
# %FORMCIRCLE Summary of this function goes here
# %   Detailed explanation goes here
# s=size(Sh,1);
# points=getcircularpoints(centre,radius,s,ang,q);
# % plot(points(:,1),points(:,2),'m*');
# for i=1:size(Sh,1)
#     [D]=pdist2(points,Sh,'euclidean');
#     [~,idx]=min(D(:,i));
#     S(i,1)=points(idx,1);
#     S(i,2)=points(idx,2);
#     Sh(i,1)=-1000;
#     Sh(i,2)=-1000;
#     points(idx,2)=10000;
#     points(idx,2)=10000;
# end
# Sh=S;
# end
def formcircle(Sh,centre,radius,ang,q):
    s = numpy.shape(Sh)[0]
    Sh2 = copy.deepcopy(Sh)
    points = getcircularpoints(centre,radius,s,ang,q)
    print("points: " , points , " yyyyyyyyyyyyyyyyyyyyyyyyyyyy")
    S = numpy.zeros([numpy.shape(Sh)[0],2])
    for i in range(numpy.shape(Sh)[0]):
        D = []
        for j in range(numpy.shape(Sh)[0]):
            D.append(numpy.sqrt(points[i][0] - numpy.square(Sh2[i][0]) + numpy.square(points[i][1] - Sh2[i][1])))
        idx = D.index(min(D))
        S[i] = [points[idx,0],points[idx,1]]
        Sh2[i] = [-1000,-1000]       
        points[idx] = [10000,10000]
    Sh = S
    print("in formcircle Sh: " , Sh, " zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz")
    return Sh

# function [  Sh ] = surroundEffect( Sh,X,Y,ra,Pd,destination,q,mindisShep,rhoash,rs)
# %SURROUNDEFFECT Summary of this function goes here
# %   Detailed explanation goes here
# XGCM=sum(X)/length(X);
# YGCM=sum(Y)/length(Y);
# coS = [mean(Sh(:,1)),mean(Sh(:,2))];
# point1=[XGCM,YGCM];
# if 0.5*norm(point1-coS)<3*ra
#     a=3*ra;
# else
#     a=0.5*norm(point1-coS);
# end
# [centre,~]=move(point1,destination,a,1);
# ang=atan2((destination(2)-point1(2)),(destination(1)-point1(1)));
# [Sh]=formcircle(Sh,point1,-rs,ang,q);
# end
def surroundEffect(Sh,X,Y,ra,Pd,destination,q,mindisShep,rhoash,rs):
	radius = 60
	XGCM = numpy.mean(X)
	YGCM = numpy.mean(Y)
	coS = [numpy.mean(Sh[:,0]),numpy.mean(Sh[:,1])]
	point1 = [XGCM,YGCM]
	diff = [point1[0] - coS[0] , point1[1] - coS[1]]
	if(0.5*numpy.linalg.norm(diff) < 3*ra):
		a = 3*ra
	else:
		a = 0.5*numpy.linalg.norm(diff)
	centre,notused = move(point1.destination,a,1)
	ang = numpy.arctan2(destination[1] - point1[1] , destination[0] - point1[0])
	Sh = formcircle(Sh,centre,radius,ang,q)
	return Sh		

# function [  Shup,Shbase,l] = monitorspill(Sh,I,X,Y,ra,Pd,Pc,destination,q,f,rhoash,rs,l,mindisShep)
# %PUREPREDATOR Summary of this function goes here
# %   Detailed explanation goes here
# mode=6;
# currentmode=[];
# GCM=[mean(X) mean(Y)];
# rad=35;
# if ~isempty(l)
#     Shup=Sh(l,:);
#     Sh1=Shup;
# end
# Shbase=Sh(I,:);
# ang=atan2(GCM(2)-destination(2),GCM(1)-destination(1));
# points = getcircularpoints( GCM,rad,size(Shup,1)+1,ang,q);
# % plot(points(:,1),points(:,2));
# if isempty(l)
#     l=[];
#     for i=1:size(Shup,1)
#         midpoint=0.5*sum(points(i:i+1,:));
#         [~,idx]=min(pdist2(midpoint,Sh1));
#         l=[l idx];
#         Sh1(idx,1)=-1000;
#         Sh1(idx,2)=-1000;
#     end
# end
# IN = inpolygon(X',Y',[points(:,1);Shbase(:,1)],[points(:,2);Shbase(:,2)]);
# idx=find(IN==0);
# Xout=X(idx);
# Yout=Y(idx);
# if ~isempty(Xout)
#     for i=1:size(Shup,1)
# %         patch([GCM(1);points(i,1);points(i+1,1)],[GCM(2);points(i,2);points(i+1,2)],'b','FaceAlpha',.01*i)
#         IN = inpolygon(Xout',Yout',[GCM(1);points(i,1);points(i+1,1)],[GCM(2);points(i,2);points(i+1,2)]);
#         idx=find(IN==1);
#         Xin=Xout(idx);
#         Yin=Yout(idx);
#         %         plot(Xin,Yin,'Color',[.3*i 0 0]);
#         DGCM=pdist2([Xin' Yin'],[mean(Xin) mean(Yin)],'euclidean');
#         [idx]=find(DGCM>0.5*f);
#         if isempty(idx)
#             %         [destination,~]=move(destination,GCM,5,1);
#             [ Sh(l(i),:) ] = singleherdingPosition( Sh(l(i),:),Pc,Xin,Yin,GCM);
#         else
#             [ Sh(l(i),:) ] = singlecollectingPositionwithin( Sh(l(i),:),0.25*Pd,Xin,Yin,[mean(Xin) mean(Yin)]);
#         end
#     end
# else
#     [Shbase] = surroundEffect(Sh,X,Y,ra,0.25*Pd,destination,q,mindisShep,rhoash,rs);
#     Shup=[];
# end
# Shup=Sh(l,:);
# end
def monitorspill(Sh,I,X,Y,ra,Pd,Pc,destination,q,f,rhoash,rs,l,mindisShep):
    #currentmode = []
    GCM=[numpy.mean(X), numpy.mean(Y)];
    rad = 35
    if(len(l) != 0):
        Shup = Sh[l,:]
        Sh1 = copy.deepcopy(Shup)
    print("I : " , I , " eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee")    
    Shbase=Sh[I,:]
    print("Shbase: " , Shbase , " fffffffffffffffffffffffffffffff")
    ang = numpy.arctan2(GCM[1]-destination[1],GCM[0]-destination[0])
    #points = getcircularpoints( GCM,rad,numpy.shape(Shup)[0]+1,ang,q);
    points = getcircularpoints( GCM,rad,len(l)+1,ang,q);
    if(len(l) == 0):
        l = []
        for i in range(1): #instead of numpy.shape(Shup)[0] as Shup not defined
            midpoint = [numpy.sum(points[i:i+2,0]),numpy.sum(points[i:i+2,1])]
            D = []
            for i in range(numpy.shape(Sh1)[0]):
                D.append(numpy.sqrt(numpy.square(Sh1[i][0] - midpoint[0]) + numpy.square(Sh1[i][1] - midpoint[1])))
            idx = D.index(min(D))
            l.append(idx)
            Sh1[idx] = [-1000,-1000]
    #for i in range(numpy.shape(Sh)[0]):
    #check if the below will work ?? , else put a for loop or google for path with arrays
    NetX = numpy.zeros(numpy.shape(Shbase)[0] + numpy.shape(points)[0])
    NetY = numpy.zeros(numpy.shape(Shbase)[0] + numpy.shape(points)[0])    
    print("NetX : " , NetX )
    print("NetY : " , NetY)
    print("Shbase: " , Shbase , " " , Shbase[0][0])
    print("points:  " , points)
    print("GGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGG")    
    for i in range(numpy.shape(Shbase)[0] + numpy.shape(points)[0]):
        if(i < numpy.shape(Shbase)[0]):
            NetX[i] = Shbase[i][0]
            NetY[i] = Shbase[i][1]
        else:
            NetX[i] = points[numpy.shape(Shbase)[0] - i][0]
            NetY[i] = points[numpy.shape(Shbase)[0] - i][0]

    print("NetX : " , NetX)
    print("NetY : " , NetY)
    PTS = numpy.asarray([NetX,NetY]).reshape(numpy.shape(NetX)[0],2)

    print("jojojjojojjojojojojojojojojo")        
    #p = path.Path([NetX,NetY])
    #p = path.Path([points[:,0],points[:,1]])
    print("PTS:  " , PTS, " cccccccccccccccccccccccccccccccccccccc")
    p = path.Path([[PTS[i][0],PTS[i][1]] for i in range(numpy.shape(NetX)[0])])
    #p = geometry.Polygon([[PTS[i][0],PTS[i][1]] for i in range(numpy.shape(NetX)[0])])
    print("p: " , p , "ppppppppppppppppppppppppppppppppppppp")
    Xt = numpy.transpose(X)
    Yt = numpy.transpose(Y)
    pts = numpy.zeros([numpy.shape(Xt)[0],2])
    for j in range(numpy.shape(Xt)[0]):
        pts[j][0] = Xt[j]
        pts[j][1] = Yt[j]
    IN = p.contains_points(pts) 
    idx = numpy.where(IN == False)
    Xout = []
    Yout = []           
    for j in idx[0]:
        Xout.append(X[j])
        Yout.append(Y[j])
    if(len(Xout) != 0 and numpy.shape(Shup)[0]>1):
        print("numpy.shape(Shup)[0]: " , numpy.shape(Shup)[0] , "tttttttttttttttttttttttttttt")
        for i in range(numpy.shape(Shup)[0]):
            p = path.Path([[(GCM[0],GCM[1]),(points[i,0],points[i,1]),(points[i+1,0],points[i+1,1])]])
            Xt = numpy.transpose(Xout)
            Yt = numpy.transpose(Yout)
            pts = numpy.zeros([numpy.shape(Xt)[0],2])
            for j in range(numpy.shape(Xt)[0]):
                pts[j][0] = Xt[j]
                pts[j][1] = Yt[j]
            IN = p.contains_points(pts)             
            idx = numpy.where(IN == True)
            Xin = []
            Yin = []
            for j in idx[0]:
                Xin.append(Xout[j]) 
                Yin.append(Yout[j])
            DGCM = []
            idx = []
            for j in range(len(Xin)):
                DGCM.append(numpy.sqrt(numpy.square(Xin[i] - numpy.mean(Xin)) + numpy.square(Yin[i] - numpy.mean(Yin))))        
                if(DGCM[j] > 0.5*f):
                    idx.append(j)
            if(len(idx) == 0):
                Sh[l[i]] = singleherdingPosition(Sh[l[i]],Pc,Xin,Yin,GCM);      
            else:
                Sh[l[i]] = singlecollectingPositionwithin(Sh[l[i]],0.25*Pd,Xin,Yin,[numpy.mean(Xin), numpy.mean(Yin)])   
    else:
        Shbase = surroundEffect(Sh,X,Y,ra,0.25*Pd,destination,q,mindisShep,rhoash,rs);          
        Shup = []
    Shup=Sh[l,:];
    return Shup,l       

# function [ Sh,mode,I] = movestrayseperate(Sh,X,Y,ra,Pd,Pc,destination,q,mindisShep,rhoash,rs,I,f)
# %MOVESTRAYSEPERATE Summary of this function goes here
# %   Detailed explanation goes here
# XGCM=mean(X);
# YGCM=mean(Y);
# mode=5;
# q=1.8;
# f=ra*log(length(X));
# CGCM=[XGCM,YGCM];
# if isempty(I)
#     [~,I]=sort(pdist2(Sh,destination));
#     I=I(end-2:end);
# end
# l=1:size(Sh,1);
# l=setdiff(l,I);
# Shbase=Sh(I,:);
# Shup=Sh(l,:);
# [Shbase] = surroundEffect(Shbase,X,Y,ra,0.25*Pd,destination,q,mindisShep,rhoash,rs);
# % if length(in(in==1))==0
# % [Shup] = surroundEffect(Shup,X,Y,ra,Pd,destination,q,mindisShep,rhoash,-5*rs);
# % end
# [ Shup,~,l] = monitorspill(Sh,I,X,Y,ra,Pd,Pc,destination,q,f,rhoash,rs,l,mindisShep);
# % [Shup,~,~,l]=purepredatorswitch(Shup,X,Y,ra,Pd,Pc,destination,1.5,34,rhoash,rs,l);
# Sh(I,:)=Shbase;
# Sh(l,:)=Shup;
# end
def movestrayseperate(Sh,X,Y,ra,Pd,Pc,destination,q,mindisShep,rhoash,rs,I,f):
	XGCM = numpy.mean(X)
	YGCM = numpy.mean(Y)
	mode = 5
	q = 1.8
	f = ra*numpy.log(len(X))
	CGCM = [XGCM,YGCM]
	dist = numpy.zeros(numpy.shape(Sh)[0])
	if(len(I) == 0):
		for i in range(numpy.shape(Sh)[0]):
			dist[i] = numpy.sqrt(numpy.square(Sh[i][0] - destination[0]) + numpy.square(Sh[i][1] - destination[1]))
		I = numpy.argsort(dist)
		I = I[-3:0] # check that the last 3 entries are coming
		print("I : " , I , " I n movestray blahblahblah")	
	l = i in range(numpy.shape(Sh)[0])
	l = numpy.setdiff1d(l,I) #l has only those entries which arent in I
	Shbase = Sh[I,:]
	Shup = Sh[l,:]
	Shbase = surroundEffect(Shbase,X,Y,ra,0.25*Pd,destination,q,mindisShep,rhoash,rs)
	#In monitorspill why Sh and why not Shup
	Shup,notused,l = monitorspill(Sh,I,X,Y,ra,Pd,Pc,destination,q,f,rhoash,rs,l,mindisShep)
	Sh[I,:] = Shbase
	Sh[l,:] = Shup	
	return Sh

# function [ S ] = singlecollectingPosition( S,last,XGCM,YGCM,Pd)
# %COLLECTINGPOSITION Summary of this function goes here
# %   Detailed explanation goes here
# plot(S(1),S(2),'k+')
# [l,~]=move(last,[XGCM,YGCM],Pd,-1);
# [S,~]=move(S,l,1,1);
# end
def singlecollectingPosition( S,last,XGCM,YGCM,Pd):
	l,s = move(numpy.asarray(last),numpy.asarray([XGCM,YGCM]),Pd,-1)
	S,s = move(S,l,1,1)	

# function [ Sh,mode ] = collectstrayseperate( Sh,X,Y,ra,Pd,Pc,destination,f,mindisShep,rhoash,rs,ang,pimultiple)
# %COLLECTSTRAYSEPERATE Summary of this function goes here
# %   Detailed explanation goes here
# mode = 4;
# XGCM = mean(X);
# YGCM = mean(Y);
# GCM = [XGCM YGCM];
# D = pdist2([X' Y'],GCM,'euclidean');
# points = getcircularpoints( [XGCM YGCM],max(D(:))+rs/2,size(Sh,1),ang,pimultiple);
# points = [points;points(1,:)];
# plot(points(:,1),points(:,2),'g-')
# Sh1 = Sh;
# ord=[];
# for i=1:size(points,1)-1
#     midpoint=0.5*sum(points(i:i+1,:));
#     [~,idx]=min(pdist2(midpoint,Sh1));
#     ord=[ord idx];
#     Sh1(idx,1)=-1000;
#     Sh1(idx,2)=-1000;
# end
# % Sh1=Sh(ord);
# idx1=[];
# for i=1:size(Sh,1)
#     IN = inpolygon(X',Y',[XGCM;points(i,1);points(i+1,1)],[YGCM;points(i,2);points(i+1,2)]);
#     idx=find(IN==1);
#     Xin=X(idx);
#     Yin=Y(idx);
#     D = pdist2([Xin' Yin'],GCM,'euclidean');
#     [~,idx]=max(D);
#     last=[Xin(idx) Yin(idx)];
#     if ~isempty(idx)
#         [ Sh(ord(i),:) ] = singlecollectingPosition( Sh(ord(i),:),last,XGCM,YGCM,Pd/4);
#     else
#         idx1=[idx1 i];
#     end
#     if idx1
#         D = min(min(pdist2(Sh,GCM,'euclidean')));
#         for j=1:length(idx1)
#             [Sh(ord(idx1(j)),:),~]=move(GCM,Sh(ord(idx1(j)),:),D,+1);
#         end
#     end
# end
# IN = inpolygon(X',Y',[XGCM;Sh(:,1)],[YGCM;Sh(:,2)]);
# idx=find(IN==1);
# Xin=X(idx);
# Yin=Y(idx);
# D = pdist2([Xin' Yin'],GCM,'euclidean');
# idx=find(D(:)>rs);
# if isempty(idx)
#     mode=1;
# end
# end
def collectstrayseperate( Sh,X,Y,ra,Pd,Pc,destination,f,mindisShep,rhoash,rs,ang,pimultiple):
    mode = 4
    XGCM = numpy.mean(X)
    YGCM = numpy.mean(Y)
    GCM = [XGCM,YGCM]
    D = []
    for i in range(len(X)):
        D.append(numpy.sqrt(numpy.square(X[i] - GCM[0]) + numpy.square(Y[i] - GCM[1])))
    #points = numpy.zeros(numpy.shape(Sh)[0] + 1)
    points1 = getcircularpoints( [XGCM,YGCM],max(D)+rs/2,numpy.shape(Sh)[0],ang,pimultiple)    
    points = numpy.zeros([numpy.shape(Sh)[0] + 1,2])
    print("points befre resizing: " , points1 , " fffffffffffffffffffffffffffffff " , points )
    for i in range(numpy.shape(Sh)[0]):
        points[i,0] = points1[i,0]
        points[i,1] = points1[i,1]
    points[numpy.shape(Sh)[0],0] = points1[0,0]
    points[numpy.shape(Sh)[0],1] = points1[0,1]    
    print("points after reszing: " , points , " gggggggggggggggggggggggggggggg")
    #points[numpy.shape(points)[0]][0] = points[0][0]
    #points[numpy.shape(points)[0]][1] = points[0][1]
    Sh1 = copy.deepcopy(Sh)
    order = []
    #midpoint = []
    for i in range(numpy.shape(points)[0] - 1):
        midpoint = [numpy.sum(points[i:i+2][0]),numpy.sum(points[i:i+2][1])]
        midpoint[0] = 0.5*midpoint[0]
        midpoint[1] = 0.5*midpoint[1]
        #midpoint[1] = 0.5*numpy.sum(points[i:i+2][1])
        dist = []
        print("Sh1: " , Sh1 , " ffffffffffffffffffffffff midpt: " , midpoint )
        for j in range(numpy.shape(Sh1)[0]):
            dist.append(numpy.sqrt(numpy.square(Sh1[j][0] - midpoint[0]) + numpy.square(Sh1[j][1] - midpoint[1])))
        idx = dist.index(min(dist))
        order.append(idx)
        Sh1[idx] = [-1000,-1000]
    idx1 = []
    for i in range(numpy.shape(Sh)[0]):
        p = path.Path([(GCM[0],GCM[1]),(points[i,0],points[i,1]),(points[i+1,0],points[i+1,1])])
        Xt = numpy.transpose(X)
        Yt = numpy.transpose(Y) 
        pts = numpy.zeros([numpy.shape(Xt)[0],2])
        for j in range(numpy.shape(Xt)[0]):
            pts[j][0] = Xt[j]
            pts[j][0] = Yt[j]
        IN = p.contains_points(pts)
        idx = numpy.where(IN == True)
        Xin = []
        Yin = []
        for j in (idx[0]):
            Xin.append(X[j])
            Yin.append(Y[j])    
        D = []  
        idx = []
        for j in range(numpy.shape(Xin)[0]):
            D.append(numpy.sqrt(numpy.square(Xin[j] - GCM[0]) + numpy.square(Yin[j] - GCM[1]))) 
        #print("D: " , D , " DDDDDDDDDDDDDDDDDDDDDDDDD")
        if(len(D) != 0):
            idx.append(D.index(max(D)))
            last = [Xin[idx[0]],Yin[idx[0]]]
        #print("idx: " , idx , " idxidxidxidxidxidxidixdixi")
        if(len(idx) != 0):
            print("S before singlecollectPos called :  " , Sh[order[i]] )
            #S = Sh[order[i]] 
            Sh[order[i]] = singlecollectingPosition(Sh[order[i]],last,XGCM,YGCM,Pd/4);
            #Sh[order[i]] = S 
            print("AFTER FUNC CALLED: S = " ,  "  " , Sh[order[i]])
        else:
            idx1.append(i)
        
        if(idx1):
            dist = []
            for j in range(numpy.shape(Sh)[0]):
                dist.append(numpy.sqrt(numpy.square(Sh[j][0] - GCM[0]) + numpy.square(Sh[j][0] - GCM[1])))
            D = min(dist)
            print("Sh: " , Sh[order[idx1[0]]], "  order: " , order , "  idx1: " , idx1 , "  movemovemovemovemovemove")
            for j in range(len(idx1)):
                Sh[order[idx1[j]]],s = move(numpy.asarray(GCM),Sh[order[idx1[j]]],D,+1);

    NetX = numpy.zeros(numpy.shape(Sh)[0] + 1)
    NetY = numpy.zeros(numpy.shape(Sh)[0] + 1)
    for i in range(numpy.shape(Sh)[0] + 1):
        if(i < numpy.shape(Sh)[0]):
            NetX[i] = Sh[i][0]
            NetY[i] = Sh[i][1]
        else:
            NetX[i] = XGCM
            NetY[i] = YGCM

    PTS = numpy.asarray([NetX,NetY]).reshape(numpy.shape(NetX)[0],2)
    p = path.Path([[PTS[i][0],PTS[i][1]] for i in range(numpy.shape(NetX)[0])])
    Xt = numpy.transpose(X)
    Yt = numpy.transpose(Y)     
    pts = numpy.zeros([numpy.shape(Xt)[0],2])
    for j in range(numpy.shape(Xt)[0]):
        pts[j][0] = Xt[j]
        pts[j][1] = Yt[j]
    IN = p.contains_points(pts) 
    idx = numpy.where(IN == True)
    Xin = []
    Yin = []
    for i in idx[0]:
        Xin.append(X[i])
        Yin.append(Y[i])
    D = []
    idx = []
    for i in range(len(Xin)):
        D.append(numpy.sqrt(numpy.square(Xin[i] - GCM[0]) + numpy.square(Yin[i] - GCM[1]))) 
        if(D[i] > rs):
            idx.append(i)
    if(len(idx) == 0):
        mode = 1   
    #return Sh,mode             

# function [  Sh,mode,I,currentmode  ] = purepredator(Sh,X,Y,ra,Pd,Pc,destination,q,f,rhoash,rs,I)
# %PUREPREDATOR Summary of this function goes here
# %   Detailed explanation goes here
# mode=6;
# currentmode=[];
# GCM=[mean(X) mean(Y)];
# rad=70;
# % q=1.8;
# ang=atan2(GCM(2)-destination(2),GCM(1)-destination(1));
# points = getcircularpoints( GCM,rad/2,size(Sh,1)+1,ang,q);
# plot(points(:,1),points(:,2));
# Sh1 = Sh;
# if isempty(I)
#     I=[];
#     for i=1:size(Sh,1)
#         midpoint=0.5*sum(points(i:i+1,:));
#         [~,idx]=min(pdist2(midpoint,Sh1));
#         I=[I idx];
#         Sh1(idx,1)=-1000;
#         Sh1(idx,2)=-1000;
#     end
# end
# for i=1:size(Sh,1)
#     patch([GCM(1);points(i,1);points(i+1,1)],[GCM(2);points(i,2);points(i+1,2)],'b','FaceAlpha',.1*i)
#     IN = inpolygon(X',Y',[GCM(1);points(i,1);points(i+1,1)],[GCM(2);points(i,2);points(i+1,2)]);
#     idx=find(IN==1);
#     Xin=X(idx);
#     Yin=Y(idx);
# %         plot(Xin,Yin,'Color',[.3*i 0 0]);
#     DGCM=pdist2([Xin' Yin'],[mean(Xin) mean(Yin)],'euclidean');
#     [idx]=find(DGCM>0.5*f);
#     if isempty(idx)
# %         [destination,~]=move(destination,GCM,5,1);
#         [ Sh(I(i),:) ] = singleherdingPosition( Sh(I(i),:),Pc,Xin,Yin,destination);
#         currentmode(I(i))=1;
#     else
#         [ Sh(I(i),:) ] = singlecollectingPositionwithin( Sh(I(i),:),0.25*Pd,Xin,Yin,[mean(Xin) mean(Yin)]);
#         currentmode(I(i))=2;
#     end
# end
# end
def purepredator(Sh,X,Y,ra,Pd,Pc,destination,q,f,rhoash,rs,I):
	print("numpy.mean(X) " , numpy.mean(X) , "lenX: " , len(X))
	mode = 6
	currentmode = []
	if(len(X) != 0):
		GCM = [numpy.mean(X),numpy.mean(Y)]
		rad = 60
		ang = numpy.arctan2(GCM[1]-destination[1],GCM[0]-destination[0])
		print("numpy.shape(Sh)[0] : " , numpy.shape(Sh) , " GCM " , GCM)
		print("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$")
		points = getcircularpoints(GCM,rad/2,numpy.shape(Sh)[0]+1,ang,q) #if Sh is array
		print("points " , points  , " " , points[0,:])
		print("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@")
		#if Sh is a list
		#points = getcircularpoints(GCM,rad/2,len(Sh[0])+1,ang,q) 

		#Sh1 = copy.deepcopy(Sh) # why do we need this Sh1		
		if(len(I) == 0):
			I = []
			#midpoint = numpy.zeros([numpy.shape(Sh)])			
			#print("midpoint before for loop " , midpoint , " " )
			for i in range(numpy.shape(Sh)[0]):
				midpoint = [numpy.sum(points[i:i+2,0]),numpy.sum(points[i:i+2,1])]
				if(numpy.shape(Sh)[0] > 1):
					midpoint[0] = 0.5*midpoint[0]
					midpoint[1] = 0.5*midpoint[1]	
				print("midpoint " , 0.5*midpoint[0])
				#midpoint = 0.5*[numpy.sum(points[i:i+1,0]),numpy.sum(points[i:i+1,1])]
				#midpoint[i][1] = 0.5*numpy.sum(points[i:i+1,1])					
				print("######################################################")
				D = []
				for j in range(numpy.shape(Sh)[0]):
					D.append(numpy.sqrt(numpy.square(midpoint[0] - Sh[j][0]) + numpy.square(midpoint[0] - Sh[j][1])))
				idx = D.index(min(D))
				I.append(idx)
				#Sh1[idx,0] = -1000
				#Sh1[idx,1] = -1000
		print(numpy.shape(Sh)[0])		
		for i in range(numpy.shape(Sh)[0]):
			p = path.Path([(GCM[0],GCM[1]),(points[i,0],points[i,1]),(points[i+1,0],points[i+1,1])])
			print(numpy.shape(X) , " kkkkkkkkkkkkkkkk")
			Xt = numpy.transpose(X)
			Yt = numpy.transpose(Y)
			pts = numpy.zeros([numpy.shape(Xt)[0],2])
			#pts[:,0] = Xt
			#pts[:,1] = Yt
			#pts = [Xt,Yt]
			for j in range(numpy.shape(Xt)[0]):
				pts[j][0] = Xt[j]
				pts[j][1] = Yt[j]
			#print("just pts: " , pts)
			print("this is the path: " , p)
			#print("X: " , pts[0][0] , " " ,pts[1][0] , " " , pts[0][1] , " " , pts[1][1] , " lolololooololololo")
			IN = p.contains_points(pts)
			print("INNNNNNNNNNNNNNNNNN " , IN)
			idx = numpy.where(IN == True)
			print(idx[0])
			print("sssssssssssssssssssssssssssss")
			Xin = []
			Yin = []
			for j in (idx[0]):
				Xin.append(X[j])
				Yin.append(Y[j])
			DGCM = []
			for j in range(numpy.shape(Xin)[0]):
				DGCM.append(numpy.sqrt(numpy.square(Xin[j] - numpy.mean(Xin)) + numpy.square(Yin[j] - numpy.mean(Yin))))
			idx = []
			print("DGCM" , DGCM , " " ,numpy.shape(Xin)[0])
			for j in range(numpy.shape(Xin)[0]):
				if(DGCM[j] > 0.5*f):
					idx.append(j)	
			#MANGE or ASK ABOUT I:
			print("I " , I , " i[o] " , I[0] , Sh)
			print("Sh : " , Sh[I[1]] , " " , Sh[I[1],:])					
			if(len(idx) == 0 and len(Xin) != 0):
				print("in if, prior to calling singleherding :" , i )
				print(" I[i]: " , I[i] , " Sh[I[i]] " , Sh[I[i]] , " ", type(Sh[I[i]]))
				print("Xin " , Xin )
				print(" Yin " , Yin)
				print("Pc " , Pc , " destination " , destination)	
				#S = Sh[I[i]]			
				Sh[I[i]] = singleherdingPosition( Sh[I[i]],Pc,Xin,Yin,destination)
				#Sh[I[i]],direct = stepspertimestepSheperd(S,Sh[I[i]],1) #sheep speed = 1            		
				#currentmode[I[i]] = 1 
			else:
				if(len(Xin) != 0):
					#S = Sh
					Sh[I[i]] = singlecollectingPositionwithin( Sh[I[i]],0.25*Pd,Xin,Yin,[numpy.mean(Xin), numpy.mean(Yin)])
					#S = movestrayseperate(Sh,X,Y,ra,Pd,Pc,destination,q,mindisShep,rhoash,rs,I,f):
					#Sh,direct = stepspertimestep(S,Sh,1) 

# function [Sh] = herdingPosition(Sh,Pc,X,Y,destination)
# %HERDINGPOS Summary of this function goes here
# %   Detailed explanation goes here
# X1=X;
# Y1=Y;
# diff=size(Sh,1)-length(X);
# S=Sh;
# for i=1:size(Sh,1)-diff
#     SGCM=pdist2([X' Y'],Sh(i,:),'euclidean');
#     [~,idx1]=min(SGCM);
#     [S1,~]=move([X(idx1),Y(idx1)],destination,Pc,-1);
#     plot(S1(1),S1(2),'k*')
#     S(i,1)=S1(1);
#     S(i,2)=S1(2);
#     X(idx1)=[];
#     Y(idx1)=[];
# end
# if diff<=0
#     for i=size(Sh,1)-diff+1:size(Sh,1)
#         SGCM=pdist2([X1' Y1'],Sh(i,:));
#         [~,idx1]=min(SGCM);
#         [S1,~]=move([X1(idx1),Y1(idx1)],destination,Pc,-1);
#         plot(S1(1),S1(2),'c*')
#         S(i,1)=S1(1);
#         S(i,2)=S1(2);
#     end
# end
# Sh=S;
# end
def herdingPosition(Sh,Pc,X,Y,destination):
	X1 = X
	Y1 = Y
	diff = numpy.shape(Sh)[0] - len(X)
	S = copy.deepcopy(Sh)
	for i in range(numpy.shape(Sh)[0]-diff):
		SGCM = []
		for j in range(len(X)):
			SGCM.append(numpy.sqrt(numpy.square(X[j] - Sh[i,0]) + numpy.square(Y[j] - Sh[i,1])))
		idx1 = SGCM.index(min(SGCM))
		S1,s = move(numpy.asarray([X[idx1],Y[idx1]]),destination,Pc,-1)
		S[i,0] = S1[0]
		S[i,1] = S1[1]
		X[idx1] = []
		Y[idx1] = []
	if(diff <= 0):
		for i in range(numpy.shape(Sh)[0]-diff+1,numpy.shape(Sh)[0]):
			SGCM =[]
			for j in range(len(X1)):
				SGCM.append(numpy.sqrt(numpy.square(X1[j] - Sh[i,0]) + numpy.square(Y1[j] - Sh[i,1])))		
			idx1 = SGCM.index(min(SGCM))
			S1,s = move(numpy.asarray([X[idx1],Y[idx1]]),destination,Pc,-1)
			S[i,0] = S1[0]
			S[i,1] = S1[1]	
	Sh = S		
			
def singlesheperdMovement( S,X,Y,ra,Pd,Pc,destination,f):
    D = numpy.zeros(numpy.shape(X))
    idx = []
    l = numpy.shape(X)
    for i in range(l[0]):
        D[i] = numpy.sqrt(numpy.square(X[i] - S[0]) + numpy.square(Y[i] - S[1]))
        if(D[i] < ra):
            idx.append(i)
    currentmode = 2 
    if(len(idx) == 0 and len(X)!=0):
        Xgcm = sum(X)/len(X)
        Ygcm = sum(Y)/len(Y)
        Dgcm = []
        ind = []
        for i in range(l[0]):  # or len if X is a list and not an array     
            Dgcm.append(numpy.sqrt(numpy.square(X[i] - Xgcm) + numpy.square(Y[i] - Ygcm)))
            if(Dgcm[i] > f):
                ind.append(i)
        if(len(ind) == 0):
            S = singleherdingPosition(S,Pd,X,Y,destination)
            currentmode = 1;
        else:
            pos = Dgcm.index(max(Dgcm))
            last = [X[pos],Y[pos]]
            S = singlecollectingPosition(S,last,Xgcm,Ygcm,Pc)
            currentmode = 2             


# function [Sh] = surroundEffectCollector( Sh,X,Y,ra,Pd,Pc,destination,f,mindisShep,rhoash,alpha,ang)
# %SURROUNDEFFECT Summary of this function goes here
# %   Detailed explanation goes here
# RoS=[];
# a=1;
# XGCM=mean(X);
# YGCM=mean(Y);
# GCM=[XGCM YGCM];
# D = pdist2([X' Y'],GCM,'euclidean');
# [Sh]=formcircle(Sh,GCM,alpha*a*0.25*max(D(:))*mindisShep,ang,2);
# end
def surroundEffectCollector( Sh,X,Y,ra,Pd,Pc,destination,f,mindisShep,rhoash,alpha,ang):
	RoS = []
	a = 1
	XGCM = numpy.mean(X)
	YGCM = numpy.mean(Y)
	GCM = [XGCM,YGCM]
	D = []
	for i in range(len(X)):
		D.append(numpy.sqrt(numpy.square(X[i] - GCM[0]) + numpy.square(Y[i] - GCM[1])))
	Sh = formcircle(Sh,GCM,alpha*a*0.25*max(D)*mindisShep,ang,2)	


# function [ Sh,alpha,ang,mode,nin,I,currentmode] = sheperdMovement( Sh,X,Y,ra,Pd,Pc,destination,f,mindisShep,rhoash,alpha,ang,mode,nin,ds,rs,method,predmasterslave,decrad,q,I)
# %SHEPERDMOVEMENT Summary of this function goes here
# %   Detailed explanation goes here
# Sh1=Sh;
# XGCM=sum(X)/length(X);
# YGCM=sum(Y)/length(Y);
# GCM=[XGCM YGCM];
# X1=X;
# Y1=Y;
# currentmode=[];
# i=1;
# plot(XGCM,YGCM,'b*');
# if mode~=5 && mode~=6
#     I=[];
# end
# % distancetodestination=pdist2([XGCM,YGCM],destination)
# % distancefurthes=max(pdist2([XGCM YGCM],[X' Y' ]))
# if size(Sh,1)>=size(X,2) 
#     Sh=herdingPosition(Sh,Pc,X,Y,destination);
# elseif size(Sh,1)==1 % if only one shepherd
#     [Sh , currentmode]=singlesheperdMovement( Sh,X,Y,ra,Pd,Pc,destination,f);
# elseif mode==4  % once the circling is complete
#     ang=0;
#     pimultiple=2;
#     [Sh,mode]=collectstrayseperate( Sh,X,Y,GCM,Pd,Pc,destination,f,mindisShep,rhoash,rs,ang,pimultiple);
# %     predmasterslave
# %     mode
#     if predmasterslave==1
#         mode=5;
#     elseif predmasterslave==3
#         mode=6;
#     elseif predmasterslave==2 && mode==1
#         mode=1;
#     end
# elseif mode==6
#     [ Sh,mode,I,currentmode ] = purepredator(Sh,X,Y,ra,Pd,Pc,destination,q,f,rhoash,rs,I);
# elseif mode==5
#     [ Sh,mode,I ] = movestrayseperate(Sh,X,Y,ra,Pd,Pc,destination,q,mindisShep,rhoash,rs,I);
# else
#     DGCM=pdist2([X' Y'],[XGCM YGCM],'euclidean');
#     [idx]=find(DGCM>f);
#     if length(idx)<=size(X',1) && mode==1 %0.7*size(X',1)
#         mode=1;
#         alpha=1;
#         ang=0;
#         [Sh] = surroundEffect(Sh,X,Y,ra,Pd,destination,q,mindisShep,rhoash,rs);
#     elseif length(idx)>size(Sh,1) || mode==2
#         mode=2;
#         [Sh]=surroundEffectCollector(Sh,X1,Y1,ra,Pd,Pc,destination,i,mindisShep,rhoash,alpha,ang);
#         D1=pdist2(Sh,Sh1,'euclidean');
#         IN=(inpolygon(X1',Y1',Sh(:,1),Sh(:,2)));
#         IN=length(IN);
#         if (trace(D1)<10 || IN==size(X1,1)) && method==1
#             mode=4;
#         elseif method==2
#             D=pdist2(Sh,Sh1,'euclidean');
#             if trace(D)<10
#                 alpha=decrad*alpha;
#                 D=max(max(pdist2(Sh,[XGCM YGCM],'euclidean')));
#                 if D<rs
#                     mode=1;
#                 end
#                 ang=ang+0.5;
#             end
#         end
#     end
# end
# for j=1:size(Sh,1)
#     [Sh(j,1),Sh(j,2)]=stepspertimestep(Sh(j,1),Sh(j,2),[Sh1(j,1),Sh1(j,2)],ds);
# end
# end
def sheperdMovement( Sh,X,Y,ra,Pd,Pc,destination,f,mindisShep,rhoash,alpha,ang,mode,nin,ds,rs,method,predmasterslave,decrad,q,I):
	Sh1 = copy.deepcopy(Sh)
	XGCM = numpy.mean(X)
	YGCM = numpy.mean(Y)
	GCM = [XGCM,YGCM]
	X1 = X
	Y1 = Y
	currentmode = []
	i = 1
	if(mode!=5 and mode!=6):
		I = []
	#Below function not converted	
	#if(numpy.shape(Sh)[0] >= len(X)):
	#	herdingPosition(Sh,Pc,X,Y,destination)
	if(numpy.shape(Sh)[0] == 1):
		singlesheperdMovement( Sh,X,Y,ra,Pd,Pc,destination,f)
	elif(mode == 4):
		ang = 0
		pimultiple = 2
		collectstrayseperate( Sh,X,Y,GCM,Pd,Pc,destination,f,mindisShep,rhoash,rs,ang,pimultiple)
		if(predmasterslave==1):
			mode = 5
		elif(predmasterslave==3):
			mode = 6
		elif(predmasterslave==2 and mode == 1):
			mode = 1
	elif(mode == 6):
		purepredator(Sh,X,Y,ra,Pd,Pc,destination,q,f,rhoash,rs,I)
	elif(mode == 5):
		movestrayseperate(Sh,X,Y,ra,Pd,Pc,destination,q,mindisShep,rhoash,rs,I);
	else:
		DGCM = []
		idx = []
		for i in range(len(X)):
			DGCM.append(numpy.sqrt(numpy.square(X[i] - XGCM) + numpy.square(Y[i] - YGCM)))
			if(DGCM[i] > f):
				idx.append(i)										
		if(len(idx) <= len(X) and mode == 1):
			mode = 1
			alpha = 1
			ang = 0	
			surroundEffect(Sh,X,Y,ra,Pd,destination,q,mindisShep,rhoash,rs);
		elif(len(idx) > numpy.size(Sh)[0] or mode == 2):
		 	mode = 2
		 	surroundEffectCollector(Sh,X1,Y1,ra,Pd,Pc,destination,i,mindisShep,rhoash,alpha,ang);
			D1 = []
			for j in range(numpy.shape(Sh)[0]):
				D1.append(numpy.sqrt(numpy.square(Sh[i][0] - Sh1[i][0]) + numpy.square(Sh[i][1] - Sh1[i][1])))
			p = path.Path([[Sh[i][0],Sh[i][1]] for j in range(numpy.shape(Sh)[0])])
			Xt = numpy.transpose(X1)				
			Yt = numpy.transpose(Y1)
			pts = numpy.zeros([numpy.shape(Xt)[0],2])
			for j in range(len(Xt)):
				pts[j][0] = Xt[j]
				pts[j][1] = Yt[j]
			IN = p.contains_points(pts)
			IN1 = numpy.where(IN == True)	
			IN1 = len(IN1[0])
			if((sum(D1) < 10 or IN1 == len(X1)) and method == 1):
				mode = 4
			elif(method == 2):
				D = []
				for i in range(numpy.shape(Sh)[0]):
					D.append(numpy.sqrt(numpy.square(Sh[i][0] - Sh1[i][0]) + numpy.square(Sh[i][1] - Sh1[i][1])))
				if(sum(D) < 10):
					alpha=decrad*alpha
					dist = []
					for i in range(numpy.shape(Sh)[0]):
						dist.append(numpy.sqrt(numpy.square(Sh[i][0] - XGCM) + numpy.square(Sh[i][1] - YGCM)))
					D = max(dist)
					if(D < rs):
						mode = 1
					ang = ang + 0.5	
	for i in range(numpy.shape(Sh)[0]):
		Sh[i],direction = stepspertimestep(Sh[i],Sh1[i],ds);				 		


def main():
	rospy.init_node('single_shepherd_marker', anonymous=True)
	#X = []
	#Y = []
	r = rospy.Rate(10)
	#sub = MarkerArray()
	sub = rospy.Subscriber("marker_sheep",MarkerArray,callback)
	pub_array = rospy.Publisher('array_sheperd',MarkerArray,queue_size=1)
	#pub = rospy.Publisher('marker_sheperd', Marker , queue_size=1)

	#print("sub has the : " , sub.markers[0].pos.position.x)
	#print("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&")
	Xstartpos=0 #swarm initial position
	Ystartpos=0

	N=5  # number of agents
	Pd = 1.5
	Pc = 1.5
	ra=2.0  # distance for self repulsion
	f = ra*(N^(2/3))
	rs = 10
	#X = Xstartpos+10*numpy.random.rand(1,N) #swarm initialization
	#Y = Ystartpos+10*numpy.random.rand(1,N)  
	#initpoints=numpy.concatenate((X,Y),axis=0)
	#print(initpoints[0])
	S = [0,0]
	S2 = [-5,5]
	Sh = numpy.asarray([S,S2]).reshape(2,2) #Sheperd Array
	#We can resize to any size depending on number of Sheperds
	#destination=[0,100] #destination
	q = 1
	rhoash = 0
	I = []

	mode=2;
	A1=[];
	destination=[125,125];
	herdcount=0;
	collectcount=0;
	method=1;
	#saver=[];
	#Dist=[];
	#human=0;
	#video=1;
	rslimit=4;
	Nslimit=5;
	predmasterslave=1;	
	ang = 0

	

	M = MarkerArray()

	shape = Marker.CUBE
	for i in range(numpy.shape(Sh)[0]):
		marker = Marker()
		marker.header.frame_id = "/my_marker_sheep";
		marker.header.stamp = rospy.get_rostime();

		marker.ns = "sheep";

		marker.id = i+N;

		marker.type = shape;

		marker.action = Marker.ADD;


		marker.pose.position.x = Sh[i][0];            

		marker.pose.position.y = Sh[i][1];

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
		M.markers.append(marker)

	# a1 = numpy.random.rand(1,50)
	# a2 = numpy.random.rand(1,50)
	# a1 = numpy.transpose(a1)
	# a2 = numpy.transpose(a2)
	# a3 = numpy.zeros([50,2])	
	# for i in range(50):
	# 	a3[i][0] = a1[i]
	# 	a3[i][1] = a2[i]
	# print("a3" , a3)	
	# p = path.Path([(0,0), (0, 1), (1, 0)])
	# val = p.contains_points(a3)
	# print("val: " , val)
	# print("*****************************************")		
	# idx = numpy.where(val == True)
	# print("idx" , idx[0])

	# for i in idx[0]:
	# 	print i

	while not rospy.is_shutdown():
		#pub.publish(marker)
		pub_array.publish(M)
		print("yolo")
		#print("X ")
		#print(X)
		#print("Y") 
		#print(Y)
		print("Sheperd" , Sh , " y " , Sh[0][1] , "sizee" , numpy.shape(Sh)[0])
		singlesheperdMovement(Sh,X,Y,ra,Pd,Pc,destination,f)
		purepredator(Sh,X,Y,ra,Pd,Pc,destination,q,f,rhoash,rs,I)
		for i in range(numpy.shape(Sh)[0]):	
			M.markers[i].pose.position.x = Sh[i][0]
			M.markers[i].pose.position.y = Sh[i][1]
			print(i," ShX : ",Sh[i][0] , " ShY: ", Sh[i][1],  " yoyoyoyyoyoyoyoyoyoyyo" )
			#M.markers.append(marker)
		print("Sheperd after function call" , Sh)
		r.sleep()

if __name__ == '__main__':
    #X = []
    #Y = []	
    try:
        main()
    except rospy.ROSInterruptException:
        pass




