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
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
import copy

#from planar import Polygon
huskyX = [0.0 , 0.0, 0.0, 0.0, 0.0, 0.0]
huskyY = [0.0 , 0.0, 0.0, 0.0, 0.0, 0.0]
Yaw    = [0.0 , 0.0, 0.0, 0.0, 0.0, 0.0]
husky_predX = [0.0, 0.0]
husky_predY = [0.0, 0.0]
Yaw_pred = [0.0, 0.0]
I = []

# def callback(msg):
#     message = "recieved  " + msg.data
#     #print(message)


    #return S

def singlecollectingPositionwithin(S,Pc,X,Y,destination):
    #print("In singlecollectingPositionwithin ...............")
    DGCM = []
    for i in range(numpy.shape(numpy.transpose(X))[0]):
        DGCM.append(numpy.sqrt(numpy.square(X[i] - destination[0]) + numpy.square(Y[i] - destination[1])))
    #if(len(X) != 0):
    idx = []
    idx.append(DGCM.index(max(DGCM)))
    #print("idx: " , idx , " popopopopopopopopopopopopop")
    if(len(idx) != 0):
        #print("Sssssssssssss: " , S)
        #S = numpy.zeros([1,2])
        S[0,0] = X[idx[0]]                                                                                 
        S[0,1] = Y[idx[0]]
        #pt = numpy.asarray(pt)
        #print("numpy.ndim(S): " , numpy.ndim(S)  , " " , S)
        S,s2 = moveSheperd(S,destination[:],Pc,-1)   
    #print("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX")
    return S    
    

def formcircle(Sh,centre,radius,ang,q):
    #print("In formcircle ...................")
    s = numpy.shape(Sh)[0]
    Sh2 = numpy.copy(Sh)
    points = getcircularpoints(centre,radius,s,ang,q)
    #print("points: " , points , " yyyyyyyyyyyyyyyyyyyyyy  s: ", s , " Sh2: " , Sh2)
    S = numpy.zeros([numpy.shape(Sh)[0],2])
    for i in range(numpy.shape(Sh)[0]):
        D = []
        for j in range(numpy.shape(Sh2)[0]):
            D.append(numpy.sqrt(numpy.square(points[j][0] - Sh2[j][0]) + numpy.square(points[j][1] - Sh2[j][1])))
        #print("FORMCIRCLE D = : " , D , " points: " , points)
        idx = D.index(min(D))
        S[i] = [points[idx][0],points[idx][1]]
        Sh2[i] = [-1000,-1000]       
        points[idx] = [10000,10000]
        #print(idx , " S[i] " , S[i] , " \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\")
    Sh = S
    #print("in formcircle Sh: " , Sh, )
    #print(" zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz")
    return Sh

def getcircularpoints(centre,radius,Nopoints,ang,q):
    #Nopoints = no. of sheperds + 1 
    #print("In getcircularpoints")
    #print("Nopoints: " , Nopoints , "  " , numpy.zeros([1,2]))
    #print("^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^")
    #print("center " , centre , " rad " , radius , " ang " , ang)
    if(Nopoints%2 == 0):
        ang = ang - 0.5*q*(22.0/7   )/Nopoints
    points = numpy.zeros([Nopoints,2])    
    for i in range(Nopoints):
        points[i,0] = centre[0]+radius*numpy.cos(ang+((i-ceil((Nopoints)/2))*q*3.14/Nopoints));
        points[i,1] = centre[1]+radius*numpy.sin(ang+((i-ceil((Nopoints)/2))*q*3.14/Nopoints));   
    #print("points in getcircular: " , points)
    #print("OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO")
    return points           

def surroundEffectCollector( Sh,X,Y,ra,Pd,Pc,destination,f,mindisShep,rhoash,alpha,ang):
    #print("IN surroundEffectCollector ...............................")
    RoS = []
    a = 1
    XGCM = numpy.mean(X)
    YGCM = numpy.mean(Y)
    GCM = [XGCM,YGCM]
    D = []
    for i in range(len(X)):
        D.append(numpy.sqrt(numpy.square(X[i] - GCM[0]) + numpy.square(Y[i] - GCM[1])))
    if(len(X) > 0):    
        Sh = formcircle(Sh,GCM,alpha*a*0.25*max(D)*mindisShep,ang,2)

    #print("Sh suuroundEffectColl after FORMCIRCLE: " , Sh)
    return Sh 
    #print("EEEEEEEEEEEEEEEEEEEEEE")


def surroundEffect(Sh,X,Y,ra,Pd,destination,q,mindisShep,rhoash,rs):
    #print("In surroundEffect .......................")
    radius = 60
    XGCM = numpy.mean(X)
    YGCM = numpy.mean(Y)
    coS = [numpy.mean(Sh[:,0]),numpy.mean(Sh[:,1])]
    #print("should be mean of sheperd pos coS : " , coS , " cccccccccccccccccc")
    point1 = [XGCM,YGCM]
    diff = [point1[0] - coS[0] , point1[1] - coS[1]]
    if(0.5*numpy.linalg.norm(diff) < 3*ra):
        a = 3*ra
    else:
        a = 0.5*numpy.linalg.norm(diff)
    pt1 = numpy.asarray(point1)
    destination = numpy.asarray(destination)
    centre,notused = moveSheperd(pt1,destination,a,1)
    #print("after moveSheperd centre: " , centre , " also Sh before formcircle: " , Sh)
    centre_list = [centre[0][0],centre[0][1]]
    ang = numpy.arctan2(destination[1] - point1[1] , destination[0] - point1[0])
    Sh = formcircle(Sh,centre_list,radius,ang,q)
    return Sh       
    #print("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&")

def monitorspill(Sh,I,X,Y,ra,Pd,Pc,destination,q,f,rhoash,rs,l,mindisShep):
    #print("In monitorspill ..........................")
    mode = 6
    #currentmode = []
    GCM=[numpy.mean(X), numpy.mean(Y)];
    rad = 35
    if(len(l) != 0):
        Shup = Sh[l,:]
        Sh1 = numpy.copy(Shup)
    #print("I : " , I , " eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee")    
    Shbase=Sh[I,:]
    #print("Shbase: " , Shbase , " fffffffffffffffffffffffffffffff")
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
    #print("NetX : " , NetX )
    #print("NetY : " , NetY)
    #print("Shbase: " , Shbase , " " , Shbase[0][0])
    #print("points:  " , points)
    #print("GGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGG")    
    for i in range(numpy.shape(Shbase)[0] + numpy.shape(points)[0]):
        if(i < numpy.shape(Shbase)[0]):
            NetX[i] = Shbase[i][0]
            NetY[i] = Shbase[i][1]
        else:
            NetX[i] = points[numpy.shape(Shbase)[0] - i][0]
            NetY[i] = points[numpy.shape(Shbase)[0] - i][0]

    #print("NetX : " , NetX)
    #print("NetY : " , NetY)
    PTS = numpy.asarray([NetX,NetY]).reshape(numpy.shape(NetX)[0],2)

    #print("jojojjojojjojojojojojojojojo")        
    #p = path.Path([NetX,NetY])
    #p = path.Path([points[:,0],points[:,1]])
    #print("PTS:  " , PTS, " cccccccccccccccccccccccccccccccccccccc")
    p = path.Path([[PTS[i][0],PTS[i][1]] for i in range(numpy.shape(NetX)[0])])
    #p = geometry.Polygon([[PTS[i][0],PTS[i][1]] for i in range(numpy.shape(NetX)[0])])
    #print("p: " , p , "ppppppppppppppppppppppppppppppppppppp")
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
        #print("numpy.shape(Shup)[0]: " , numpy.shape(Shup)[0] , "tttttttttttttttttttttttttttt")
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
    #print("JJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJ")

    
def movestrayseperate(Sh,X,Y,ra,Pd,Pc,destination,q,mindisShep,rhoash,rs,I,f):
    #print("In movestrayseperate : ,,,,,,,,,,,,,,,,,,,I: " , I)
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
        #print("I : " , I , " I n movestray blahblahblah")   
    l = []    
    for i in range(numpy.shape(Sh)[0]):
        l.append(i)
    l = numpy.setdiff1d(l,I) #l has only those entries which arent in I
    Shbase = Sh[I,:]
    Shup = Sh[l,:]
    Shbase = surroundEffect(Shbase,X,Y,ra,0.25*Pd,destination,q,mindisShep,rhoash,rs)
    #In monitorspill why Sh and why not Shup
    #print("Sh: " , Sh , " I: " , I , " X: " , X , "Y: " , Y)
    #print("mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm ")
    Shup,l = monitorspill(Sh,I,X,Y,ra,Pd,Pc,destination,q,f,rhoash,rs,l,mindisShep)
    Sh[I,:] = Shbase
    Sh[l,:] = Shup  
    return Sh
    #print("LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL")

def herdingPosition(Sh,Pc,X,Y,destination):
    #print("In herdingPosition:  ,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,")
    X1 = X
    Y1 = Y
    diff = numpy.shape(Sh)[0] - len(X)
    S = numpy.copy(Sh)
    for i in range(numpy.shape(Sh)[0]-diff):
        SGCM = []
        for j in range(len(X)):
            SGCM.append(numpy.sqrt(numpy.square(X[j] - Sh[i,0]) + numpy.square(Y[j] - Sh[i,1])))
        idx1 = SGCM.index(min(SGCM))
        S,s = moveSheperd(numpy.asarray([X[idx1],Y[idx1]]),destination,Pc,-1)
        #S[i,0] = S1[0]
        #S[i,1] = S1[1]
        #print("idx1: " , idx1 , " " , type(X))
        #if(len(idx1) != 0):
        #X[idx1] = []
        #Y[idx1] = []
        numpy.delete(X,idx1)
        numpy.delete(Y,idx1)
    if(diff <= 0):
        for i in range(numpy.shape(Sh)[0]-diff+1,numpy.shape(Sh)[0]):
            SGCM =[]
            for j in range(len(X1)):
                SGCM.append(numpy.sqrt(numpy.square(X1[j] - Sh[i,0]) + numpy.square(Y1[j] - Sh[i,1])))      
            idx1 = SGCM.index(min(SGCM))
            S,s = moveSheperd(numpy.asarray([X[idx1],Y[idx1]]),destination,Pc,-1)
            #S[i,0] = S1[0]
            #S[i,1] = S1[1]  
    Sh = S    
    return Sh
    #print("QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ")      


def collectstrayseperate( Sh,X,Y,ra,Pd,Pc,destination,f,mindisShep,rhoash,rs,ang,pimultiple):
    print("In COLLECTSTRAYseparate ..........................")
    mode = 4
    XGCM = numpy.mean(X)
    YGCM = numpy.mean(Y)
    GCM = [XGCM,YGCM]
    D = []
    for i in range(len(X)):
        D.append(numpy.sqrt(numpy.square(X[i] - GCM[0]) + numpy.square(Y[i] - GCM[1])))
    #points = numpy.zeros(numpy.shape(Sh)[0] + 1)
    points1 = getcircularpoints( [XGCM,YGCM],max(D)+rs/2.0,numpy.shape(Sh)[0],ang,pimultiple)    
    points = numpy.zeros([numpy.shape(Sh)[0] + 1,2])
    #print("points befre resizing: " , points1 , " fffffffffffffffffffffffffffffff " , points )
    for i in range(numpy.shape(Sh)[0]):
        points[i,0] = points1[i,0]
        points[i,1] = points1[i,1]
    points[numpy.shape(Sh)[0],0] = points1[0,0]
    points[numpy.shape(Sh)[0],1] = points1[0,1]    
    #print("points after reszing: " , points , " gggggggggggggggggggggggggggggg")
    #points[numpy.shape(points)[0]][0] = points[0][0]
    #points[numpy.shape(points)[0]][1] = points[0][1]
    Sh1 = numpy.copy(Sh)
    order = []
    #midpoint = []
    for i in range(numpy.shape(points)[0] - 1):
        midpoint = [numpy.sum(points[i:i+2][0]),numpy.sum(points[i:i+2][1])]
        midpoint[0] = 0.5*midpoint[0]
        midpoint[1] = 0.5*midpoint[1]
        #midpoint[1] = 0.5*numpy.sum(points[i:i+2][1])
        dist = []
        #print("Sh1: " , Sh1 , " ffffffffffffffffffffffff midpt: " , midpoint )
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
            #print("S before singlecollectPos called :  " , Sh[order[i]] )
            #S = Sh[order[i]] 
            Sh[order[i]] = singlecollectingPosition(Sh[order[i]],last,XGCM,YGCM,Pd/4);
            #Sh[order[i]] = S 
            #print("AFTER FUNC CALLED: S = " ,  "  " , Sh[order[i]])
        else:
            idx1.append(i)
        
        if(idx1):
            dist = []
            for j in range(numpy.shape(Sh)[0]):
                dist.append(numpy.sqrt(numpy.square(Sh[j][0] - GCM[0]) + numpy.square(Sh[j][0] - GCM[1])))
            D = min(dist)
            #print("Sh: " , Sh[order[idx1[0]]], "  order: " , order , "  idx1: " , idx1 , "  movemovemovemovemovemove")
            for j in range(len(idx1)):
                Sh[order[idx1[j]]],s = moveSheperd(numpy.asarray(GCM).reshape(1,2),Sh[order[idx1[j]]].reshape(1,2),D,+1);

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
    print("MODE : " , mode)     
    return Sh,mode             
    #print("WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW")

def purepredator(Sh,X,Y,ra,Pd,Pc,destination,q,f,rhoash,rs,I):
    #print("In PUREPREDATOR ..................I = " , I , " " )
    pimultiple = (22.0/7.0)*q
    mode = 6
    Sh1 = numpy.copy(Sh)
    #currentmode = []
    #print("numpy.mean(X) " , numpy.mean(X) , "lenX: " , len(X))
    if(len(X) != 0):
        GCM = [numpy.mean(X),numpy.mean(Y)]
        rad = 60
        ang = numpy.arctan2(GCM[1]-destination[1],GCM[0]-destination[0])       
        #print("numpy.shape(Sh)[0] : " , numpy.shape(Sh)[0] , " GCM " , GCM)
        #print("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$")
        points = getcircularpoints(GCM,rad/2,numpy.shape(Sh)[0]+1,ang,q) #if Sh is array
        #points = getcircularpoints([0,0] , 10, 3, 0, 1)
        #print("points " , points  , " " , points[0,:])
        #print("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@")
        #if Sh is a list
        #points = getcircularpoints(GCM,rad/2,len(Sh[0])+1,ang,q)         
        if(len(I) == 0):
            I = []
            #midpoint = numpy.zeros([numpy.shape(Sh)])          
            #print("midpoint before for loop " , midpoint , " " )
            for i in range(numpy.shape(Sh)[0]):
                midpoint = [numpy.sum(points[i:i+2,0]),numpy.sum(points[i:i+2,1])]
                #print("midpoint before getting mul by 1/2: " , midpoint)
                if(numpy.shape(Sh)[0] > 1):
                    midpoint[0] = 0.5*midpoint[0]
                    midpoint[1] = 0.5*midpoint[1]   
                #print("midpoint " , midpoint)
                #midpoint = 0.5*[numpy.sum(points[i:i+1,0]),numpy.sum(points[i:i+1,1])]
                #midpoint[i][1] = 0.5*numpy.sum(points[i:i+1,1])                    
                #print("######################################################")
                D = []
                for j in range(numpy.shape(Sh)[0]):
                    D.append(numpy.sqrt(numpy.square(midpoint[0] - Sh1[j][0]) + numpy.square(midpoint[0] - Sh1[j][1])))
                idx = D.index(min(D))
                Sh1[idx,0] = -1000
                Sh1[idx,1] = -1000
                #print("D: " , D  , " idx: " , idx , "rororrorororrororrorroro") 
                I.append(idx)                
        #print("numpy.shape(Sh)[0] " , numpy.shape(Sh)[0] , " I= " , I)       
        for i in range(numpy.shape(Sh)[0]):
            p = path.Path([(GCM[0],GCM[1]),(points[i,0],points[i,1]),(points[i+1,0],points[i+1,1])])
            #print(numpy.shape(X) , " kkkkkkkkkkkkkkkk")
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
            #print("this is the path: " , p)
            #print("X: " , pts[0][0] , " " ,pts[1][0] , " " , pts[0][1] , " " , pts[1][1] , " lolololooololololo")
            IN = p.contains_points(pts)
            #print("INNNNNNNNNNNNNNNNNN " , IN)
            idx = numpy.where(IN == True)
            #print(idx[0])
            #print("sssssssssssssssssssssssssssss")
            Xin = []
            Yin = []
            for j in (idx[0]):
                Xin.append(X[j])
                Yin.append(Y[j])
            DGCM = []
            for j in range(numpy.shape(Xin)[0]):
                DGCM.append(numpy.sqrt(numpy.square(Xin[j] - numpy.mean(Xin)) + numpy.square(Yin[j] - numpy.mean(Yin))))
            idx = []
            #print("DGCM" , DGCM , " " ,numpy.shape(Xin)[0])
            for j in range(numpy.shape(Xin)[0]):
                if(DGCM[j] > 0.5*f):
                    idx.append(j)   
            #MANGE or ASK ABOUT I:
            #print("I " , I , " i[o] " , I[0] , Sh)
            #print("Sh : " , Sh[I[1]] , " " , Sh[I[1],:])                    
            if(len(idx) == 0 and len(Xin) != 0):
                #print("in if, prior to calling singleherding : i= " , i )
                #print(" I[i]: " , I[i] , " Sh[I[i]] " , Sh[I[i]] , " ", type(Sh[I[i]]))
                #print("Xin " , Xin )
                #print(" Yin " , Yin)
                #print("Pc " , Pc , " destination " , destination)
                #changing Sh[I[i]] to S below to use in stepspertimestep
                S = Sh[I[i]]
                #Sh[I[i]] = singleherdingPosition( Sh[I[i]],Pc,Xin,Yin,destination)
                #print("Going in singleherdingPosition: " , Sh[I[i]])
                Sh[I[i]] = singleherdingPosition( Sh[I[i]].reshape(1,2),Pc,Xin,Yin,destination)
                S = Sh[I[i]]
                #print("New Pos " , Sh[I[i]])
                #Sh[I[i]],direct = stepspertimestepSheperd(S,Sh[I[i]],1)
                #currentmode[I[i]] = 1 
            else:
                if(len(Xin) != 0):
                    #print("Going in singlecollectingPositionWithin")
                    mindisShep = 6
                    #mode = 0
                    #print("Going in singlecollectingPositionwithin, " , Sh[I[i]])
                    Sh[I[i]] = singlecollectingPositionwithin( Sh[I[i]].reshape(1,2),0.25*Pd,Xin,Yin,[numpy.mean(Xin), numpy.mean(Yin)])     
                    #currentmode[I[i]] = 1 
                    #collectstrayseperate(Sh,X,Y,ra,Pd,Pc,destination,f,mindisShep,rhoash,rs,ang,pimultiple)
                    #S = movestrayseperate(Sh,X,Y,ra,Pd,Pc,destination,q,mindisShep,rhoash,rs,I,f)
                    #Sh,direct = stepspertimestep(S,Sh,1) #sheep speed = 1            
    #print("PurePredator Ending with Sh: " , Sh)            
    #print("PPPPPPPPPPPPPPPPPPPPPPPPPPPPP")
    return Sh , I

def sheperdMovement( Sh,X,Y,ra,Pd,Pc,destination,f,mindisShep,rhoash,alpha,ang,mode,nin,ds,rs,method,predmasterslave,decrad,q,I):
    #print("Sh: " , Sh , " YYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYY")
    XGCM = numpy.mean(X)
    YGCM = numpy.mean(Y)
    #Sh1 = numpy.copy(Sh)
    Sh1 = numpy.copy(Sh)
    GCM = [XGCM,YGCM]
    X1 = copy.deepcopy(X)
    Y1 = copy.deepcopy(Y)
    #currentmode = []
    i = 1
    if(mode!=5 and mode!=6):
        #print("NOT ENTERING HERE >>>>>>>>>>")
        I = []
    #Below function not converted   
    if(numpy.shape(Sh)[0] >= len(X)):
       herdingPosition(Sh,Pc,X1,Y1,destination)
    if(numpy.shape(Sh)[0] == 1):
        #print("IN singlesheperdMovement  ..................." , Sh)
        #print("X: " , X)
        Sh = singlesheperdMovement(Sh,X1,Y1,ra,Pd,Pc,destination,f)        
        #print("AFTER EFFECTS of above func: "  ,Sh)
        #Sh = Sh.reshape(1,2)
        #Sh1 = Sh.reshape(1,2)
        #Sh1[0,0] = 1000
        #print("Let's see if Sh1 affects Sh: " , Sh , " AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA")
        #IT DOES CHANGE
    elif(mode == 4):
        #print("IN MODE = 4 " ) 
        ang = 0
        pimultiple = 2
        Sh , mode = collectstrayseperate( Sh,X1,Y1,GCM,Pd,Pc,destination,f,mindisShep,rhoash,rs,ang,pimultiple)
        #print("In ShpMovement after CollStraySeparate: " , Sh)
        if(predmasterslave==1):
            mode = 5
        elif(predmasterslave==3):
            mode = 6
        elif(predmasterslave==2 and mode == 1):
            mode = 1
    elif(mode == 6):
        Sh, I = purepredator(Sh,X1,Y1,ra,Pd,Pc,destination,q,f,rhoash,rs,I)
        #print("in SHPMOvement after PUREPREDATOR " , Sh)
    elif(mode == 5):
        Sh = movestrayseperate(Sh,X1,Y1,ra,Pd,Pc,destination,q,mindisShep,rhoash,rs,I);
    else:
        DGCM = []
        idx = []
        for i in range(len(X)):
            DGCM.append(numpy.sqrt(numpy.square(X[i] - XGCM) + numpy.square(Y[i] - YGCM)))
            if(DGCM[i] > f):
                idx.append(i)                                       
        if(len(idx) <= len(X) and mode == 1):
            #print("entering surround EFFect ")
            mode = 1
            alpha = 1
            ang = 0 
            Sh = surroundEffect(Sh,X,Y,ra,Pd,destination,q,mindisShep,rhoash,rs);
        elif(len(idx) > numpy.shape(Sh)[0] or mode == 2):
            mode = 2
            Sh = surroundEffectCollector(Sh,X1,Y1,ra,Pd,Pc,destination,i,mindisShep,rhoash,alpha,ang);
            D1 = []
            #print("D1 found with sh and sh1: ")
            #for i in range(1,10):
            for j in range(numpy.shape(Sh)[0]):
                D1.append(numpy.sqrt(numpy.square(Sh[j][0] - Sh1[j][0]) + numpy.square(Sh[j][1] - Sh1[j][1])))
            p = path.Path([[Sh[j][0],Sh[j][1]] for j in range(numpy.shape(Sh)[0])])
            Xt = numpy.transpose(X1)                
            Yt = numpy.transpose(Y1)
            pts = numpy.zeros([numpy.shape(Xt)[0],2])
            for j in range(len(Xt)):
                pts[j][0] = Xt[j]
                pts[j][1] = Yt[j]
            IN = p.contains_points(pts)
            IN1 = numpy.where(IN == True)
            #print("IN1: " , IN1 , " p : " , p , "sum(D1): ", sum(D1))   
            IN1 = len(IN1[0])
            if((sum(D1) < 10 or IN1 == len(X1)) and method == 1):
                #print("MODE CHANGES TO 4 ")
                mode = 4
            elif(method == 2):
                D = []
                for i in range(numpy.shape(Sh)[0]):
                    D.append(numpy.sqrt(numpy.square(Sh[i][0] - Sh1[i][0]) + numpy.square(Sh[i][1] - Sh1[i][1])))
                #print("In SheperdMovement D = " , D , " sum(D): " , sum(D))
                if(sum(D) < 10):
                    alpha=decrad*alpha
                    dist = []
                    for i in range(numpy.shape(Sh)[0]):
                        dist.append(numpy.sqrt(numpy.square(Sh[i][0] - XGCM) + numpy.square(Sh[i][1] - YGCM)))
                    D = max(dist)
                    if(D < rs):
                        mode = 1
                    ang = ang + 0.5
    #print("numpy.shape(Sh)[0]: " , numpy.shape(Sh) , "  " , Sh , " ", Sh1, " 111111111111111111")                 
    #if(numpy.shape(Sh)[0] == 1):
        #Sh,direction = stepspertimestepSheperd(Sh,Sh1,ds);                        
    #else:    
    #for i in range(numpy.shape(numpy.asarray(Sh))[0]):
    Sh,direction = stepspertimestepSheperd(Sh,Sh1,ds);   #ds = 1.5                      
    #print("Sh after last for loop : " , Sh , " mode " , mode)    
    return Sh,mode,I

def singlesheperdMovement(S,X,Y,ra,Pd,Pc,destination,f):
    #print("singlesheperdMovement in Sh : " , S , "  " , numpy.shape(X) , " |||||||||||||||||")
    D = numpy.zeros(numpy.shape(X)[0])
    idx = []
    l = numpy.shape(X)
    for i in range(l[0]):
        D[i] = numpy.sqrt(numpy.square(X[i] - S[0][0]) + numpy.square(Y[i] - S[0][1]))
        #print(i, " D[i] " , D[i] )
        if(D[i] < ra):
            idx.append(i)
    #currentmode = 2 
    #print("idx should be empty: " , idx , " XXXXXXXXXXXXXXXXXXXXXXXXXXXx")
    if(len(idx) == 0):
        Xgcm = numpy.mean(X)
        Ygcm = numpy.mean(Y)
        Dgcm = []
        ind = []
        for i in range(l[0]):  # or len if X is a list and not an array     
            Dgcm.append(numpy.sqrt(numpy.square(X[i] - Xgcm) + numpy.square(Y[i] - Ygcm)))
            #print(i, " dgcm should have some: " , Dgcm[i])
            if(Dgcm[i] > f):
                ind.append(i)
        print("GCM: ", [Xgcm, Ygcm])        
        if(len(ind) == 0):
            print("singleherding: " , S , " HHHHHHHHHHHHHHHHHHHH")
            S = singleherdingPosition(S,Pc,X,Y,destination)
            #print("After HERDING: aka new value " , S , " hhhhhhhhhhhhhhhhhhhhhh" )
            #currentmode = 1;            
        else:
            pos = Dgcm.index(max(Dgcm))
            last = [X[pos],Y[pos]]
            print("singlecollecting: " , S , " CCCCCCCCCCCCCCCCCCCCC")
            S = singlecollectingPosition(S,last,Xgcm,Ygcm,Pd)
            #print("AFTER singlecollectingPosition: " , S , " ccccccccccccccccc")
            #currentmode = 2             
    return S
            
def singleherdingPosition(S,Pc,X,Y,destination):
    #print("In HERDING " , S)
    #S2 = numpy.copy(S)
    #S2 = [0,0]
    l = numpy.shape(X)
    #Dgcm = numpy.zeros(numpy.shape(X))
    Dgcm = []
    #print("XXXXXXXXXXXX " , numpy.shape(X))
    for i in range(l[0]):  # or len if X is a list and not an array     
        Dgcm.append(numpy.sqrt(numpy.square(X[i] - destination[0]) + numpy.square(Y[i] - destination[1])))
    #idx = numpy.argmax(Dgcm,axis = 0)
    if(len(Dgcm) != 0): # NEW ADDITION , ................
        pos = Dgcm.index(max(Dgcm))
    idx = []
    for i in range(len(Dgcm)):
        if(Dgcm[i] == Dgcm[pos]):
            idx.append(i)
    #print("X and Y at POS: " , X[pos] , " " , Y[pos] , " " , S[0] , " ",S[0,0])
    if(len(idx) != 0):
        agent = numpy.zeros(2)
        agent[0] = X[pos]
        agent[1] = Y[pos]
        #agent = numpy.asarray([X[pos],Y[pos]])
        #print("agent " , agent)
        #S1 = S
        S[0,0] = agent[0]
        #S[0][1] = Y[pos]
        #print("Entering Move pt1: " , S , " pt2: " , destination , " " , agent)
        S,s2 = moveSheperd(agent,destination[:],Pc,-1) #s2 is useless
        #S,s2 = move(S,S,0,0)
        #S = S1
        #print("S after move : " , S ,"  ", )
        #print("hhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhh ") 
    #S,s2 = stepspertimestepSheperd(S,S2,1.5)   
    #numpy.asarray(S1)
    #print("S after limiting its steps: " , S , " ssssssssssssssssssssssssssss " )
    return S

def singlecollectingPosition( S,last,XGCM,YGCM,Pd):
    #print("last = : " , last , " XGCM: " , XGCM , " YGCM: " , YGCM , " numpy.asarray([XGCM,YGCM]) " , numpy.asarray([XGCM,YGCM]).reshape(2,1))
    #print("in singlecollecting: " , type(S) , "  " , S)
    G = numpy.zeros(2)
    G[0] = XGCM
    G[1] = YGCM
    #print("G; " , G , " GGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGG")
    l,s = moveSheperd(numpy.asarray(last),G,Pd,-1)
    #print("l = " , l , " llllllllllllllllllllllllll")
    #l = l.reshape(1,2)
    S,s = moveSheperd(S,l,1,1)
    #S = S.reshape(1,2)
    #print("INSIDE Func singlecollecting: " , S , )
    #print("cccccccccccccccccccccccccccccccccccccc") 
    return S


# def stepspertimestepSheperd(currentpoints,initialpoints,steps): # to limit the step taken per unit time
# #steps: steps to be taken towards currentpoints from initialpoints
#     difference=currentpoints-initialpoints
#     norm=numpy.linalg.norm(difference,axis=0)
#     l=numpy.shape(difference)
#     #print("in stepper  L : " , l , " " , norm, " llolololooloolooololo " , currentpoints)
#     direction=numpy.zeros((2,1))
#     #for i in range(1):
#     step=steps
#     if norm<steps:
#         step=norm
#     if norm==0:
#         unitvec=[0,0]
#     else:
#         unitvec=[difference[0]/norm,difference[1]/norm]
#         direction[0]=unitvec[0]
#         direction[1]=unitvec[1]
#     currentpoints[0] = currentpoints[0]+unitvec[0]*step
#     currentpoints[1] = currentpoints[1]+unitvec[1]*step
#     #print("currentpoints " , currentpoints , " poppppopopopopo")
#     return currentpoints,direction

def stepspertimestepSheperd(currentpoints,initialpoints,steps): # to limit the step taken per unit time
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

def moveSheperd( point1,point2,intensity,direction ): 
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
    #print direction
    if numpy.size(direction)!=0:
        inertiapoints=initpoints+h*direction
    return inertiapoints

def inertialEffect( initpoints,direction,h ):
    inertiapoints=initpoints
    #print direction
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
        #print("inititpoints: " , initpoints[:,1:10])
        #print("init.T ", initpoints.T[1:10] ,)
        #print(" D in cluster: " , D[1:10]);
        #print("CCCCCCCCCCCCCCCCCCCCCCCCCCCCC")
        #print("")
        cl = [i for i in D if i < rs]
        #print("cl: ")
        #print(cl)
        #print("CLCLCLCCLCLCLCCLCLCCLCLCLLL")
        closeagents=numpy.where((D<(rs)))  # Check why this is happening
        #print("closeagents: " , closeagents, " " , numpy.size(closeagents))
        for i in range(numpy.size(closeagents)):
            #print("FFFFFFFFFFFFFFFFFFFFFFFFFFFFFF")
            agent1=initpoints[:,closeagents[0][i]] 
            D=numpy.linalg.norm(initpoints.T-agent1,axis=1)
            sortedD=numpy.argsort(D)
            orderedD = numpy.sort(D)
            #indices of all the nieghbours within 5m of an agent
            D_less_than_10 = numpy.where((orderedD < 10))
            #n = len(D_less_than_10[0])
            #n = 62
            # if(swarm_split != 1):
            #     n = len(D_less_than_10[0])
                

            #     #print
            # else:
            #     #print("clustering: n: " ,n)    
            #     D_less_than_5 = numpy.where((orderedD < 5.0))
                #n = len(D_less_than_5[0])
            #sortedDindex=numpy.where((sortedD<=3))
            #if(len(D_less_than_10) < 5):
                #n = len(D_less_than_10)
            #else:
                #n = 5    
            #if(toggle == 0):
            #sortedDindex=numpy.where((sortedD <= n)) #nearest neighbours which are inside a radius with center as the agent
            #n = floor(0.9*40)
            #sortedDindex=sortedD[:n]
            if(swarm_split == 0):
                #10% of the total
                sortedDindex=sortedD[1:3] # 1+int(N*0.1)   taking start point as 1 coxz we dont want to include the agent itself
            else:
                sortedDindex=sortedD[1:1+n]
            sortedDindex = numpy.asarray(sortedDindex).reshape(1,len(sortedDindex))
            print("sortedD: " , sortedDindex )
            #print("orderedD: " , orderedD)
            #print("D_less_than_10: ", len(D_less_than_10) )
            #sortedDindex=numpy.where(())
            agent2=(initpoints[:,sortedDindex[0]]).mean(1)
            # print("())))))))))))))))))))))))))))))))))))))))))))")
            # print(closeagents)
            # print(orderedD)
            # print(D_less_than_10)
            # print(n)
            # print(sortedDindex)
            # print(sortedDindex[0])
            # print("agent2: " , agent2 )
            # print("initpoints[:,sortedDindex[0]] : " ,initpoints[:,sortedDindex[0]])
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
        NS,shepherdingpoints[:,closeagents[0]]=move(S,shepherdingpoints[:,closeagents[0]],rhos,-1)
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

def sheepmovements(initpoints,n,c,rs,Shepherd,ra,rhoa,rhos,unitvec,h,e,dps,mode,destination,p,swarm_split,destination_other):
    copypoints = copy.deepcopy(initpoints)
    if(n != 1):
        #if(swarm_split == 1):
        initpoints = clusteringEffect(initpoints,n,c,rs,Shepherd,swarm_split)
    idx1 = []
    X = initpoints[0,:]
    Y = initpoints[1,:]
    for i in range(len(X)):
        if(destination[0] != destination_other[0] or destination[1] != destination_other[1]):
            dist1 = sqrt(pow(initpoints[0][i] - destination[0],2) + pow(initpoints[1][i] - destination[1],2))
            dist2 = sqrt(pow(initpoints[0][i] - destination_other[0],2) + pow(initpoints[1][i] - destination_other[1],2))    
            if(dist1 > 10 and dist2 > 10):
                idx1.append(i)
            # dist2 = sqrt(pow(initpoints[0][i] - destination_other[0],2) + pow(initpoints[1][i] - destination_other[1],2))    
            # if(dist2 > 10):
            #     idx1.append(i)
        else:
            dist1 = sqrt(pow(initpoints[0][i] - destination[0],2) + pow(initpoints[1][i] - destination[1],2))
            if(dist1 >= 10 + 5):
                idx1.append(i)        

    initpoints1 = sheperdingEffect(initpoints,Shepherd,rs,rhos)
    for i in range(len(initpoints[0])):
        if(i not in idx1):
            initpoints1[0][i] = initpoints[0][i]
            initpoints1[1][i] = initpoints[1][i]
        else:
            initpoints[0][i] = initpoints1[0][i]
            initpoints[1][i] = initpoints1[1][i]            

    #print("initpoints " , initpoints )
    #print("inipoints1 " , initpoints1)        
    #COMMENTED AS NO USE SEEMS TO BE PRESENT..................................
    # initpoints = abs(initpoints - initpoints1)
    # initpoints[0] = initpoints[0] + initpoints[1]
    # idx = []
    # for i in range(len(initpoints[0])):
    #     if(initpoints[0][i] != 0):
    #         idx.append(i)
    # #idx = idx.intersect(idx1)
    # idx.sort()
    # idx1.sort()
    # #print("idx: " , idx , " idx1: " , idx1 , " " , len(idx) , " " , len(idx1))
    # j = 0; k = 0;
    # temp =[]
    # while(j < len(idx) and k < len(idx1)):
    #     if(idx[j] == idx1[k]):
    #         temp.append(j)
    #         j = j+1; k=k+1;
    #     elif(idx[j] > idx1[k]):
    #         k = k + 1
    #     else:
    #         j = j + 1
    
    #print("before intertial initpoints")
    #print(initpoints)        
    #if(len(idx1)!=0 and len(unitvec)!= 0): #changed idx to idx1......
    #initpoints1[:,idx1] = inertialEffect(initpoints1[:,idx1],unitvec[:,idx1],h)
    initpoints = inertialEffect(initpoints,unitvec,h)
    #initpoints = initpoints1
    #print("AFTER IT")
    #print(initpoints)
    #r = numpy.random.randint(len(X))
    #for i in range()    
    initpoints = angularnoiseEffect(initpoints,e,p)

    #initpoints[:,idx1] = selfRepellingEffect(initpoints[:,idx1],ra,rhoa)
    initpoints = selfRepellingEffect(initpoints,ra,rhoa) #repelling for all TIMES!!

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

def position_husky5(data):
    global husky_predX, husky_predY, Yaw_pred
    husky_predX[1] = data.pose.pose.position.x
    husky_predY[1] = data.pose.pose.position.y
    husky_orientation = data.pose.pose.orientation

    orientation_list = [husky_orientation.x,husky_orientation.y,husky_orientation.z,husky_orientation.w]
    (roll,pitch,yaw) = euler_from_quaternion(orientation_list)
    Yaw_pred[1] = yaw

def position_husky4(data):
    global husky_predX, husky_predY, Yaw_pred
    husky_predX[0] = data.pose.pose.position.x
    husky_predY[0] = data.pose.pose.position.y
    husky_orientation = data.pose.pose.orientation

    orientation_list = [husky_orientation.x,husky_orientation.y,husky_orientation.z,husky_orientation.w]
    (roll,pitch,yaw) = euler_from_quaternion(orientation_list)
    Yaw_pred[0] = yaw    

def position_husky0(data):
    global huskyX, huskyY, Yaw
    huskyX[0] = data.pose.pose.position.x
    huskyY[0] = data.pose.pose.position.y
    husky_orientation = data.pose.pose.orientation

    orientation_list = [husky_orientation.x,husky_orientation.y,husky_orientation.z,husky_orientation.w]
    (roll,pitch,yaw) = euler_from_quaternion(orientation_list)
    Yaw[0] = yaw

def position_husky1(data):
    global huskyX, huskyY, Yaw
    huskyX[1] = data.pose.pose.position.x
    huskyY[1] = data.pose.pose.position.y
    husky_orientation = data.pose.pose.orientation

    orientation_list = [husky_orientation.x,husky_orientation.y,husky_orientation.z,husky_orientation.w]
    (roll,pitch,yaw) = euler_from_quaternion(orientation_list)
    Yaw[1] = yaw

def position_husky2(data):
    global huskyX, huskyY, Yaw
    huskyX[2] = data.pose.pose.position.x
    huskyY[2] = data.pose.pose.position.y
    husky_orientation = data.pose.pose.orientation

    orientation_list = [husky_orientation.x,husky_orientation.y,husky_orientation.z,husky_orientation.w]
    (roll,pitch,yaw) = euler_from_quaternion(orientation_list)
    Yaw[2] = yaw

def position_husky3(data):
    global huskyX, huskyY, Yaw
    huskyX[3] = data.pose.pose.position.x
    huskyY[3] = data.pose.pose.position.y
    husky_orientation = data.pose.pose.orientation

    orientation_list = [husky_orientation.x,husky_orientation.y,husky_orientation.z,husky_orientation.w]
    (roll,pitch,yaw) = euler_from_quaternion(orientation_list)
    Yaw[3] = yaw

def position_husky6(data):
    global huskyX, huskyY, Yaw
    huskyX[4] = data.pose.pose.position.x
    huskyY[4] = data.pose.pose.position.y
    husky_orientation = data.pose.pose.orientation

    orientation_list = [husky_orientation.x,husky_orientation.y,husky_orientation.z,husky_orientation.w]
    (roll,pitch,yaw) = euler_from_quaternion(orientation_list)
    Yaw[4] = yaw

def position_husky7(data):
    global huskyX, huskyY, Yaw
    huskyX[5] = data.pose.pose.position.x
    huskyY[5] = data.pose.pose.position.y
    husky_orientation = data.pose.pose.orientation

    orientation_list = [husky_orientation.x,husky_orientation.y,husky_orientation.z,husky_orientation.w]
    (roll,pitch,yaw) = euler_from_quaternion(orientation_list)
    Yaw[5] = yaw        

def main():
    rospy.init_node('sheep_sheperd_u', anonymous=True)
    #pub = rospy.Publisher('chatter', String, queue_size=10)
    #sub = rospy.Subscriber("chatter",String,callback)

    #k = 35 #Spreading the swarm
    pub_marker = rospy.Publisher('multi_sheep_sheperd200', MarkerArray , queue_size=1)
 
    pub_vel_husky0 = rospy.Publisher('/Husky0/husky_velocity_controller/cmd_vel',Twist, queue_size=10)
    pub_vel_husky1 = rospy.Publisher('/Husky1/husky_velocity_controller/cmd_vel',Twist, queue_size=10)
    pub_vel_husky2 = rospy.Publisher('/Husky2/husky_velocity_controller/cmd_vel',Twist, queue_size=10)
    pub_vel_husky3 = rospy.Publisher('/Husky3/husky_velocity_controller/cmd_vel',Twist, queue_size=10)
    #pub_vel_husky6 = rospy.Publisher('/Husky6/husky_velocity_controller/cmd_vel',Twist, queue_size=10)
    #pub_vel_husky7 = rospy.Publisher('/Husky7/husky_velocity_controller/cmd_vel',Twist, queue_size=10)
    pub_vel_husky_pred0 = rospy.Publisher('/Husky4/husky_velocity_controller/cmd_vel',Twist, queue_size=10)
    pub_vel_husky_pred1 = rospy.Publisher('/Husky5/husky_velocity_controller/cmd_vel',Twist, queue_size=10)

    # rospy.Subscriber('/Husky4/base_pose_ground_truth', Odometry, position_husky4)
    # rospy.Subscriber('/Husky5/base_pose_ground_truth', Odometry, position_husky5)
    # rospy.Subscriber('/Husky1/base_pose_ground_truth', Odometry, position_husky1)
    # rospy.Subscriber('/Husky2/base_pose_ground_truth', Odometry, position_husky2)
    # rospy.Subscriber('/Husky3/base_pose_ground_truth', Odometry, position_husky3)
    # rospy.Subscriber('/Husky0/base_pose_ground_truth', Odometry, position_husky0)
    #rospy.Subscriber('/Husky6/base_pose_ground_truth', Odometry, position_husky6)
    #rospy.Subscriber('/Husky7/base_pose_ground_truth', Odometry, position_husky7)

#Final Destinations: 
#('dest1: ',)
    destination1x = [-30 , -71. ,-108. ,-140. , -63. , -82. , -78. ,-116. , -63. , -96., -119. , -71.,  -109. , -62. ,-105. , -91. ,-142. , -68. ,-128. ,-133. ,-136. , -80. ,-128. , -59.,
  -130.]
    destination1y =  [  0.,  108. ,-124.,  -24. , 227. ,  53., -185.,   78.,   90.,   55., -192. ,  68.,
  -167., -147. , -91. , 173. ,  34. , 105. , -22., -146. , -95.  , -5. , 225. , 205.,
   -44.]

#('dest2: ',)
    destination2x = [ 30. , 198. , 169. , 218.,  179. , 171.,  223.,  187.,  240. , 212. , 232. , 239.,
   196. , 248. , 176.  ,243. , 157. , 220. , 179. , 151. , 197. , 196. , 164. , 240.,
   230.]
    destination2y = [ 0. ,-107., -228.,  234., -157., -223. ,-209. ,-141. , 145. , 138. ,  37. ,-162.,
    67. ,  80. , 249. ,  48. ,  42. ,  62.,  103. , 125. ,-151.,  186. , 187., -222.,
  -100.]

#('dest: ',)
    destinationx = [ 100.  , 16. , -57.,   33. , -10. ,  54. ,  11. ,  72.,    7. , -92. ,  47. , -85.,
   -56.  , 40. , -24. , -26. ,  15. ,   7. , -71. ,  -8.,   61. ,  57. ,  21.,  -38.,
    74.]
    destinationy = [  0. , 238. , 159.,  -76. , -20. ,-209. ,  31. , -59. ,-171.  , -6. ,-180. , -20.,
   144., -230. ,  86. ,  69., -127., -119. , -48., -211. , 120. ,  36. , 231.,  -82.,
   142.]
    
    rate = rospy.Rate(10) # 10hz
    MC = len(destinationy)
    MC = 1;
    start_time = numpy.zeros((1,MC))
    stop_time = numpy.zeros((1,MC))
    split_time = numpy.zeros((1,MC))
    diff = numpy.zeros((1,MC))
    dest1_arr = numpy.zeros((2,MC))
    dest2_arr = numpy.zeros((2,MC))
    dest_arr = numpy.zeros((2,MC))
    split_diff = numpy.zeros((1,MC))
    agent_sep = numpy.zeros((1,MC))
    not_completed = numpy.zeros((1,MC))
    task = numpy.zeros((1,MC))
    # mc_dest1x = [ -69., -67.]
    # mc_dest1y = [ -10., -32.]
    # mc_dest2x = [ 69.,   5.]
    # mc_dest2y = [ 35.,  48.]
    # mc_destx = [  36.,  37.]
    # mc_desty = [  -47.,  18.]    

    for mc1 in range(MC):            
        e=0.3 # strength of error movement
        p=0.05 # probability of an error
        Xstartpos=0 #swarm initial position
        Ystartpos=0
        h = 0.5 #strength of inertial force
        
        N = 20 # number of agents
        
        N1 = copy.deepcopy(N)
        ra=3.0  # distance for self repulsion
        mindisShep = 6
        n=int(numpy.ceil(0.35*N1))
        #n = 2;
        #n = (N*90)/100 #as we'll have two groups after this
        Pd = ra*log(N)
        Pc = ra
        threshold = 10.0
        dps=1 # speed of agents
        
        f = N^(1/2)
        rhod = 3 #STRENGTH of  moving towards destination
        rhoa=2.0# strength of self repulsion 
        rhos=1.0#strength of predatory repulsion 
        q = 1.8
        #I = []
        rhoash = 1.0
        #Shepherd=[-0,100]

        rs = 20.0  #radius of influence
        c=1.05 #strength of clustering
        
        nod = 3 #no. of destination
        #destination=[0,-45] #destination
        #destination1 = [-50,-10]
        #destination2 = [50,-10]
        mode1 = 2;
        mode2 = 2
        alpha = 1
        ang = 0
        mode = 2
        nin = 0 
        method = 1 
        method1 = 1
        method2 = 1
        predmasterslave = 3 
        predmasterslave1 = 3
        predmasterslave2 = 3
        ds = 1.5
        decrad = 0.9
        I = []
        I1 = []
        I2 = []

        x=Xstartpos+numpy.random.rand(1,N) #swarm initialization
        y=Ystartpos+numpy.random.rand(1,N)  #gives random values from [0,1)
        initpoints=numpy.concatenate((x,y),axis=0)
        direction=numpy.zeros((2,N))
        unitvec=numpy.zeros((2,N))
        initpoints[:,0] = [-4.5,0.0]
        initpoints[:,1] = [-3.5,0.0]
        initpoints[:,2] = [4.5,0.0]
        initpoints[:,3] = [3.5,0.0]
        initpoints[:,4] = [-3.5,3.0]
        initpoints[:,5] = [3.5,-3.0]
        #S1 = [-1.5,100.0]
        S1 = [0.0, 25.0]
        #S2 = [1.5,-20.0]
        S2 = [0.0, -25.0]
        S3 = [-2.5,100]
        S4 = [2.5,-20.0]
        nos = 2 #Number of sheperd
        #Sh = numpy.asarray([S1,S2,S3,S4]).reshape(nos,2)
        #Sh = numpy.asarray([S1,S2,S3]).reshape(nos,2) #Sheperd Array    
        #Sh = numpy.asarray(S1).reshape(nos,2)
        Sh = numpy.asarray([S1,S2]).reshape(nos,2)

        H = [Twist(), Twist(), Twist(), Twist(), Twist(), Twist()]
        H_pred = [Twist(), Twist()]
        H1 = Twist()
        H2 = Twist()
        H3 = Twist()
        H4 = Twist()
        H0 = Twist()
        M = MarkerArray()    
        #x = []
        #y = []

        nod = 3 #no. of destination
    
        destination = [destinationx[mc1],destinationy[mc1]]
        destination1 = [destination1x[mc1],destination1y[mc1]]
        destination2 = [destination2x[mc1],destination2y[mc1]]


        l = 0;
        shape = Marker.CUBE;
        for i1 in range(N+nos+nod): #to include shepherd too
            marker = Marker()
            #print("i1 " , i1)
            marker.header.frame_id = "/multi_sheep_sheperd200";
            marker.header.stamp = rospy.get_rostime();
            

            marker.ns = "mul_shp";
            
            marker.id = i1;
            
            marker.type = shape;
            
            marker.action = Marker.ADD;

            if(i1 < N):
                marker.pose.position.x = x[0][i1];            

                marker.pose.position.y = y[0][i1];
            
            marker.pose.position.z = 0;

            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            
            marker.scale.x = 1.0;
            marker.scale.y = 1.0;
            marker.scale.z = 1.0;
            
            #marker.color.r = i1/10.0;
            marker.color.r = 0.0;
            #marker.color.g = 2*i1/10.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;
       
            marker.lifetime = rospy.Duration();

            if(i1 >= N and i1<N+nos):
                marker.ns = "sheperd"
                marker.pose.position.x = Sh[i1-N][0];
                marker.pose.position.y = Sh[i1-N][1];
                #marker.color.r = 2*i1/10.0;
                marker.color.r = 1.0;
                marker.color.g = 0.0;
                #marker.color.b = i1/5.0;
                marker.color.b = 0.0;

            #marker.color.r = 0.0;    
            if(i1 >= N+nos):
                
                marker.ns = "Point"
                marker.type = Marker.CYLINDER
                marker.scale.x = 2*threshold;
                marker.scale.y = 2*threshold;
                if(l == 0):
                    marker.pose.position.x = destination1[0]
                    marker.pose.position.y = destination1[1]
                    #marker.pose.position.x = -128
                    #marker.pose.position.y =  225                    
                    l = 1
                elif(l == 1):
                    marker.pose.position.x = destination2[0]
                    marker.pose.position.y = destination2[1]
                    #marker.pose.position.x = 164
                    #marker.pose.position.y = 187                    
                    marker.scale.x = 2*threshold;
                    marker.scale.y = 2*threshold;                    
                    l = 2 
                else:
                    marker.pose.position.x = destination[0]
                    marker.pose.position.y = destination[1]
                    #marker.color.r = 0.8
                    marker.scale.x = 2*(threshold+5);
                    marker.scale.y = 2*(threshold+5);
                marker.color.b = 1.0;
                

                marker.scale.z = 0.5;   
            #x.append(marker.pose.position.x)
            #y.append(marker.pose.position.y)


            M.markers.append(marker)
            #print("id = " , M.markers[i1].id , "ini pos " , x[0][i1%N] , " , " , y[0][i1%N])

            m=numpy.size(initpoints[0])
            #print("this will be the new N " , m)
        #for i in range(3):
            #M.markers[i].action = marker.ADD
        count = 0;    
        flag = 0;

        outthreshold = []
        for i in range(N):
            outthreshold.append(i)

        s = 0.25    
        toggle = 0; #for testing made toggle as 1, will change back to 0 and comment the grp1 anf grp2 thing
        grp1 = [] ; grp2 = []
        # for j in range(N):
        #     if(j < N/2):
        #         grp1.append(j)
        #     else:
        #         grp2.append(j)    
        switch = 1; # change back to 1
        crossed_swarm = 0;
        enter_once = 0; enter_once2 = 0;
        swarm_split = 0;
        out1 = 0; out2 = 0; out3 = 0;
        in1 = 0; in2 = 0; 
        eating1 = 0 ; eating2 = 0; eaten3 = 0;
        pos_update1 = 0; pos_update2 = 0
        new_pos1 = [0,0] ; new_pos2 = [0,0]
        entered1 = 0; entered2 = 0;
        #print("m1: ", mc1 , " ", rospy.get_rostime())
        outthreshold1 = outthreshold ; outthreshold2 = outthreshold
        inthreshold1 = []; inthreshold2 = [];
        dest3 = [0,0]
        too_close = 0;
        
        sheperd_speed = numpy.zeros(nos);
        delta_theta_shepherd = numpy.zeros(nos);
        change_in_heading_shepherd = numpy.zeros(nos)

        speed_sheep = numpy.zeros(N);
        delta_theta = numpy.zeros(N)
        change_in_heading = numpy.zeros(N)
        dist_left_to_cover = numpy.zeros(N)
        k = 0.3; k_pred = 1.0 #1.5 times sheep
        start_time[0][mc1] = rospy.get_rostime().to_sec()

        # Sh00 = S1
        # Sh01 = S3
        # Sh10 = S2
        # Sh11 = S4
        while not rospy.is_shutdown():
            #hello_str = "hello world %s" % rospy.get_time()
            #rospy.loginfo(hello_str)
            #print("hello_str")
            #pub.publish(hello_str)
            #print(M.markers[0].id , " " , M.markers[1].id , "  " , M.markers[2].id , "tolo")
            pub_marker.publish(M)
            # pub_vel_husky1.publish(H[1])
            # pub_vel_husky2.publish(H[2])
            # pub_vel_husky3.publish(H[3])
            # pub_vel_husky0.publish(H[0])
            #pub_vel_husky6.publish(H[4])
            #pub_vel_husky7.publish(H[5])
            # pub_vel_husky_pred0.publish(H_pred[0])            
            # pub_vel_husky_pred1.publish(H_pred[1])
            #print(M.markers[0].id , " " , M.markers[1].id , "  " , M.markers[2].id)
            print("yolo")
            N=numpy.size(initpoints[0])
            Pd=ra*numpy.log(N1)
            Pc=ra
            f=ra*numpy.sqrt(N1)

            temp_Sh = copy.deepcopy(Sh)
            temp_initpoints = copy.deepcopy(initpoints)
            #plotpoints(initpoints,Shepherd,direction,rs)
            if(toggle == 0):
                #print("initpoints: " , initpoints)
                sheepmovements(initpoints,n,c,rs,Sh,ra,rhoa,rhos,unitvec,h,e,dps,mode,[1000,1000],p,swarm_split,[1000,1000])
                #print("ini after: " , initpoints)
            else:
                #print("initpoints: " , numpy.concatenate((x1,y1)) )
                #print([x1,y1])
                #print("QQQQQQQQQQQQQQQQQQQQQQQQQQQQQ")
                #print("GRP1 INFO: ")
                swarm_split = 1
                # print("grp1")
                # print(initpoints[:,grp1].T)
                # print("grp2")
                # print(initpoints[:,GRP2].T)
                #print(Sh[0])
                #print(destination1)
                #print(initpoints[0][grp1].T)
                
                # DD1 = numpy.linalg.norm(initpoints.T - Sh[0],axis = 1)
                # DD2 = numpy.linalg.norm(initpoints.T - Sh[1],axis = 1)
                # far_from1 = numpy.where(DD1 > 4*rs)
                # far_from2 = numpy.where(DD2 > 4*rs)
                # separated_agents = numpy.intersect1d(far_from1[0],far_from2[0])
                # grp1 = numpy.setdiff1d(grp1,separated_agents)
                # grp2 = numpy.setdiff1d(grp2,separated_agents)
                # print("grp1: " , grp1)
                # agent_sep[0][mc1] = numpy.size(separated_agents)                


                #g3 = numpy.intersect1d(ini)
                #n = n/2; #groups made 
                if(out1 == 0 or out2 == 0):
                    g1 = numpy.intersect1d(grp1,outthreshold1)
                    g2 = numpy.intersect1d(grp2,outthreshold2)                    
                    sheepmovements(initpoints,n,c,rs,Sh,ra,rhoa,rhos,unitvec,h,e,dps,mode1,destination1,p,swarm_split,destination2)
                
                    if(len(g1) >= 0.1*(numpy.size(grp1)) and eating1 == 0):
                        #print("GGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGG")
                        #print("g1: " , g1, " " , g1[0])
                        n11 = numpy.size(grp1)
                        f1 = ra*pow(len(grp1),0.5)
                        #print("f1: " , f1)
                        #if(n11 > 0):
                        Pd1 = ra*log(len(grp1))
                        #else:
                        #    Pd1 = 0    
                        #n = ceil(9*n11/10)
                        #print("Pd1 g1 : " , Pd1)
                        #sheepmovements(initpoints,n,c,rs,Sh,ra,rhoa,rhos,unitvec,h,e,dps,mode1,destination1,p,swarm_split)
                        #sheepmovements(initpoints[:,grp1],0.9*len(grp1),c,rs,Sh[0].reshape(1,2),ra,rhoa,rhos,unitvec,h,e,(dps/5.0),mode,destination1,p)
                        #S0,mode1,I1 = sheperdMovement(numpy.asarray([Sh[0],Sh[2]]).reshape(2,2),initpoints[0][g1],initpoints[1][g1],ra,Pd1,Pc,destination1,f1,mindisShep,rhoash,alpha,ang,mode1,nin,ds,rs,method1,predmasterslave1,decrad,q,I1)
                        #print("g1: " , g1)
                        S0,mode1,I1 = sheperdMovement(numpy.asarray([Sh[0]]).reshape(1,2),initpoints[0][g1],initpoints[1][g1],ra,Pd1,Pc,destination1,f1,mindisShep,rhoash,alpha,ang,mode1,nin,ds,rs,method1,predmasterslave1,decrad,q,I1)
                        print("")
                    else:
                        #EATING THE TASK STARTS HERE, THIS MEANS SHEPEHRD JUST STANDING STILL
                        #out1 = 1;
                        eating1 = 1;
                        m1 = numpy.arctan2((destination[1] - destination1[1]),(destination[0] - destination1[0]))
                        if(m1 > pi/2):
                            alpha1 = pi - m1;
                        else:
                            alpha1 = m1;
                        alpha1 = m1
                        if(entered1 == 0):        
                            dest1 = [0,0];
                            dest1[0] = destination1[0] - 8.0*numpy.cos(alpha1)
                            dest1[1] = destination1[1] - 8.0*numpy.sin(alpha1) #  - sin() as alpha1 is negative 
                            M1 = numpy.arctan2((destination[1] - dest1[1]),(destination[0] - dest1[0]))
                            entered1 = 1;
                        #print("UUU11111111111111111UUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUU")
                        #print("dest1: " , dest1 , " alpha1: " , alpha1)
                        cur_angle1 = numpy.arctan2((Sh[0][1] - destination1[1]),(Sh[0][0] - destination1[0]))

                        if not (Sh[0][0] <= dest1[0] and Sh[0][1] >= dest1[1]):
                            #print("dest1: " , dest1 , " Sh[0] " , Sh[0])
                        #if(numpy.size(neg_y1) != 0):
                            d1 = sqrt(pow(Sh[0][0] - destination1[0],2) + pow(Sh[0][1] - destination1[1],2))
                            d11 = sqrt(pow(dest1[0] - destination1[0],2) + pow(dest1[1] - destination1[1],2))
                            dr = d11 - d1;
                            #d1 = d1 + abs(cur_angle1)
                            #new_pos1[0] = destination1[0] + (d1+dr)*numpy.cos(cur_angle1 - pi/36)
                            #new_pos1[1] = destination1[1] + (d1+dr)*numpy.sin(cur_angle1 - pi/36)
                            S00 , a = moveSheperd(S0[0],numpy.asarray(dest1).reshape(1,2),1,1)
                            #S01 , a = moveSheperd(S0[1],numpy.asarray(dest1).reshape(1,2),1,1)
                            D10=numpy.linalg.norm(initpoints.T-S00,axis=1)
                            #D11=numpy.linalg.norm(initpoints.T-S01,axis=1)
                            if(numpy.where(D10 < rs)):
                                S00, a = moveSheperd(S0[0],numpy.asarray(destination1).reshape(1,2),1,-1)
                            # if(numpy.where(D11 < rs)):    
                            #     S01, a = moveSheperd(S0[1],numpy.asarray(destination1).reshape(1,2),1,-1)
                            S0[0][0] = S00[0][0]
                            S0[0][1] = S00[0][1]
                            #S0[1][0] = S01[0][0]
                            #S0[1][1] = S01[0][1]    

                    

                    if(len(g2) >= 0.1*(numpy.size(grp2)) and eating2 == 0):
                        #print("(((((((99999999999999999999999999999")
                        #print("GRP2")
                        #print("g2: " , g2)
                        #print("outthreshold2: " , outthreshold2)
                        n22 = numpy.size(grp2)
                        f2 = ra*pow(len(grp2),0.5)
                        #if(len(grp2) > 0):
                        Pd2 = ra*log(len(grp2))
                        # else:
                        #     Pd2 = 0;    
                        #n = ceil(9*n22/10)
                        #print("f2: " , f2 , " " , len(grp2) , " " , numpy.size(grp2))
                        #sheepmovements(initpoints,n,c,rs,Sh,ra,rhoa,rhos,unitvec,h,e,dps,mode2,destination2,p,swarm_split)
                        #Sh1,mode2,I2 = sheperdMovement(numpy.asarray([Sh[1],Sh[3]]).reshape(2,2),initpoints[0][g2],initpoints[1][g2],ra,Pd2,Pc,destination2,f2,mindisShep,rhoash,alpha,ang,mode2,nin,ds,rs,method2,predmasterslave2,decrad,q,I2)
                        #print("g2: " , g2)
                        Sh1,mode2,I2 = sheperdMovement(numpy.asarray([Sh[1]]).reshape(1,2),initpoints[0][g2],initpoints[1][g2],ra,Pd2,Pc,destination2,f2,mindisShep,rhoash,alpha,ang,mode2,nin,ds,rs,method2,predmasterslave2,decrad,q,I2)
                        print("")
                    else:
                        #out2 = 1;
                        eating2 = 1;
                        m2 = numpy.arctan2((destination[1] - destination2[1]),(destination[0] - destination2[0]))
                        if(m2 < 0):
                            alpha2 = pi + m2;
                        else:
                            alpha2 = m2; 
                        alpha2 = m2       
                        if(entered2 == 0):
                            dest2 = [0,0];
                            dest2[0] = destination2[0] - 8.0*numpy.cos(alpha2)
                            dest2[1] = destination2[1] - 8.0*numpy.sin(alpha2)
                            M2 = numpy.arctan2((destination[1] - dest2[1]),(destination[0] - dest2[0]))
                            entered2 = 1;
                        #print("UUUUUUUUU222222222222222UUUUU222222222UUUUUUUUUUUUUUUUU")
                        #print("dest2: " , dest2 , " alpha2: " , alpha2)
                        cur_angle2 = numpy.arctan2((Sh[1][1] - destination2[1]),(Sh[1][0] - destination2[0]))

                        if not (Sh[1][0] >= dest2[0] and Sh[1][1] >= dest2[1]):
                        #if(numpy.size(neg_y2) != 0):
                            # m2 = ANGLE BETWEEN DESTINATION AND DESTINATION2
                            # cur_angle2 = ANGLE B/W Sheperd2 and DESTINATION2
                            d2 = sqrt(pow(Sh[1][0] - destination2[0],2) + pow(Sh[1][1] - destination2[1],2))
                            d22 = sqrt(pow(dest2[0] - destination2[0],2) + pow(dest2[1] - destination2[1],2))
                            dr = d22 - d2; #PUT THE CONDITION TO CHECK DISTANCE BETWEEN dest2 and Sh and reduce that
                            #d2 = d2 + abs(cur_angle2)
                            S10 , a = moveSheperd(Sh1[0],numpy.asarray(dest2).reshape(1,2),1,1)
                            #S11 , a = moveSheperd(Sh1[1],numpy.asarray(dest2).reshape(1,2),1,1)
                            D20 =numpy.linalg.norm(initpoints.T-S10,axis=1)
                            #D21 =numpy.linalg.norm(initpoints.T-S11,axis=1)
                            if(numpy.where(D20 < rs)):
                                S10, a = moveSheperd(S10,numpy.asarray(destination2).reshape(1,2),1,-1)
                            # if(numpy.where(D21 < rs)):    
                            #     S11, a = moveSheperd(S11,numpy.asarray(destination2).reshape(1,2),1,-1)
                            Sh1[0][0] = S10[0][0]
                            Sh1[0][1] = S10[0][1]
                            #Sh1[1][0] = S11[0][0]
                            #Sh1[1][1] = S11[0][1]                                                    
                        

                    Sh[0][0] = S0[0][0]
                    Sh[0][1] = S0[0][1]
                    # Sh[2][0] = S0[1][0]
                    # Sh[2][1] = S0[1][1]                
                    Sh[1][0] = Sh1[0][0]
                    Sh[1][1] = Sh1[0][1]
                    # Sh[3][0] = Sh1[1][0]
                    # Sh[3][1] = Sh1[1][1]  



                if(out1 == 1 and out2 == 1 and (in1 == 0 or in2 == 0)): #and pos_update1 == 1 and pos_update2 == 1
                    #print("|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||")
                    if(enter_once2 == 0):
                        mode1 = 2
                        mode2 = 2;                 
                        destination = [10, 0]                               
                        M.markers[N+nos+nod-1].pose.position.x = destination[0]
                        M.markers[N+nos+nod-1].pose.position.y = destination[1]
                        M.markers[N+nos+nod-1].scale.x = 2*(threshold+5)
                        M.markers[N+nos+nod-1].scale.y = 2*(threshold+5)                    
                        enter_once2 = 1;
                    
                    outthreshold = []        
                    for i in range(N):
                        dist.append(sqrt(pow(initpoints[0][i] - destination[0],2) + pow(initpoints[1][i] - destination[1],2)))
                        if(sqrt(pow(initpoints[0][i] - destination[0],2) + pow(initpoints[1][i] - destination[1],2)) > (threshold)+5): #making threshold 20, so that joining happens earleir                    
                            outthreshold.append(i)

                    g1 = numpy.intersect1d(grp1,outthreshold)
                    g2 = numpy.intersect1d(grp2,outthreshold)
                    # if(len(g1) < len(g2)):
                    #     n = len(g1)
                    # else:
                    #     n = len(g2)    
                    swarm_split = 1
                    # if(len(outthreshold) > 0.1*(N)):
                    #     #print("len(outthreshold): " , len(outthreshold) , " 0.1*(N): " , 0.1*(N))
                    #     sheepmovements(initpoints,n,c,rs,Sh,ra,rhoa,rhos,unitvec,h,e,dps,mode,destination,p,swarm_split)  
                    #     #sheepmovements(initpoints,floor(0.9*(len(g1))),c,rs,Sh,ra,rhoa,rhos,unitvec,h,e,dps,mode,destination,p,swarm_split)
                    #     #sheepmovements(initpoints,floor(0.9*(len(g2))),c,rs,Sh,ra,rhoa,rhos,unitvec,h,e,dps,mode,destination,p,swarm_split)
                    #     S0,mode1,I1 = sheperdMovement(Sh[0].reshape(1,2),initpoints[0][g1],initpoints[1][g1],ra,Pd,Pc,destination,f,mindisShep,rhoash,alpha,ang,mode1,nin,ds,rs,method,predmasterslave,decrad,q,I1)    
                    #     S1,mode1,I2 = sheperdMovement(Sh[1].reshape(1,2),initpoints[0][g2],initpoints[1][g2],ra,Pd,Pc,destination,f,mindisShep,rhoash,alpha,ang,mode2,nin,ds,rs,method,predmasterslave,decrad,q,I2)
                    #     #out3 = 1;    
                    #print("g1: " , g1)
                    #print("g2: " , g2)

                    sheepmovements(initpoints,n,c,rs,Sh,ra,rhoa,rhos,unitvec,h,e,dps,mode1,destination,p,swarm_split,destination)

                    if(len(g1) > ceil(0.1*len(grp1)) and in1 == 0 and too_close == 0):
                        f1 = pow(len(grp1),0.5)
                        Pd1 = ra*log(len(grp1))
                        #print("A2")
                        #n = ceil(9*len(grp1)/10)
                        
                        #S0,mode1,I1 = sheperdMovement(numpy.asarray([Sh[0],Sh[2]]).reshape(2,2),initpoints[0][g1],initpoints[1][g1],ra,Pd1,Pc,destination,f1,mindisShep,rhoash,alpha,ang,mode1,nin,ds,rs,method1,predmasterslave1,decrad,q,I1)      
                        S0,mode1,I1 = sheperdMovement(numpy.asarray([Sh[0]]).reshape(1,2),initpoints[0][g1],initpoints[1][g1],ra,Pd1,Pc,destination,f1,mindisShep,rhoash,alpha,ang,mode1,nin,ds,rs,method1,predmasterslave1,decrad,q,I1) 
                    #elif(): #if the swarms meet mid way, then herd together
                    else:
                        in1 = 1;   

                    if(len(g2) > ceil(0.1*len(grp2)) and in2 == 0 and too_close == 0):#if too_close becomes 1, then in2 will become 1
                        f2 = pow(len(grp2),0.5)
                        Pd2 = ra*log(len(grp2))
                        #print("A3")
                        #n = ceil(9*len(grp2)/10)
                        #sheepmovements(initpoints,n,c,rs,Sh,ra,rhoa,rhos,unitvec,h,e,dps,mode2,destination,p,swarm_split)
                        #Sh1,mode2,I2 = sheperdMovement(numpy.asarray([Sh[1],Sh[3]]).reshape(2,2),initpoints[0][g2],initpoints[1][g2],ra,Pd2,Pc,destination,f2,mindisShep,rhoash,alpha,ang,mode2,nin,ds,rs,method2,predmasterslave2,decrad,q,I2)
                        Sh1,mode2,I2 = sheperdMovement(numpy.asarray([Sh[1]]).reshape(1,2),initpoints[0][g2],initpoints[1][g2],ra,Pd2,Pc,destination,f2,mindisShep,rhoash,alpha,ang,mode2,nin,ds,rs,method2,predmasterslave2,decrad,q,I2)                      
                        #S3,mode2,I2 = sheperdMovement(Sh[3].reshape(1,2),initpoints[0][g2],initpoints[1][g2],ra,Pd,Pc,destination,f,mindisShep,rhoash,alpha,ang,mode2,nin,ds,rs,method,predmasterslave,decrad,q,I2)                      
                    else:    
                        in2 = 1;

                    Sh[0][0] = S0[0][0]
                    Sh[0][1] = S0[0][1]
                    # Sh[2][0] = S0[1][0]
                    # Sh[2][1] = S0[1][1]                
                    Sh[1][0] = Sh1[0][0]
                    Sh[1][1] = Sh1[0][1]
                    # Sh[3][0] = Sh1[1][0]
                    # Sh[3][1] = Sh1[1][1]                          
                    
                    # if(too_close == 0):
                    #     #print("CCCCCCCCCCCCCCCCCCCCCCCCCC")
                    #     for j1 in grp1:
                    #         for j2 in grp2:
                    #             #inter_dist = numpy.linalg.norm(initpoints[:][j1] - initpoints[:][j2],axis = 1)    
                    #             inter_dist = numpy.sqrt(pow(initpoints[0][j1] - initpoints[0][j2],2) + pow(initpoints[0][j1] - initpoints[0][j2],2))
                    #             if(inter_dist <= 3.0):
                    #                 too_close = 1;
                    #                 break;
                    #         if(too_close == 1):
                    #             break;          

                if(in1 == 1 and in2 == 1):
                    #m3 = (m1+m2)/3
                    #temp_dx = destination[0] + 8.0*numpy.cos(m3)
                    #temp_dy = destination[1] + 8.0*numpy.sin(m3)
                    # M.markers[N+nos+nod-1].pose.position.x = temp_dx
                    # M.markers[N+nos+nod-1].pose.position.y = temp_dy
                    # M.markers[N+nos+nod-1].scale.x = 2*(threshold+5)
                    # M.markers[N+nos+nod-1].scale.y = 2*(threshold+5)                    
                    swarm_split = 0; #swarm joined back, now herd sheep together
                    #new_dest = [temp_dx,temp_dy]
                    #rs = 1;
                    #destination = [0,0]
                    Pd = ra*log(N)
                    n = (N*90)/100
                    #n = ceil(0.9*N)
                    #print("=======================================================================")
                    sheepmovements(initpoints,n,c,rs,Sh,ra,rhoa,rhos,unitvec,h,e,dps,mode,destination,p,swarm_split,destination)
                    Sh,mode,I = sheperdMovement(Sh,initpoints[0][outthreshold],initpoints[1][outthreshold],ra,Pd,Pc,destination,f,mindisShep,rhoash,alpha,ang,mode1,nin,ds,rs,method,predmasterslave,decrad,q,I)
                        



                #if(out3 == 1):
                    #sheepmovements(initpoints,floor(0.9*N),c,rs,Sh,ra,rhoa,rhos,unitvec,h,e,dps,mode,destination,p,swarm_split)
                # if(out1 == 1 and out2 == 1 and (len(outthreshold) > 0.1*(N)) and out3 == 0):
                #     #print("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")
                #     sheepmovements(initpoints,floor(0.9*N),c,rs,Sh,ra,rhoa,rhos,unitvec,h,e,dps,mode,destination,p,swarm_split)
                #     S0,mode1,I1 = sheperdMovement(Sh[0].reshape(1,2),initpoints[0][g1],initpoints[1][g1],ra,Pd,Pc,destination,f,mindisShep,rhoash,alpha,ang,mode1,nin,ds,rs,method,predmasterslave,decrad,q,I1)    
                #     S1,mode1,I2 = sheperdMovement(Sh[1].reshape(1,2),initpoints[0][g2],initpoints[1][g2],ra,Pd,Pc,destination,f,mindisShep,rhoash,alpha,ang,mode2,nin,ds,rs,method,predmasterslave,decrad,q,I2)
                # else:
                #     out3 = 1;    
                #sheepmovements(initpoints[:,grp2],0.9*len(grp2),c,rs,Sh[1].reshape(1,2),ra,rhoa,rhos,unitvec,h,e,(dps/5.0),mode,destination2,p)   
            #sheepmovement(initpoints,e,p,ra,rhoa,dps,Shepherd,direction,n,c,rs,rhos,h)
            #sheperdmovement(Sh,initpoints[0],initpoints[1],ra,Pd,Pc,destination,f,mindisShep,rhoash,rs,I)
            #Sh,mode,I = sheperdMovement(Sh,initpoints[0][outthreshold],initpoints[1][outthreshold],ra,Pd,Pc,destination,f,mindisShep,rhoash,alpha,ang,mode,nin,ds,rs,method,predmasterslave,decrad,q,I)
            #if(toggle == 1):
                
                #print("Sh: " , Sh[0] , " " , Sh[0][0] , " " , Sh[0].reshape(1,2))
                #`print(initpoints[:,grp1])
                #print(initpoints[0][grp1])
                #Sh,mode,I = sheperdMovement(Sh,initpoints[0][grp1],initpoints[1][grp1],ra,Pd,Pc,destination1,f,mindisShep,rhoash,alpha,ang,mode,nin,ds,rs,method,predmasterslave,decrad,q,I)
                #Sh[0],mode,I = sheperdMovement(Sh[0].reshape(1,2),initpoints[0][grp1],initpoints[1][grp1],ra,Pd,Pc,destination1,f,mindisShep,rhoash,alpha,ang,mode,nin,ds,rs,method,predmasterslave,decrad,q,I)
                #S1,mode1,I2 = sheperdMovement(Sh[1].reshape(1,2),initpoints[0][grp2],initpoints[1][grp2],ra,Pd,Pc,destination2,f,mindisShep,rhoash,alpha,ang,mode2,nin,ds,rs,method,predmasterslave,decrad,q,I2)
                #print("WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWw")
                #print("After: " , Sh[0] , " " , grp1)        
                #print("len_grp1 + len_grp2  :", len(grp1) + len(grp2))
                #print("len g1 + len g2 " , len(g1) + len(g2))
                #print("smae or nmto " , S0[0][0] , " " , Sh[0][0])
                #print("S0[0]"  , S0[0] , " " , S0[0].reshape(1,2))
                #print("g2 : " , g2)
                #print("grp2" , grp2)
                #print("outthreshold2 " , outthreshold2)
                #print("Sh1[0] " , Sh1)
                #print("numpy.asarray : " , numpy.asarray([Sh1[0],Sh1[1]]).reshape(2,2))
                #print("out2 : " , out2)
                #print("scale x : " , M.markers[N+nos+1].scale.x)
                #print("Sh1: " , Sh1 , " Sh11 " , Sh11)

                # Sh[0][0] = S0[0][0]
                # Sh[0][1] = S0[0][1]
                # Sh[2][0] = S0[1][0]
                # Sh[2][1] = S0[1][1]                
                # Sh[1][0] = Sh1[0][0]
                # Sh[1][1] = Sh1[0][1]
                # Sh[3][0] = Sh1[1][0]
                # Sh[3][1] = Sh1[1][1]  
                
                #print("S0: " , S0 , " " , S0[0] , " " , S0[0][0] , " Sh[0]  " , Sh[0][0])
                #print("type: " , type(Sh), "Sh " , Sh)
            #rospy.Duration(0.05)
            #plt.pause(0.05) --- similar to ros::Duration(0.05) or rospy.Duration(0.05)
            #plt.clf() --- clears the plot screen blah blah
            #print(initpoints)

            #print("N: " , N)
            #print("yolo mode " , mode , " " , )
            #print("sheperd:" , Sh , " " , type(Sh))
            #print("I : " , I)
            GCM = [numpy.mean(initpoints[0][:]),numpy.mean(initpoints[1][:])]
            Dcheck = sqrt(pow(Sh[0][0] - GCM[0],2) + pow(Sh[0][1] - GCM[1],2))
            #for i in range(nos):
                #Dcheck.append(sqrt(pow(Sh[i][0] - numpy.mean(initpoints[0][:]),2) + pow(Sh[i][1] - numpy.mean(initpoints[1][:],2))))
            #print("oloy " ,  )
            #print("Distance from GCM: " , Dcheck, " ", type(Sh))
            #grp1 = [] # sheep grps
            #grp2 = []
            #temp_initpoints2 = copy.deepcopy(initpoints)
            #print("inipo " , initpoints)
            #print()
            
            ############SHEEEP CPNTROLLER ############################################
            for i in range(N):
                #if(toggle == 1):
                    #print(i)
                #change_in_heading[i] = numpy.arctan2(unitvec[1][i] , unitvec[0][i])    
                #delta_theta[i] = change_in_heading[i] - Yaw[i]; #we can scale this
                # if(delta_theta[i] >= pi):
                #     delta_theta[i] = delta_theta[i]%(pi)
                # elif(delta_theta[i] <= -(pi)):
                #     delta_theta[i] = delta_theta[i]%(-pi)                
                #temp_initpoints2 = copy.deepcopy(initpoints)
                
                M.markers[i].pose.position.x = initpoints[0][i] #huskyX[i] #initpoints[0][i] 
                M.markers[i].pose.position.y = initpoints[1][i] #huskyY[i] #initpoints[1][i] 
                #print(numpy.linalg.norm(initpoints[:,i].T - temp_initpoints[:,i].T))
                #initpoints[0][i] = huskyX[i]
                #initpoints[1][i] = huskyY[i]
                
                #dist_left_to_cover[i] = numpy.sqrt(pow(temp_initpoints2[1][i]-huskyY[i],2) + pow(temp_initpoints2[0][i]-huskyX[i],2))
                #if(dist_left_to_cover[i] < 0.01 or i in inthreshold1 or i in inthreshold2):
                    #speed_sheep[i] = 0.0;
                    #delta_theta[i] = 0.0
                #else:
                    #speed_sheep[i] = 0.5*dist_left_to_cover[i] #?? how will this work: works amazingly good                

                #H[i].linear.x = speed_sheep[i]
                #H[i].angular.z = k*(delta_theta[i])
                #print("i: " , i)
                #print("dist left: " , dist_left_to_cover[i] , " " , delta_theta[i])
                #print("init: " , temp_initpoints2[:,i] , " HU: " , huskyX[i],huskyY[i] )


            # H0.linear.x = speed_sheep[0];
            # H0.angular.z = k*(delta_theta[0])            
            # H1.linear.x = speed_sheep[1];
            # H1.angular.z = k*(delta_theta[1])     
            # H2.linear.x = speed_sheep[2];
            # H2.angular.z = k*(delta_theta[2])
            # H3.linear.x = speed_sheep[3];
            # H3.angular.z = k*(delta_theta[3])            
            #print("K= " , k)

            #############PREDATOR CONTROLLER#######################################################################################
            for i in range(nos):    
                #M.markers[N1+i].pose.position.x = M.markers[N1+i].pose.position.x + s*(2.0)*(toggle);
                #M.markers[N1+i].pose.position.x = Sh[i][0]
                print("i= " , i , " pred")
                print("excepted position: " , Sh[i])
                print("s: " , s , " switch: " , switch)
                #print("current positon: " , [husky_predX[i], husky_predY[i]])
                if(toggle == 0):
                    #sheperd_speed[i] = 1.5;                                        
                    M.markers[N1+i].pose.position.y =   M.markers[N1+i].pose.position.y - (0.15)*(switch);
                if(toggle == 1):
                    #change_in_heading_shepherd[i] = numpy.arctan2(Sh[i][1] - temp_Sh[i][1], Sh[i][0] - temp_Sh[i][0]) #find the new heading using initial positon and the new postion
                    #print("change in angle: " , change_in_heading_shepherd[i]*180/pi)
                    #delta_theta_shepherd[i] = change_in_heading_shepherd[i] - Yaw_pred[i]
                    #print("delta before normalising: " , delta_theta_shepherd[i]*180/pi)
                    # if(delta_theta_shepherd[i] >= pi):
                    #     delta_theta_shepherd[i] = delta_theta_shepherd[i]%(pi)
                    # elif(delta_theta_shepherd[i] <= -(pi)):
                    #     delta_theta_shepherd[i] = delta_theta_shepherd[i]%(-pi)
                    #print("delta_theta after: " , delta_theta_shepherd[i]*180/pi)    
                    #H3.angular.z = k*(change_in_heading_shepherd - Yaw[N1+i])
                    #sheperd_speed[i] = 0.75
                    #pred_dist_from_sheep = numpy.linalg.norm(initpoints.T - Sh[i] , axis = 1)
                    #no_of_sheep_within_5m = numpy.where(pred_dist_from_sheep < 3.0  ) #3*ra as per Strombom
                    #no_of_sheep_within_1m = numpy.where(pred_dist_from_sheep < 2.0)
                    #if(numpy.size(no_of_sheep_within_5m) > 0):
                    #    sheperd_speed[i] = 0.5 #to prevent the predator hitting the sheep
                        # if(numpy.size(no_of_sheep_within_1m) > 0):
                        #     speed_sheep[no_of_sheep_within_1m[0]] = 0.6#increasing the speed of sheep to prevent sticking with the predator

                    #H3.linear.x = sheperd_speed;    
                    M.markers[N1+i].pose.position.x = Sh[i][0]#husky_predX[i]  #Sh[i][0]
                    M.markers[N1+i].pose.position.y = Sh[i][1]#husky_predY[i]  #Sh[i][1]

                Sh[i][0] = M.markers[N1+i].pose.position.x
                Sh[i][1] = M.markers[N1+i].pose.position.y
                # Sh00 = [Sh[0][0],Sh[0][1]]
                # Sh01 = [Sh[2][0],Sh[2][1]]
                # Sh10 = [Sh[1][0],Sh[1][1]]
                # Sh11 = [Sh[3][0],Sh[3][1]]
                print("")                                  
                if(nos > 1):
                    #toggle = toggle*(-1)
                    switch = switch*(-1)
                #print(i , " x " , M.markers[N1+i].pose.position.x , "  " , M.markers[N1+i].pose.position.y)
                #H_pred[i].linear.x = sheperd_speed[i]
                #H_pred[i].angular.z = k*(delta_theta_shepherd[i])
                
            # H4.linear.x = sheperd_speed[0]        
            # H4.angular.z = k_pred*(delta_theta_shepherd[0])            
            # H5.linear.x = sheperd_speed[1]
            # H5.angular.z = k_pred*(delta_theta_shepherd[1])
                           
            
            #if(eating1 == 1 and out1 == 0): # 
            if(out1 == 0):
                #near_dest1 = numpy.linalg.norm(initpoints[:][grp1].T - destination1,axis = 1)
                #in_dest1 = numpy.where(near_dest1 < threshold) 
                #print("EEEEEEAAAAAAAAAAAAAAATTTTTTTTTTTTIIIIIIIINNNNNNNNNNGGGGGGGGGGGG!!!!!!!!!!!")               
                in_dest1 = N - numpy.size(outthreshold1)
                if(in_dest1 == 0 and eating1 == 1):
                    in_dest1 = 1;
                M.markers[N1+nos].scale.x = M.markers[N1+nos].scale.x - (0.02*in_dest1)
                M.markers[N1+nos].scale.y = M.markers[N1+nos].scale.y - 0.02*(in_dest1)
                #S0,mode1,I1 = sheperdMovement(Sh[0].reshape(1,2),[],[],ra,Pd,Pc,dest1,f,mindisShep,rhoash,alpha,ang,mode1,nin,ds,rs,method,predmasterslave,decrad,q,I1)      
                neg_x1 = numpy.where(Sh[0][0] > initpoints[0,grp1])
                neg_y1 = numpy.where(Sh[0][1] - 2 < initpoints[1,grp1])
                #print("neg_x1: " , neg_x1)
                #if((m1 - cur_angle1) > pi/10):
                #print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
                #print("grp1: " , len(grp1), " in_dest1: " , in_dest1 , " " ,in_dest1/len(grp1))
                if(M.markers[N1+nos].scale.x < 0.20):
                    #eating1 = 0;
                    out1 = 1;     

            # if(eating1 == 1 and pos_update1 == 0 and (out2 == 0 or out1 == 0)):    
            #     #if(sqrt(pow(dest1[0] - Sh[0][0],2) + pow(dest1[1] - Sh[0][1],2)) > 3.0):
            #     if not (Sh[0][0] <= dest1[0] and Sh[0][1] >= dest1[1]):
            #         #print("dest1: " , dest1 , " Sh[0] " , Sh[0])
            #     #if(numpy.size(neg_y1) != 0):
            #         d1 = sqrt(pow(Sh[0][0] - destination1[0],2) + pow(Sh[0][1] - destination1[1],2))
            #         d11 = sqrt(pow(dest1[0] - destination1[0],2) + pow(dest1[1] - destination1[1],2))
            #         dr = d11 - d1;
            #         S00 , a = moveSheperd(S0[0],numpy.asarray(dest1).reshape(1,2),1,1)
            #         S01 , a = moveSheperd(S0[1],numpy.asarray(dest1).reshape(1,2),1,1)
            #         D10=numpy.linalg.norm(initpoints.T-S00,axis=1)
            #         D11=numpy.linalg.norm(initpoints.T-S01,axis=1)
            #         if(numpy.where(D10 < rs)):
            #             S00, a = moveSheperd(S0[0],numpy.asarray(destination1).reshape(1,2),1,-1)
            #         if(numpy.where(D11 < rs)):    
            #             S01, a = moveSheperd(S0[1],numpy.asarray(destination1).reshape(1,2),1,-1)
            #         S0[0][0] = S00[0][0]
            #         S0[0][1] = S00[0][1]
            #         S0[1][0] = S01[0][0]
            #         S0[1][1] = S01[0][1]    
            #     else:
            #         pos_update1 = 1;    



            #if(eating2 == 1 and out2 == 0 ): #
            #in_dest2 = N - numpy.size(outthreshold2)
            if(out2 == 0):
                #print("grp2: ", grp2)
                #print("initpoints[:][grp2], " , initpoints[:][grp2].T)
                #near_dest2 = numpy.linalg.norm(initpoints[:][grp2].T - destination2,axis = 1)
                #in_dest2 = numpy.where(near_dest2 < threshold)
                in_dest2 = N - numpy.size(outthreshold2)
                if(in_dest2 == 0 and eating2 == 1):
                    in_dest2 = 1;                
                M.markers[N1+nos+1].scale.x = M.markers[N1+nos+1].scale.x - (0.02*(in_dest2))
                M.markers[N1+nos+1].scale.y = M.markers[N1+nos+1].scale.y - (0.02*(in_dest2))
                #S1,mode2,I2 = sheperdMovement(Sh[1].reshape(1,2),[],[],ra,Pd,Pc,dest2,f,mindisShep,rhoash,alpha,ang,mode2,nin,ds,rs,method,predmasterslave,decrad,q,I2)      
                # print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
                #print("m2: " , m2 , " cur_angle2 " , cur_angle2)
                #print("grp1: " , len(grp2), " in_dest1: " , in_dest2 , " " ,in_dest2/len(grp2))
                neg_x2 = numpy.where(Sh[1][0] < initpoints[0,grp2])
                neg_y2 = numpy.where(Sh[1][1] - 2 < initpoints[1,grp2])
                #print("neg_x2: " , neg_x2)
                #if((m2 - cur_angle2) < -pi/10):
                if(M.markers[N1+nos + 1].scale.x < 0.20):
                    #eating2 = 0;
                    out2 = 1;

            c2 = 0;
                    
            # if(eating2 == 1 and pos_update2 == 0 and (out2 == 0 or out1 == 0)):        
            #     #if(sqrt(pow(dest2[0] - Sh[1][0],2) + pow(dest2[1] - Sh[1][1],2)) > 3.0):
            #     if not (Sh[1][0] >= dest2[0] and Sh[1][1] >= dest2[1]):
            #     #if(numpy.size(neg_y2) != 0):
            #         # m2 = ANGLE BETWEEN DESTINATION AND DESTINATION2
            #         # cur_angle2 = ANGLE B/W Sheperd2 and DESTINATION2
            #         d2 = sqrt(pow(Sh[1][0] - destination2[0],2) + pow(Sh[1][1] - destination2[1],2))
            #         d22 = sqrt(pow(dest2[0] - destination2[0],2) + pow(dest2[1] - destination2[1],2))
            #         dr = d22 - d2; #PUT THE CONDITION TO CHECK DISTANCE BETWEEN dest2 and Sh and reduce that
            #         #d2 = d2 + abs(cur_angle2)
            #         S10 , a = moveSheperd(Sh1[0],numpy.asarray(dest2).reshape(1,2),1,1)
            #         S11 , a = moveSheperd(Sh1[1],numpy.asarray(dest2).reshape(1,2),1,1)
            #         D20 =numpy.linalg.norm(initpoints.T-S10,axis=1)
            #         D21 =numpy.linalg.norm(initpoints.T-S11,axis=1)
            #         if(numpy.where(D20 < rs)):
            #             S10, a = moveSheperd(S10,numpy.asarray(destination2).reshape(1,2),1,-1)
            #         if(numpy.where(D21 < rs)):    
            #             S11, a = moveSheperd(S11,numpy.asarray(destination2).reshape(1,2),1,-1)
            #         Sh1[0][0] = S10[0][0]
            #         Sh1[0][1] = S10[0][1]
            #         Sh1[1][0] = S11[0][0]
            #         Sh1[1][1] = S11[0][1]                            
            #     else:
            #         pos_update2 = 1    



            D1 = numpy.linalg.norm(initpoints.T-Sh[0],axis=1)
            D2 = numpy.linalg.norm(initpoints.T-Sh[1],axis=1)    
            n2 = numpy.size(numpy.where(D1 < rs/2)) #rs = 15 => rs/3 = 5
            n3 = numpy.size(numpy.where(D2 < rs/2)) #5 should be enough for gazebo based
            # print("??????????????????????????????????????????")
            # print("n2: " , n2 , " " , numpy.where(D1 < rs))
            # print("Sh[0]: " , Sh[0])
            #print("D1: " , D1)
            if((n2 > 0 or n3 > 0) and crossed_swarm == 0):
                s = 0.15
            else:
                s = 0.35
            #WON"T NEED THE SHEPERDS TO GO BACK and FORTH    
            # if(Sh[0][1] <= Sh[1][1]):
            #     switch = switch*(-1)
            #     crossed_swarm = 1;
            #     s = s*3

            #if(Sh[0][1] <= Sh[1][1] and enter_once == 0):
            #print("dist b/w preds: " , Sh[0][1] , " " , Sh[1][1])
            if(abs(Sh[0][1] - Sh[1][1]) < rs/2 and enter_once == 0): #the rpedators start from the x axis
                switch = 0;
                toggle = 1;
                #crossed_swarm = 1;
                enter_once = 1;
                split_time[0][mc1] = rospy.get_rostime().to_sec()
                for j in range(len(initpoints[0])):
                    if(initpoints[0][j] < 0):
                        grp1.append(j)
                    else:
                        grp2.append(j)

                x1 = initpoints[0][grp1]
                y1 = initpoints[1][grp1]

                x2 = initpoints[0][grp2]
                y2 = initpoints[1][grp2]

                crossed_swarm = 0; #this happens only once
                            
                # M.markers[N1].pose.position.y = 100;
                # M.markers[N1+nos-1].pose.position.y = 100;
                # Sh[0][1] = 100;
                # Sh[nos-1][1] = 100;
                #rs = rs + 5;    
            temp_initpoints = copy.deepcopy(initpoints)    
            
            dist = []    
            flag = 0
            #print("Destination: " ,destination)
            # for i in range(N):
            #     dist.append(sqrt(pow(initpoints[0][i] - destination[0],2) + pow(initpoints[1][i] - destination[1],2)))
            #     if(sqrt(pow(initpoints[0][i] - destination[0],2) + pow(initpoints[1][i] - destination[1],2)) <= threshold):
            #         flag = flag + 1

            #if(out1 == 1 and out2 == 1):
                # outthreshold = []        
                # for i in range(N):
                #     dist.append(sqrt(pow(initpoints[0][i] - destination[0],2) + pow(initpoints[1][i] - destination[1],2)))
                #     if(sqrt(pow(initpoints[0][i] - destination[0],2) + pow(initpoints[1][i] - destination[1],2)) > threshold):
                #         #flag = flag + 1
                #         outthreshold.append(i)
                        #print("Deleting " , i  ," " )
            
            if(out1 == 0):            
                outthreshold1 = []   
                inthreshold1 = []
                for i in range(N):
                    dist.append(sqrt(pow(initpoints[0][i] - destination1[0],2) + pow(initpoints[1][i] - destination1[1],2)))
                    if(sqrt(pow(initpoints[0][i] - destination1[0],2) + pow(initpoints[1][i] - destination1[1],2)) > threshold):
                        #flag = flag + 1
                        outthreshold1.append(i)
                    else:
                        inthreshold1.append(i)    

            if(out2 == 0):     
                #print("fiding outthreshold2 here")
                inthreshold2 = []       
                outthreshold2 = []        
                for i in range(N):
                    dist.append(sqrt(pow(initpoints[0][i] - destination2[0],2) + pow(initpoints[1][i] - destination2[1],2)))
                    if(sqrt(pow(initpoints[0][i] - destination2[0],2) + pow(initpoints[1][i] - destination2[1],2)) > threshold):
                        #flag = flag + 1
                        outthreshold2.append(i)
                    else:
                        inthreshold2.append(i)    
                #print("this is what i found : " , outthreshold2 )        
            
            #dist_from_dest = numpy.linalg.norm(initpoints.T - new_dest,axis = 1)
            dist_from_dest = numpy.linalg.norm(initpoints.T - destination,axis = 1)
            outthreshold = numpy.where(dist_from_dest > threshold + 5)
            inthreshold = numpy.where(dist_from_dest <= threshold + 5)
            in_g1 = numpy.intersect1d(inthreshold[0],grp1)
            in_g2 = numpy.intersect1d(inthreshold[0],grp2)
            #if(numpy.size(inthreshold) > 0 and out1 == 1 and out2 == 1 and eaten3 == 0):
            if(numpy.size(in_g1) > 0 and numpy.size(in_g2) > 0):    
                #START FEEDING
                #print("PPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPP")
                #print("inthreshold: " , numpy.size(inthreshold) , " " , 0.2/(0.9*N) , " " , 0.2*(numpy.size(inthreshold)))
                M.markers[N+nos+nod-1].scale.x = M.markers[N+nos+nod-1].scale.x - 0.02*(numpy.size(inthreshold))
                M.markers[N+nos+nod-1].scale.y = M.markers[N+nos+nod-1].scale.y - 0.02*(numpy.size(inthreshold))
                if(M.markers[N+nos+nod-1].scale.x < 0.1):
                    eaten3 = 1;

            if(eaten3 == 1):
                #print("one complete");
                agents_dist  = numpy.linalg.norm(initpoints.T - numpy.asarray(destination).reshape(1,2),axis = 1)
                agents_away = numpy.where(agents_dist > threshold + 5 + rs)
                agent_sep[0][mc1] = numpy.size(agents_away)
                break;            

            #print("outthreshold " , outthreshold )
            #print(dist)
            #print("sheeps pos " , initpoints[])
            #if(flag >= 1):
                #for i in cordi:
                    #print(i , " " , initpoints[0][i] , " " , initpoints[1][i])
                #break        
                    #temp_initpoints = numpy.delete(temp_initpoints,i,1)
                    
            # for i in reversed(cordi):
            #     initpoints = numpy.delete(initpoints,i,1)
            #     direction = numpy.delete(direction,i,1)
            #     #print("dist: " , dist[i] , )
            #print("DELETING " , numpy.shape(initpoints))
            #initpoints = temp_initpoints        
            #if(flag >= n):
                #print("STOPPPPPPPPPPPPPP " , "count is: " , count)
                #break        

            count = count + 1;
            #print("COUNT: " , count)
            if(count > 2000):
                print("Count more than , " , count)
                not_completed[0][mc1] = 1;
                if(M.markers[N+nos].scale.x > 0.2):
                    task[0][mc1] = 1;
                if(M.markers[N+nos+1].scale.x > 0.2):
                    task[0][mc1] = task[0][mc1] + 2;    
                elif(eaten3 != 0 and eating2 == 1 and eating1 == 1):
                    task[0][mc1] = 4
                agents_dist  = numpy.linalg.norm(initpoints.T - numpy.asarray(destination).reshape(1,2),axis = 1)
                agents_away = numpy.where(agents_dist > threshold + 5 + rs)
                agent_sep[0][mc1] = numpy.size(agents_away)                    
                break    
            
            rate.sleep()


        #print("SSSSSSSSSSSSSSSSSSSSSSSSSSSSS")
        #print("mc1 : " ,mc1)    
        stop_time[0][mc1] = rospy.get_rostime().to_sec()
        split_diff[0][mc1] = split_time[0][mc1] - start_time[0][mc1]
        diff[0][mc1] = stop_time[0][mc1] - start_time[0][mc1]

        #print("diff: " , diff)
    print("++++++++++++++++++++++++++++++++++++++")
    diff = stop_time - start_time
    split_diff = stop_time - split_time
    print("FINAL diff: " , diff)
    print("FINAL SPLIT DIFF" , split_diff)
    print("not_completed: " , not_completed)
    print("Final Destinations: " )
    print("agent_sep :" , agent_sep)
    print("task problem: " , task)


if __name__ == '__main__':
    #x = []
    #y = []
    try:
        main()
    except rospy.ROSInterruptException:
        pass
