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
            #--------SOMETHING-NEW------------------------------------------------------------------
            #indices of all the nieghbours within 10m of an agent
            #D_less_than_10 = numpy.where((orderedD < 10.0))            
            #n = len(D_less_than_10[0])            
            #---------------------------------------------------------------------------------------
            #----------------------INITIALLY--------------------------------------------------------------------------------------
            #sortedDindex=numpy.where((sortedD <= n)) #nearest neighbours which are inside a radius with center as the agent
            #Above line means taking indices of D which are less than n; i.e first n terms of the array D
            #But we want the nearest n agents that means we need the indices of the sortedD, which has the indices of array D
            #in ascending order. 
            #---------------------------------------------------------------------------------------------------------------
            sortedDindex=sortedD[:n] #CHANGED LINE
            sortedDindex = numpy.asarray(sortedDindex).reshape(1,n)
            #print("sortedD: " , sortedDindex )
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

def sheepmovements(initpoints,n,c,rs,Shepherd,ra,rhoa,rhos,unitvec,h,e,dps,mode,destination,p,swarm_split):
    #print("IN SHEEPMOVEMENT >>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
    #print("sheperd: " , Shepherd)
    copypoints = copy.deepcopy(initpoints)
    if(n != 1):
        #if(swarm_split == 1):
        initpoints = clusteringEffect(initpoints,n,c,rs,Shepherd,swarm_split)
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
