import numpy
import matplotlib.pyplot as plt
def inertialEffect( initpoints,direction,h ):
	inertiapoints=numpy.zeros((2,numpy.size(direction)/2))
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
			agent2=(initpoints[:,sortedDindex[0]]).mean(1)
			clusterpoints[:,closeagents[0][i]],agent2=move(agent1,agent2,c,1)
	return clusterpoints
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
	clusterpoints=clusteringEffect(initpoints[:],n,c,rs,Shepherd[:])
	errorpoints=angularnoiseEffect(initpoints[:],e,p) # The effect of noise   
	repelledpoints=selfRepellingEffect(initpoints[:],ra,rhoa)         
	fearpoints=sheperdingEffect(initpoints[:],Shepherd[:],rs,rhos) 
	inertiapoints=inertialEffect(initpoints[:],direction[:],h)
	allpoints=inertiapoints+repelledpoints+fearpoints+errorpoints+clusterpoints
	allpoints2=repelledpoints+fearpoints+errorpoints+clusterpoints
	initpoints,direction=stepspertimestep(allpoints,initpoints[:],dps)
	return direction
	# plt.scatter(currentpoints[0],currentpoints[1],marker='^')
	# return currentpoints,direction
def stepspertimestep(currentpoints,initialpoints,steps): # to limit the step taken per unit time
#steps: steps to be taken towards currentpoints from initialpoints
	difference=currentpoints-initialpoints
	l=numpy.shape(difference)
	if numpy.size(difference)>2:
		norm=numpy.linalg.norm(difference,axis=0)
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
	else:
		direction=[0,0]
		norm=numpy.linalg.norm(difference)
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
	return currentpoints,direction
def move( point1,point2,intensity,direction ):
	if numpy.shape(point1)[0]!=0 and numpy.shape(point2)[0]!=0:
		difference=point1-point2
		norm=numpy.linalg.norm(difference,axis=0)
		l=numpy.shape(point2)
		if numpy.size(point1)>2:
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
def angularnoiseEffect( initpoints,e,p ):	# The noise function
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
def plotpoints(initpoints,Shepherd,direction,rs,destination,destinationthreshold):
	plt.quiver(initpoints[0],initpoints[1],direction[0],direction[1],width=0.02,headwidth=0.2)
	plt.scatter(initpoints[0],initpoints[1],marker='^')
	plt.scatter(Shepherd[0],Shepherd[1], c='r')
	shepherdcircle=plt.Circle(Shepherd,rs, color='r', alpha=0.2, lw=5)
	fig = plt.gcf()
	ax = fig.gca()
	ax.add_artist(shepherdcircle)
	plt.scatter(destination[0],destination[1], c='g')
	destinationcircle=plt.Circle(destination,destinationthreshold, color='g', alpha=0.2, lw=5)
	fig = plt.gcf()
	ax = fig.gca()
	ax.add_artist(destinationcircle)
	GCM=initpoints.mean(1)
	plt.scatter(GCM[0],GCM[1],c='y')
def main():
	e=0.3 # strength of error movement
	p=0.05 # probability of an error
	Xstartpos=0 #swarm initial position
	Ystartpos=0
	N=100# number of agents
	n=numpy.ceil(0.9*N)
	dps=1 # speed of agents
	ds=1.5#speed of the shepherd
	ra=2.0  # distance for self repulsion
	rhoa=2.0# strength of self repulsion 
	rhos=1.0#strength of predatory repulsion 
	Shepherd=[-0,0]
	h=0.5#strength of inertial force
	rs=65   #radius of influence
	c=1.05 #strength of clustering
	destination=[150,150] #destination
	destinationthreshold=10
	X=Xstartpos+numpy.random.rand(1,N) #swarm initialization
	Y=Ystartpos+numpy.random.rand(1,N)
	initpoints=numpy.concatenate((X,Y),axis=0)
	direction=numpy.zeros((2,N))
	while(1):
		N=numpy.size(initpoints[0])
		Pd=ra*numpy.log(N)
		Pc=ra
		f=ra*numpy.sqrt(N)
		plotpoints(initpoints,Shepherd,direction,rs,destination,destinationthreshold)
		direction=sheepmovement(initpoints,e,p,ra,rhoa,dps,Shepherd,direction,n,c,rs,rhos,h)
		plt.pause(0.05)
		plt.clf()
if __name__== "__main__":
	main()