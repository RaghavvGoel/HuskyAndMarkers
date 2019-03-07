function[X,Y,V1] = initPhysico(X,Y,V1, deltaT,S,rs,dps,rhos,leaderidx,ra,destination,destinationthreshold)
idx=find(pdist2([X;Y]',destination)>destinationthreshold);
if ~isempty(S)
    idxshepherd=[];
    
    for i=1:size(S,1)
    idx1=intersect(find(pdist2([X',Y'],S(i,:))<=rs ),idx);
%   plot(X(idx1),Y(idx1))
    idxshepherd=union(idx1,idxshepherd);
    end

    idxgravity=intersect(setdiff(1:length(X),idxshepherd),idx);    
else
    idxgravity=intersect(setdiff(1:length(X),leaderidx),idx);
    idxshepherd=[];
end

Xleader=X(leaderidx);Yleader=Y(leaderidx);
Xshepherd=X(idxshepherd);Yshepherd=Y(idxshepherd);
Xgravity=X(idxgravity);Ygravity=Y(idxgravity);V=V1(idxgravity,:);
initpoints=[X',Y'];
D = [Xgravity',Ygravity'];
m = 1.0; % mass of agent
p = 2.0; % degree to the distance between the two agents
R = 5; % Range of repulsive forces (in metres)
C = 50; % range of forces applicable (in metres)
cf = 0.2; % coefficient of friction for self-stabilisation, restricting the maximum velocity
Fmax = 1; % Maximum force that can be experienced by each agent
% Gravitational constant specific to the hexagonal formation
G = Fmax * R^p * (2.0 - 1.5^(1-p))^(p/(1-p)); %this is = 56.25 ,after putting values in calculator
%% Inter-Agent
if ~isempty(idxgravity)
    for i=1:size(D,1)
        dist = D-D(i,:); % vector distance(B-A) between points
        d =pdist2(D,D(i,:)); % magnitude of distance(|B-A|) between points
        rows = find(d<=C & d>0 & d~=R); % filtering out zero the point itself
        dist = dist(rows,:);
        d = d(rows, :);
        repRows = find(d<R);
        d(repRows) = -d(repRows); % for repulsive force, negating
        scalar = G*m*m*d.^-(p+1); % assuming p = 2, otherwise sign for repulsion would create problems
        dist=(dist.*scalar);% getting projections of scalar along points
        F(i,:) = sum(dist,1); % getting resultant forces on each point
    end
    
    deltaV = (F * deltaT);
    V = ((V + deltaV)*(1.0 - cf));
    for i=1:size(V,1)
        if norm(V(i,:))>=1
            V(i,:)=normr(V(i,:));
        end
    end
    deltaD = V * deltaT;
    D = (D + deltaD);
    X(idxgravity) = D(:,1)';
    Y(idxgravity)= D(:,2)';
    V1(idxgravity,:)=V;
end
%% With Leader
if ~isempty(leaderidx)
    Xgravity=X(idxgravity);Ygravity=Y(idxgravity);V=V1(idxgravity,:);
    D = [Xgravity',Ygravity'];
    leader=[Xleader' Yleader'];
    for i=1:size(D,1)
        dist = leader-D(i,:); % vector distance(B-A) between points
        d =pdist2(leader,D(i,:)); % magnitude of distance(|B-A|) between points
        rows = find(d<=ra); % filtering out zero the point itself
        dist = dist(rows,:);
        d = d(rows, :);
        scalar = G*m*m*d.^-(p+1); % assuming p = 2, otherwise sign for repulsion would create problems
        dist=normr(dist.*scalar);% getting projections of scalar along points
        F(i,:) = sum(dist,1); % getting resultant forces on each point
    end
    deltaV = (F * deltaT);
    V = normr((V + deltaV)*(1.0 - cf));
    deltaD = V * deltaT;
    D = (D + deltaD);
    X(idxgravity) = D(:,1)';
    Y(idxgravity)= D(:,2)';
    V1(idxgravity,:)=V;
end
%% Shepherd
if ~isempty(S)
    [Xshepherd,Yshepherd] = sheperdingEffect(Xshepherd,Yshepherd,S,rs,rhos);
    X(idxshepherd)=Xshepherd;Y(idxshepherd)=Yshepherd;
    [X,Y,V] = stepspertimestep(X,Y,initpoints,dps*deltaT);
    V1(idxshepherd)=V(idxshepherd);
end
end