%% Initializaing Agent Params
close all;
clear;
mode=2;
A1=[];
destination=[125 125];
herdcount=0;
collectcount=0;
method=1;
saver=[];
Dist=[];
human=0;
countiter=zeros(2,6);
% zebra=[1100 300;1000 3000];
for predmasterslave=3
    for Ns=1%:4%:2:5 %D(t)<=2*rs*rhoa*Ns*ds/(dps*1.8*pi)
        for N=100%:100:500%100:100:600%:100:%0:1000:6000
            %             N=zebra(Ns-2,N1);
            Dist=[];
            %     N = 50; % No of agents
            n = ceil(0.9*N);  % No of Nearest Agents
            remagents=N-n;
            %         predmasterslave=3   ;%hybrid=1 slave=2 %predator=3
            %             method=1;% 1-> stray collection by predator 2->stray collection by dolphin circle
            L = 150; %Size of the field
            rs = 30; %Shepherd detection distance
            ra = 2;% agent to agent interaction distance
            rhoa = 2;% relative strength of repulsion from other agents
            c = 1.05;% relative strength of attraction to the n nearest neighbours
            rhos = 1; % relative strength of repulsion from the shepherd
            h = 0.5;%relative strength of proceeding in the previous direction
            dps = 1;%agent displacement per time step 1 m ts21
            p = 0.05;%probability of moving per time step while grazing 0.05
            
            %% Initializing Shepherd Params
            
            %     Ns = 5; %Number of Shepherds
            ds = 1.5; %Shepherd displacement per time step 1.5 m ts21
            Pd = ra*log(N); % Driving position
            Pc = ra; % collecting position ra m behind the furthest agent
            e = 0.3; % relative strength of angular noise 0.3
            f = ra*sqrt(N);
            rhoash = 5;
            mindisShep = 6;
            
            %% Initializing Local Shepherd Params
            
            ns = 20; % number of nearest agents the local shepherd operates on 20
            b = pi/2;% blind angle behind the shepherd p/2
            q=1;% 2-> 100% encapsulation
            %% Main
            figure(1);
            hold on;
            plot(L,L);
            grid on;
            axis([0 L 0 L]);
            grid on;
            grid minor;
            hold on;
            [X,Y]=placeHerd(N);
            S = destination; %Release the Sheperds
            for i=2:Ns
                S(i,1)=S(1,1)+randi(10,1);
                S(i,2)=S(1,2)+randi(10,1);
            end
            Filename=strcat('Shepherds = ',num2str(Ns),' Sheep = ',num2str(N),'.avi');
            %                     Filename=strcat('Number of Shepherds = ',num2str(Ns),'Number of Sheep = ',num2str(N),'  Radius of Influence =',num2str(rs),'.avi');
            %                     Filename=strcat('PurePredator Number of Shepherds =',num2str(Ns),'Number of Sheep = ',num2str(N),'  Radius of Influence =',num2str(rs),'.avi');
            %             Filename=strcat('Cluster Coeff =',num2str(c));
            vidObj = VideoWriter(Filename);
            vidObj.FrameRate=1;
            open(vidObj);
            plot(S(:,1),S(:,2),'r+');
            hold on;
            unitvec=[];
            grid on;
            axis([0 L+50 0 L+50]);
            pindex = 1;
            alpha=1;
            ang=0;
            count=0;
            decrad=0.9;
            I=[];
            % while(count~=500)
            % S1=S;
            initpoints=[X',Y'];
            Smemory=[];
            modememory=[];
            while(1)
                %             viscircles(initpoints,ra*ones(size(initpoints,1),1),'LineWidth',1,'Color','b');
                %     S1=[S1;S];
                count=count+1;
                [N Ns count]
                N=length(X);
                Pd = ra*log(N);
                f = ra*sqrt(N);
                %      n= ceil(0.9*N);
                [A]=plotPoints(L,S,rs,destination,mode,I);
                %             A1=[A1;A];
                if n==1
                    rhoa=10;
                    rs=5;
                else
                    n=ceil(0.9*N);
                end
                if max(pdist2(initpoints,destination))>10 && count<1000
                    initpoints=[X',Y'];
                    [X,Y,unitvec] = sheepmovements(X,Y,n,c,rs,S,ra,rhoa,rhos,unitvec,h,e,initpoints,dps,mode,destination,count);
                    [X,Y] = placeHerd(X,Y);
                    if isempty(X)
                        break;
                    end
                    mode1=mode;
                    pause(0.0001);
                    idx=find(pdist2(initpoints,destination)>10);
                    if human==0
                        [S,alpha,ang,mode,n,I,currentmode] = sheperdMovement(S,X(idx),Y(idx),ra,Pd,Pc,destination,f,mindisShep,rhoash,alpha,ang,mode,n,ds,rs,method,predmasterslave,decrad,q,I);
                    else
                        set (gcf, 'WindowButtonMotionFcn', @mouseMove);
                        Sh = get (gca, 'CurrentPoint');
                        [S(1),S(2)]=stepspertimestep(Sh(1,1),Sh(1,2),[S(1),S(2)],ds);
                    end
%                     if (mode==6||Ns==1) && ~isempty(currentmode)
%                         Smemory=cat(3,Smemory,S);
%                         modememory=cat(3,modememory,currentmode);
%                     end
                else
                    break;
                end
                writeVideo(vidObj,getframe);
                clf;
                hold on;
                GCM=mean([X' Y']);
                %                 Dist=[Dist mean(mean(pdist2([X;Y]',S)))-2*rs];
                Dist=[Dist max(pdist2(GCM,[X;Y]'))];
            end
            %             Dister=[Dister;Dist];
            figure(9)
            plot(Dist);
            hold on;
            close(vidObj);
            %% Plot Shepherd path
            %             figure
            %             close all;
            %             j=1;
            %             i=1;
            %             plot(destination(2),destination(1),'g+')
            %             hold on;
            %             viscircles(destination,10,'LineWidth',1,'Color','y');
            %             plot([Smemory(j,1,i),Smemory(j,1,i+1)],[Smemory(1,2,i),Smemory(1,2,i+1)],'Color','k','LineStyle',':');
            %             plot([Smemory(j,1,i),Smemory(j,1,i+1)],[Smemory(1,2,i),Smemory(1,2,i+1)],'Color','k','LineStyle','-');
            %             c=['k' 'r' 'b' 'y' 'm' 'g' 'c'];
            %             for j=1:Ns
            %                 for i=1:size(Smemory,3)-1
            %                     if modememory(1,j,i)==1
            %                         plot([Smemory(j,1,i),Smemory(j,1,i+1)],[Smemory(j,2,i),Smemory(j,2,i+1)],'Color',c(j),'LineStyle','-');
            %                         herdcount=herdcount+1;
            %                     else
            %                         plot([Smemory(j,1,i),Smemory(j,1,i+1)],[Smemory(j,2,i),Smemory(j,2,i+1)],'Color',c(j),'LineStyle',':');
            %                         collectcount=collectcount+1;
            %                     end
            %                     hold on;
            %                 end
            %             end
            %             figure
            %             plot(A1)
            %             xlabel('Iterations');
            %             ylabel('Area Covered');
            %             title('..Collect ___Herd');
            % title('Predator algorithm with minimum distance between Sheep-Shepherd = 2*sqrt(N)');
            %         % savefig(gcf,strcat(Filename,'.fig'));
            %         saveas(gcf,strcat(strcat('Shepherd',num2str(Ns),num2str(N),' Sheep'),'.tif'))
            %         % saveas(gcf,strcat(Filename,'.eps'))
            %         temper=[Ns N herdcount collectcount herdcount+collectcount];
            %         saver=[saver;temper];
            %         if count>=2000
            %             break;
            %         end
            %         figure(Ns);
            %         plot(Dist);
            %         hold on;
            %         savefig(gcf,strcat(Filename(1:13),'.fig'));
            %         pause(0.00000000000001);
            %             if count>=1000
            %                 N=N-100;
            %             else
            %                 countiter(predmasterslave-1,N)=count;
            %             end
            
        end
        plot(ones(1,1000)*2*rs*rhoa*Ns*ds/(dps*1.8*pi))
    end
end
function mouseMove (object, eventdata)
C = get (gca, 'CurrentPoint');
% C=C(1,1:2);
end