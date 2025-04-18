close all;
clear;


global dt DT

%test
dt = 0.001;
DT = 0.01;

Qmin = [-inf,-inf,-inf,-pi/4,-1]';
Qmax = -Qmin;
Umin = [Qmin(4),Qmin(5)]';
Umax = -Umin;
L = 2.5;
tau_gamma = 0.0;
tau_v = 0.0;

a = 0: 0.1 : 2*pi;  % angle step for points on circular path 
xPath = 9 + 5*sin(a); yPath = 7-5*cos(a);

% x y theta gamma v
Q0 = [15;5;pi/2;0;0];
% gammaD VD
U0 = [0;1];
Q = Q0;
U = U0;

timeToRun = 60;
numTimesteps = timeToRun/DT;
integrationStepsPerTimeStep = DT/dt;
numIntegrationSteps = round(numTimesteps*integrationStepsPerTimeStep);
QAll = zeros((numIntegrationSteps),length(Q));
% 
% inputCommandMat = [0,1,0;
%                    Umax(1),1,1;
%                    -Umax(1),1,11;
%                    0,0,21];
% [commandMat] = movementCommands(timeToRun,DT,inputCommandMat);

Ld = 2;

path = [xPath;yPath];
crossTrackErrorMat = zeros(numTimesteps,1);

for j = 1:numTimesteps
    
    [gammaD,endDistance,crossTrackError] = purePursuit(Q,L,Ld,path);
    crossTrackErrorMat(j)=crossTrackError;
    if endDistance<0.3
        QAll = QAll(1:(j-1)*integrationStepsPerTimeStep,:);
        crossTrackErrorMat=crossTrackErrorMat(1:j);
        break
    end

    U = [gammaD;1];
    [QNext] = robot_bike_dyn(Q,U,Umin,Umax,Qmin,Qmax,L,tau_gamma,tau_v);
    Q = QNext(end,:)';
    QAll((j-1)*integrationStepsPerTimeStep+1:(j-1)*integrationStepsPerTimeStep+integrationStepsPerTimeStep,:) = QNext;
    
     

end

f2 = figure();
a2 = axes(f2);
lineLength = linspace(0,1);
thetaLine = [QAll(end,1),QAll(end,2)]+[(lineLength*cos(QAll(end,3)))',(lineLength*sin(QAll(end,3)))'];
hold on;


theta = linspace(0,2*pi)';
%desiredPath = rD*[cos(theta),sin(theta)]+[8,10];
%plot(desiredPath(:,1),desiredPath(:,2),'DisplayName','Desired Path')

plot(QAll(:,1),QAll(:,2),'LineWidth',2,'DisplayName','Path');
plot(thetaLine(:,1),thetaLine(:,2),'LineWidth',2,'DisplayName','Direction of Travel');
scatter(Q0(1),Q0(2),100,'o','DisplayName','Initial Position');
scatter(QAll(end,1),QAll(end,2),100,'*','DisplayName','Final Position');

% plot(QAll(:,1),QAll(:,2),'LineWidth',2,'DisplayName','Path');
% plot(thetaLine(:,1),thetaLine(:,2),'LineWidth',2,'DisplayName','Direction of Travel');
% plot(desiredPath(:,1),desiredPath(:,2),'--','LineWidth',1,'DisplayName','Desired Path')


xlabel('X []');
ylabel('Y []');
title("\tau_\gamma = 0, \tau_v = 0");

legend
axis equal



f3 = figure();
a3 = axes(f3);
hold on;
plot([0:DT:(length(crossTrackErrorMat)-1)*DT],crossTrackErrorMat,DisplayName='Cross Track Error')

xlabel('Time [s]');
ylabel('Cross Track Error [m]');
title("Cross Track Error vs. Time");

function [gammaD,endDistance,crossTrackError] = purePursuit(Q,L,Ld,path)
xPath = path(1,:);
yPath = path(2,:);


globalRobotLoc = [Q(1:2)];
distanceMatRobot= (sum((Q(1:2)-[xPath;yPath]).^2).^.5);
[Dmin,minPointIndex] = min(distanceMatRobot,[],2);
nearestPathPoint = [xPath(minPointIndex);yPath(minPointIndex)];
futurePathMat = [xPath(minPointIndex:end);yPath(minPointIndex:end)];
distanceMatPath = (sum((nearestPathPoint-futurePathMat).^2).^.5);
targetPointIndex = find(distanceMatPath>Ld,1);
globalTargetPoint = futurePathMat(:,targetPointIndex);
globalDistanceVector = globalTargetPoint-globalRobotLoc;
robotDistanceVector = trot2(Q(3), 'rad')'*[globalDistanceVector;1];
% close all
% plot(xPath,yPath)
% hold on
% scatter(globalRobotLoc(1),globalRobotLoc(2))
% scatter(nearestPathPoint(1),nearestPathPoint(2))
% scatter(globalTargetPoint(1),globalTargetPoint(2))
% axis equal
k=2*robotDistanceVector(2)/Ld^2;
gammaD = atan(k*L);
endDistance=distanceMatRobot(end);
crossTrackError=Dmin;
end




function [commandMat] = movementCommands(timeToRun,DT,inputCommandMat)
%specify input commands as a matrix where the first and second columns are
%the commands and the third column is the time at which it starts
timeSteps = [inputCommandMat(1,end):DT:inputCommandMat(end,end)];
%commandMat = interp1()
commandMat = [];
for m = 1:size(inputCommandMat,1)-1
    A = inputCommandMat(m,:);
    n = (inputCommandMat(m+1,end)-inputCommandMat(m,end))/DT;
    t = [inputCommandMat(m,end):DT:inputCommandMat(m+1,end)-DT]';
    B = repmat(A,[n,1]);
    B(:,end) = t;
    commandMat = [commandMat;B];
end
end


function [QNext] = robot_bike_dyn(Q,U,Umin,Umax,Qmin,Qmax,L,tau_gamma,tau_v)
global dt DT

U = max(U,Umin);
Q = max(Q,Qmin);
U = min(U,Umax);
Q = min(Q,Qmax);

x = Q(1);
y = Q(2);
theta = Q(3);
gamma = Q(4);
V = Q(5);

gammaD = U(1);
VD = U(2);

steps = (DT/dt-1);
QNext = zeros(steps+1,length(Q));
QNext(1,:) = Q;

if tau_gamma == 0
tau_gamma= dt;
end
if tau_v == 0
tau_v= dt;
end

for i = 1:steps
    x = Q(1);
    y = Q(2);
    theta = Q(3);
    gamma = Q(4);
    V = Q(5);

    xDot = V*cos(theta);
    yDot = V*sin(theta);
    thetaDot = V*tan(gamma)/L;
    VDot = (-V+VD)/tau_v;
    gammaDot = (-gamma+gammaD)/tau_gamma;
    QDot = [xDot;yDot;thetaDot;gammaDot;VDot];
    QNew = Q+QDot*dt;
    Q = QNew;

    QNext(i+1,:) = QNew;
end








end








































