close all;
clear;


global dt DT

%test
dt = 0.001;
DT = 0.1;

Qmin = [-inf,-inf,-inf,-inf,-inf]';
Qmax = -Qmin;
Umin = [-inf,-inf]';
Umax = -Umin;
L = 1;
tau_gamma = 0.01;
tau_v = 0.01;

% x y theta gamma v
Q0 = [0;0;0;0;0];
% gammaD VD
U0 = [1;1];
Q = Q0;
U = U0;

numTimesteps = 10;
numIntegrationSteps = numTimesteps*DT/dt;

QAll = zeros(numIntegrationSteps,length(Q));
for j = 1:numTimesteps
    [QNext] = robot_bike_dyn(Q,U,Umin,Umax,Qmin,Qmax,L,tau_gamma,tau_v);
    Q = QNext(end,:)';
    QAll((j-1)*100+1:(j-1)*100+100,:) = QNext;
end

f1 = figure();
a1 = axes(f1);
lineLength = linspace(0,1);
thetaLine = [QAll(end,1),QAll(end,2)]+[(lineLength*cos(QAll(end,3)))',(lineLength*sin(QAll(end,3)))'];
hold on;
plot(QAll(:,1),QAll(:,2),'DisplayName','Path');
plot(thetaLine(:,1),thetaLine(:,2),'DisplayName','Direction of Travel');
legend
axis equal



%B Part 1
dt = 0.01;
DT = 0.1;

Qmin = [-inf,-inf,-inf,-pi/4,-5]';
Qmax = -Qmin;
Umin = [Qmin(4),Qmin(5)]';
Umax = -Umin;
L = 2.5;
tau_gamma = 0.0;
tau_v = 0.0;

rD = 3;
%find steering angle for 3 m
%thetaDot = omega = v tan (gamma/L) = omega*r tan (gamma/L) ;
% r = 1/tan(gamma/L);
% gammaD = L*atan(1/r)
gammaD = atan(L/rD);

% x y theta gamma v
Q0 = [10;10;pi/2;0;0];
% gammaD VD
U0 = [gammaD;1];
Q = Q0;
U = U0;

timeToRun = round(3.3*pi,-floor(log10(DT)));
numTimesteps = timeToRun/DT;
integrationStepsPerTimeStep = DT/dt;
numIntegrationSteps = round(numTimesteps*integrationStepsPerTimeStep);
QAll = zeros((numIntegrationSteps),length(Q));
for j = 1:numTimesteps+1
    [QNext] = robot_bike_dyn(Q,U,Umin,Umax,Qmin,Qmax,L,tau_gamma,tau_v);
    Q = QNext(end,:)';
    QAll((j-1)*integrationStepsPerTimeStep+1:(j-1)*integrationStepsPerTimeStep+integrationStepsPerTimeStep,:) = QNext;
end

f1 = figure();
a1 = axes(f1);
lineLength = linspace(0,1);
thetaLine = [QAll(end,1),QAll(end,2)]+[(lineLength*cos(QAll(end,3)))',(lineLength*sin(QAll(end,3)))'];
hold on;
plot(QAll(:,1),QAll(:,2),'LineWidth',2,'DisplayName','Path');
plot(thetaLine(:,1),thetaLine(:,2),'LineWidth',2,'DisplayName','Direction of Travel');

theta = linspace(0,2*pi)';
desiredPath = rD*[cos(theta),sin(theta)]+[10-rD,10];
plot(desiredPath(:,1),desiredPath(:,2),'--','LineWidth',1,'DisplayName','Desired Path')
legend
axis equal
xlabel('X []');
ylabel('Y []');
title("B, Part 1");



%B Part 2
dt = 0.01;
DT = 0.1;

Qmin = [-inf,-inf,-inf,-pi/4,-5]';
Qmax = -Qmin;
Umin = [Qmin(4),Qmin(5)]';
Umax = -Umin;
L = 2.5;
tau_gamma = 0.0;
tau_v = 0.0;

rD = 2;
%find steering angle for 3 m
%thetaDot = omega = v tan (gamma/L) = omega*r tan (gamma/L) ;
% r = 1/tan(gamma/L);
% gammaD = L*atan(1/r)
gammaD = atan(L/rD);

% x y theta gamma v
Q0 = [10;10;pi/2;0;0];
% gammaD VD
U0 = [gammaD;1];
Q = Q0;
U = U0;

timeToRun = round(3.3*pi*0.84,-floor(log10(DT)));
numTimesteps = timeToRun/DT;
integrationStepsPerTimeStep = DT/dt;
numIntegrationSteps = round(numTimesteps*integrationStepsPerTimeStep);
QAll = zeros((numIntegrationSteps),length(Q));
for j = 1:numTimesteps+1
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
desiredPath = rD*[cos(theta),sin(theta)]+[8,10];

plot(QAll(:,1),QAll(:,2),'LineWidth',2,'DisplayName','Path');
plot(thetaLine(:,1),thetaLine(:,2),'LineWidth',2,'DisplayName','Direction of Travel');
plot(desiredPath(:,1),desiredPath(:,2),'--','LineWidth',1,'DisplayName','Desired Path')

legend
axis equal
xlabel('X []');
ylabel('Y []');
title("B, Part 2");


%C Part 0
dt = 0.01;
DT = 0.1;

Qmin = [-inf,-inf,-inf,-pi/4,-5]';
Qmax = -Qmin;
Umin = [Qmin(4),Qmin(5)]';
Umax = -Umin;
L = 2.5;
tau_gamma = 0.0;
tau_v = 0.0;

% x y theta gamma v
Q0 = [10;10;pi/2;0;0];
% gammaD VD
U0 = [0;1];
Q = Q0;
U = U0;

timeToRun = 21;
numTimesteps = timeToRun/DT;
integrationStepsPerTimeStep = DT/dt;
numIntegrationSteps = round(numTimesteps*integrationStepsPerTimeStep);
QAll = zeros((numIntegrationSteps),length(Q));

inputCommandMat = [0,1,0;
                   Umax(1),1,1;
                   -Umax(1),1,11;
                   0,0,21];
[commandMat] = movementCommands(timeToRun,DT,inputCommandMat);
for j = 1:numTimesteps
    U = commandMat(j,1:2)';
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


%C Part 1
dt = 0.01;
DT = 0.1;

Qmin = [-inf,-inf,-inf,-pi/4,-5]';
Qmax = -Qmin;
Umin = [Qmin(4),Qmin(5)]';
Umax = -Umin;
L = 2.5;
tau_gamma = 0.0;
tau_v = 0.0;

% x y theta gamma v
Q0 = [10;10;pi/2;0;0];
% gammaD VD
U0 = [0;1];
Q = Q0;
U = U0;

timeToRun = 21;
numTimesteps = timeToRun/DT;
integrationStepsPerTimeStep = DT/dt;
numIntegrationSteps = round(numTimesteps*integrationStepsPerTimeStep);
QAll = zeros((numIntegrationSteps),length(Q));

inputCommandMat = [0,1,0;
                   Umax(1),1,1;
                   -Umax(1),1,11;
                   0,0,21];
[commandMat] = movementCommands(timeToRun,DT,inputCommandMat);
tauVMat = [0:0.4:2];
f2 = figure();
a2 = axes(f2);
hold on;

for i = 1:length(tauVMat)
    tau_v = tauVMat(i);
    Q = Q0;
for j = 1:numTimesteps
    U = commandMat(j,1:2)';
    [QNext] = robot_bike_dyn(Q,U,Umin,Umax,Qmin,Qmax,L,tau_gamma,tau_v);
    Q = QNext(end,:)';
    QAll((j-1)*integrationStepsPerTimeStep+1:(j-1)*integrationStepsPerTimeStep+integrationStepsPerTimeStep,:) = QNext;
end


lineLength = linspace(0,1);
thetaLine = [QAll(end,1),QAll(end,2)]+[(lineLength*cos(QAll(end,3)))',(lineLength*sin(QAll(end,3)))'];
plot(QAll(:,1),QAll(:,2),'LineWidth',2,'DisplayName',strcat("Path: [\tau_\gamma, \tau_v]: [", num2str([tau_gamma]), ', ',num2str([tau_v]),']'));
%plot(thetaLine(:,1),thetaLine(:,2),'DisplayName','Direction of Travel');

theta = linspace(0,2*pi)';
%desiredPath = rD*[cos(theta),sin(theta)]+[8,10];
%plot(desiredPath(:,1),desiredPath(:,2),'DisplayName','Desired Path')
%scatter(Q0(1),Q0(2),100,'o','DisplayName','Initial Position');
%scatter(QAll(end,1),QAll(end,2),100,'*','DisplayName','Final Position');
end
xlabel('X []');
ylabel('Y []');
title("\tau_\gamma = 0, \tau_v = [0,0.4,0.8,1.2,1.6,2]");
legend
axis equal

%C Part 2
dt = 0.01;
DT = 0.1;

Qmin = [-inf,-inf,-inf,-pi/4,-5]';
Qmax = -Qmin;
Umin = [Qmin(4),Qmin(5)]';
Umax = -Umin;
L = 2.5;
tau_gamma = 0.0;
tau_v = 0.0;

% x y theta gamma v
Q0 = [10;10;pi/2;0;0];
% gammaD VD
U0 = [0;1];
Q = Q0;
U = U0;

timeToRun = 21;
numTimesteps = timeToRun/DT;
integrationStepsPerTimeStep = DT/dt;
numIntegrationSteps = round(numTimesteps*integrationStepsPerTimeStep);
QAll = zeros((numIntegrationSteps),length(Q));

inputCommandMat = [0,1,0;
                   Umax(1),1,1;
                   -Umax(1),1,11;
                   0,0,21];
[commandMat] = movementCommands(timeToRun,DT,inputCommandMat);
tauVMat = [0:0.4:2];
tauGammaMat = [0:0.4:2];

f2 = figure();
a2 = axes(f2);
hold on;

for i = 1:length(tauGammaMat)
    tau_gamma = tauGammaMat(i);
    Q = Q0;
for j = 1:numTimesteps
    U = commandMat(j,1:2)';
    [QNext] = robot_bike_dyn(Q,U,Umin,Umax,Qmin,Qmax,L,tau_gamma,tau_v);
    Q = QNext(end,:)';
    QAll((j-1)*integrationStepsPerTimeStep+1:(j-1)*integrationStepsPerTimeStep+integrationStepsPerTimeStep,:) = QNext;
end


lineLength = linspace(0,1);
thetaLine = [QAll(end,1),QAll(end,2)]+[(lineLength*cos(QAll(end,3)))',(lineLength*sin(QAll(end,3)))'];
plot(QAll(:,1),QAll(:,2),'LineWidth',2,'DisplayName',strcat("Path: [\tau_\gamma, \tau_v]: [", num2str([tau_gamma]), ', ',num2str([tau_v]),']'));
%plot(thetaLine(:,1),thetaLine(:,2),'DisplayName','Direction of Travel');

theta = linspace(0,2*pi)';
%desiredPath = rD*[cos(theta),sin(theta)]+[8,10];
%plot(desiredPath(:,1),desiredPath(:,2),'DisplayName','Desired Path')
%scatter(Q0(1),Q0(2),100,'o','DisplayName','Initial Position');
%scatter(QAll(end,1),QAll(end,2),100,'*','DisplayName','Final Position');
end

xlabel('X []');
ylabel('Y []');
title("\tau_\gamma = [0,0.4,0.8,1.2,1.6,2], \tau_v = 0");
legend
axis equal





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








































