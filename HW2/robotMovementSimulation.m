close all;
clear;


global dt DT

%test
dt = 0.001;
DT = 0.01;

% %Part B
% 
% Qmin = [-inf,-inf,-inf,-pi/4,-1]';
% Qmax = -Qmin;
% Umin = [Qmin(4),Qmin(5)]';
% Umax = -Umin;
% L = 2.5;
% tau_gamma = 0.0;
% tau_v = 0.0;
% 
% a = 0: 0.01 : 2*pi;  % angle step for points on circular path 
% xPath = 9 + 5*sin(a); yPath = 7-5*cos(a);
% 
% % x y theta gamma v
% Q0 = [15;5;pi/2;0;0];
% % gammaD VD
% U0 = [0;1];
% Q = Q0;
% U = U0;
% 
% timeToRun = 60;
% numTimesteps = timeToRun/DT;
% integrationStepsPerTimeStep = DT/dt;
% numIntegrationSteps = round(numTimesteps*integrationStepsPerTimeStep);
% QAll = zeros((numIntegrationSteps),length(Q));
% % 
% % inputCommandMat = [0,1,0;
% %                    Umax(1),1,1;
% %                    -Umax(1),1,11;
% %                    0,0,21];
% % [commandMat] = movementCommands(timeToRun,DT,inputCommandMat);
% 
% Ld = 2;
% 
% path = [xPath;yPath];
% crossTrackErrorMat = zeros(numTimesteps,2);
% 
% for j = 1:numTimesteps
% 
%      [gammaD,endDistance,crossTrackError,crossTrackErrorInterpolated] = purePursuit(Q,L,Ld,path);
%     crossTrackErrorMat(j,1)=crossTrackError;
%     crossTrackErrorMat(j,2)=crossTrackErrorInterpolated;
%     if endDistance<0.3
%         QAll = QAll(1:(j-1)*integrationStepsPerTimeStep,:);
%         crossTrackErrorMat=crossTrackErrorMat(1:j,:);
%         break
%     end
% 
%     U = [gammaD;1];
%     [QNext] = robot_bike_dyn(Q,U,Umin,Umax,Qmin,Qmax,L,tau_gamma,tau_v);
%     Q = QNext(end,:)';
%     QAll((j-1)*integrationStepsPerTimeStep+1:(j-1)*integrationStepsPerTimeStep+integrationStepsPerTimeStep,:) = QNext;
% 
% 
% 
% end
% 
% f2 = figure();
% a2 = axes(f2);
% lineLength = linspace(0,1);
% thetaLine = [QAll(end,1),QAll(end,2)]+[(lineLength*cos(QAll(end,3)))',(lineLength*sin(QAll(end,3)))'];
% hold on;
% 
% 
% theta = linspace(0,2*pi)';
% %desiredPath = rD*[cos(theta),sin(theta)]+[8,10];
% %plot(desiredPath(:,1),desiredPath(:,2),'DisplayName','Desired Path')
% 
% plot(QAll(:,1),QAll(:,2),'LineWidth',2,'DisplayName','Path');
% 
% plot(xPath,yPath,'--','LineWidth',1,'DisplayName','Desired Path');
% plot(thetaLine(:,1),thetaLine(:,2),'LineWidth',2,'DisplayName','Ending Direction of Travel');
% scatter(Q0(1),Q0(2),100,'o','DisplayName','Initial Position');
% scatter(QAll(end,1),QAll(end,2),100,'*','DisplayName','Final Position');
% 
% % plot(QAll(:,1),QAll(:,2),'LineWidth',2,'DisplayName','Path');
% % plot(thetaLine(:,1),thetaLine(:,2),'LineWidth',2,'DisplayName','Direction of Travel');
% % plot(desiredPath(:,1),desiredPath(:,2),'--','LineWidth',1,'DisplayName','Desired Path')
% 
% 
% xlabel('X []');
% ylabel('Y []');
% title("\tau_\gamma = 0, \tau_v = 0");
% 
% legend
% axis equal
% 
% 
% 
% f3 = figure();
% a3 = axes(f3);
% hold on;
% plot([0:DT:(length(crossTrackErrorMat)-1)*DT],crossTrackErrorMat(:,1),DisplayName='Cross Track Error From Given Points')
% plot([0:DT:(length(crossTrackErrorMat)-1)*DT],crossTrackErrorMat(:,2),DisplayName='Cross Track Error Interpolated')
% 
% xlabel('Time [s]');
% ylabel('Cross Track Error [m]');
% title("Cross Track Error vs. Time");
% 
% legend









% % Part C 1
% 
% Qmin = [-inf,-inf,-inf,-pi/4,-1]';
% Qmax = -Qmin;
% Umin = [Qmin(4),Qmin(5)]';
% Umax = -Umin;
% L = 2.5;
% tau_gamma = 0.0;
% tau_v = 0.0;
% 
% % a = 0: 0.01 : 2*pi;  % angle step for points on circular path 
% % xPath = 9 + 5*sin(a); yPath = 7-5*cos(a);
% 
% 
% xPath= [linspace(0,10),linspace(10,10),linspace(10,20)];
% yPath= [linspace(0,0),linspace(0,5),linspace(5,5)];
% 
% 
% 
% % x y theta gamma v
% Q0 = [0;0;0;0;0];
% % gammaD VD
% U0 = [0;1];
% Q = Q0;
% U = U0;
% 
% timeToRun = 60;
% numTimesteps = timeToRun/DT;
% integrationStepsPerTimeStep = DT/dt;
% numIntegrationSteps = round(numTimesteps*integrationStepsPerTimeStep);
% QAll = zeros((numIntegrationSteps),length(Q));
% % 
% % inputCommandMat = [0,1,0;
% %                    Umax(1),1,1;
% %                    -Umax(1),1,11;
% %                    0,0,21];
% % [commandMat] = movementCommands(timeToRun,DT,inputCommandMat);
% 
% f2 = figure();
% a2 = axes(f2);
% hold on;
% axis equal
% plot(a2,xPath,yPath,'--','LineWidth',1,'DisplayName','Desired Path');
% 
% 
% 
% f3 = figure();
% a3 = axes(f3);
% hold on;
% 
% f4 = figure();
% a4 = axes(f4);
% hold on;
% 
% 
% 
% LdMat = [0.5:0.5:3.5];
% colorMat = ['rgbmckyw'];
% for i = 1:length(LdMat)
%     color =colorMat(i);
% path = [xPath;yPath];
% crossTrackErrorMat = zeros(numTimesteps,2);
% Ld = LdMat(i);
% Q=Q0;
% for j = 1:numTimesteps
%      [gammaD,endDistance,crossTrackError,crossTrackErrorInterpolated] = purePursuit(Q,L,Ld,path);
%     crossTrackErrorMat(j,1)=crossTrackError;
%     crossTrackErrorMat(j,2)=crossTrackErrorInterpolated;
%     if endDistance<0.35
%         QAll = QAll(1:(j-1)*integrationStepsPerTimeStep,:);
%         crossTrackErrorMat=crossTrackErrorMat(1:j,:);
%         break
%     end
%     U = [gammaD;1];
%     [QNext] = robot_bike_dyn(Q,U,Umin,Umax,Qmin,Qmax,L,tau_gamma,tau_v);
%     Q = QNext(end,:)';
%     QAll((j-1)*integrationStepsPerTimeStep+1:(j-1)*integrationStepsPerTimeStep+integrationStepsPerTimeStep,:) = QNext;
% end
% 
% 
% lineLength = linspace(0,1);
% thetaLine = [QAll(end,1),QAll(end,2)]+[(lineLength*cos(QAll(end,3)))',(lineLength*sin(QAll(end,3)))'];
% hold on;
% 
% 
% theta = linspace(0,2*pi)';
% %desiredPath = rD*[cos(theta),sin(theta)]+[8,10];
% %plot(desiredPath(:,1),desiredPath(:,2),'DisplayName','Desired Path')
% 
% plot(a2,QAll(:,1),QAll(:,2),'LineWidth',1.5,'DisplayName',strcat("Path for: L_d = ",num2str(Ld)));
% 
% %plot(a2,thetaLine(:,1),thetaLine(:,2),'LineWidth',2,'DisplayName','Ending Direction of Travel');
% %scatter(a2,Q0(1),Q0(2),100,'o','DisplayName','Initial Position');
% %scatter(a2,QAll(end,1),QAll(end,2),100,'*','DisplayName','Final Position');
% 
% % plot(QAll(:,1),QAll(:,2),'LineWidth',2,'DisplayName','Path');
% % plot(thetaLine(:,1),thetaLine(:,2),'LineWidth',2,'DisplayName','Direction of Travel');
% % plot(desiredPath(:,1),desiredPath(:,2),'--','LineWidth',1,'DisplayName','Desired Path')
% 
% 
% xlabel(a2,'X []');
% ylabel(a2,'Y []');
% title(a2,"Comparing L_d Effects on Path");
% 
% legend(a2)
% 
% 
% %plot(a3,[0:DT:(length(crossTrackErrorMat)-1)*DT],crossTrackErrorMat(:,1),DisplayName='Cross Track Error From Given Points')
% plot(a3,[0:DT:(length(crossTrackErrorMat)-1)*DT],crossTrackErrorMat(:,2),DisplayName=strcat("Cross Track Error  for: L_d = ",num2str(Ld)))
% 
% xlabel(a3,'Time [s]');
% ylabel(a3,'Cross Track Error [m]');
% title(a3,"Cross Track Error vs. Time");
% 
% legend(a3)
% 
% 
% bar(a4, strcat("Max for: L_d = ",num2str(Ld)), max(crossTrackErrorMat(:,2)),color)
% bar(a4, strcat("Mean for: L_d = ",num2str(Ld)), mean(crossTrackErrorMat(:,2)),color)
% bar(a4, strcat("95% for: L_d = ",num2str(Ld)), prctile(crossTrackErrorMat(:,2),95),color)
% bar(a4, strcat("RMS for: L_d = ",num2str(Ld)), rms(crossTrackErrorMat(:,2)),color)
% ylabel(a4,'Cross Track Error [m]');
% title(a4,"Cross Track Error Statistics for Various L_d");
% 
% if Ld == 2
%     f5 = figure();
%     a5 = axes(f5);
%     hold on;
%     histogram(crossTrackErrorMat(:,2))
%     xlabel(a5,'Cross Track Error [m]');
%     ylabel(a5,'Instances []');
%     title(a5,"Cross Track Error Histogram for L_d = 2 m");
% end
% 
% end


% % Part C 2
% 
% Qmin = [-inf,-inf,-inf,-pi/4,-2]';
% Qmax = -Qmin;
% Umin = [Qmin(4),Qmin(5)]';
% Umax = -Umin;
% L = 2.5;
% tau_gamma = 0.0;
% tau_v = 0.0;
% 
% % a = 0: 0.01 : 2*pi;  % angle step for points on circular path 
% % xPath = 9 + 5*sin(a); yPath = 7-5*cos(a);
% 
% 
% xPath= [linspace(0,10),linspace(10,10),linspace(10,20)];
% yPath= [linspace(0,0),linspace(0,5),linspace(5,5)];
% 
% 
% 
% % x y theta gamma v
% Q0 = [0;0;0;0;0];
% % gammaD VD
% U0 = [0;1];
% Q = Q0;
% U = U0;
% 
% timeToRun = 60;
% numTimesteps = timeToRun/DT;
% integrationStepsPerTimeStep = DT/dt;
% numIntegrationSteps = round(numTimesteps*integrationStepsPerTimeStep);
% QAll = zeros((numIntegrationSteps),length(Q));
% % 
% % inputCommandMat = [0,1,0;
% %                    Umax(1),1,1;
% %                    -Umax(1),1,11;
% %                    0,0,21];
% % [commandMat] = movementCommands(timeToRun,DT,inputCommandMat);
% 
% f2 = figure();
% a2 = axes(f2);
% hold on;
% axis equal
% plot(a2,xPath,yPath,'--','LineWidth',1,'DisplayName','Desired Path');
% 
% 
% 
% f3 = figure();
% a3 = axes(f3);
% hold on;
% 
% f4 = figure();
% a4 = axes(f4);
% hold on;
% 
% 
% 
% LdMat = [0.5:0.5:3.5];
% VMat = [1,2];
% colorMat = ['rgbmckyw'];
% for i = 1:length(VMat)
%     color =colorMat(i);
% path = [xPath;yPath];
% crossTrackErrorMat = zeros(numTimesteps,2);
% %Ld = LdMat(i);
% Ld = 2;
% Q=Q0;
% for j = 1:numTimesteps
%      [gammaD,endDistance,crossTrackError,crossTrackErrorInterpolated] = purePursuit(Q,L,Ld,path);
%     crossTrackErrorMat(j,1)=crossTrackError;
%     crossTrackErrorMat(j,2)=crossTrackErrorInterpolated;
%     if endDistance<0.35
%         QAll = QAll(1:(j-1)*integrationStepsPerTimeStep,:);
%         crossTrackErrorMat=crossTrackErrorMat(1:j,:);
%         break
%     end
%     V = VMat(i);
%     U = [gammaD;V];
%     [QNext] = robot_bike_dyn(Q,U,Umin,Umax,Qmin,Qmax,L,tau_gamma,tau_v);
%     Q = QNext(end,:)';
%     QAll((j-1)*integrationStepsPerTimeStep+1:(j-1)*integrationStepsPerTimeStep+integrationStepsPerTimeStep,:) = QNext;
% end
% 
% 
% lineLength = linspace(0,1);
% thetaLine = [QAll(end,1),QAll(end,2)]+[(lineLength*cos(QAll(end,3)))',(lineLength*sin(QAll(end,3)))'];
% hold on;
% 
% 
% theta = linspace(0,2*pi)';
% %desiredPath = rD*[cos(theta),sin(theta)]+[8,10];
% %plot(desiredPath(:,1),desiredPath(:,2),'DisplayName','Desired Path')
% 
% plot(a2,QAll(:,1),QAll(:,2)-((V-1)/8),'LineWidth',1.5,'DisplayName',strcat("Path for: L_d = ",num2str(Ld)));
% 
% %plot(a2,thetaLine(:,1),thetaLine(:,2),'LineWidth',2,'DisplayName','Ending Direction of Travel');
% %scatter(a2,Q0(1),Q0(2),100,'o','DisplayName','Initial Position');
% %scatter(a2,QAll(end,1),QAll(end,2),100,'*','DisplayName','Final Position');
% 
% % plot(QAll(:,1),QAll(:,2),'LineWidth',2,'DisplayName','Path');
% % plot(thetaLine(:,1),thetaLine(:,2),'LineWidth',2,'DisplayName','Direction of Travel');
% % plot(desiredPath(:,1),desiredPath(:,2),'--','LineWidth',1,'DisplayName','Desired Path')
% 
% 
% xlabel(a2,'X []');
% ylabel(a2,'Y []');
% title(a2,"Comparing L_d Effects on Path");
% 
% legend(a2)
% 
% 
% %plot(a3,[0:DT:(length(crossTrackErrorMat)-1)*DT],crossTrackErrorMat(:,1),DisplayName='Cross Track Error From Given Points')
% plot(a3,[0:DT:(length(crossTrackErrorMat)-1)*DT],crossTrackErrorMat(:,2),DisplayName=strcat("Cross Track Error  for: V = ",num2str(V)))
% 
% xlabel(a3,'Time [s]');
% ylabel(a3,'Cross Track Error [m]');
% title(a3,"Cross Track Error vs. Time");
% 
% legend(a3)
% 
% 
% bar(a4, strcat("Max for: L_d = ",num2str(V)), max(crossTrackErrorMat(:,2)),color)
% bar(a4, strcat("Mean for: L_d = ",num2str(V)), mean(crossTrackErrorMat(:,2)),color)
% bar(a4, strcat("95% for: L_d = ",num2str(V)), prctile(crossTrackErrorMat(:,2),95),color)
% bar(a4, strcat("RMS for: L_d = ",num2str(V)), rms(crossTrackErrorMat(:,2)),color)
% ylabel(a4,'Cross Track Error [m]');
% title(a4,"Cross Track Error Statistics for Various V");
% 
% % if Ld == 2
% %     f5 = figure();
% %     a5 = axes(f5);
% %     hold on;
% %     histogram(crossTrackErrorMat(:,2))
% %     xlabel(a5,'Cross Track Error [m]');
% %     ylabel(a5,'Instances []');
% %     title(a5,"Cross Track Error Histogram for L_d = 2 m");
% % end
% 
% end



% Part D



% a = 0: 0.01 : 2*pi;  % angle step for points on circular path 
% xPath = 9 + 5*sin(a); yPath = 7-5*cos(a);


xPath= [linspace(0,10),linspace(10,10),linspace(10,20)];
yPath= [linspace(0,0),linspace(0,5),linspace(5,5)];

Qmin = [-inf,-inf,-inf,-pi/4,-1]';
Qmax = -Qmin;
Umin = [Qmin(4),Qmin(5)]';
Umax = -Umin;
L = 2.5;
tau_gamma = 0.0;
tau_v = 0.0;

% x y theta gamma v
Q0 = [0;0;0;0;0];
% gammaD VD
U0 = [0;1];
Q = Q0;
U = U0;

timeToRun = 60;
numTimesteps = timeToRun/DT;
integrationStepsPerTimeStep = DT/dt;
numIntegrationSteps = round(numTimesteps*integrationStepsPerTimeStep);
QAll = zeros((numIntegrationSteps),length(Q));

f2 = figure();
a2 = axes(f2);
hold on;
axis equal
plot(a2,xPath,yPath,'--','LineWidth',1,'DisplayName','Desired Path');

f3 = figure();
a3 = axes(f3);
hold on;

f4 = figure();
a4 = axes(f4);
hold on;

f5 = figure();
a5 = axes(f5);
hold on;

LdMat = [0.5:0.5:3.5];
colorMat = ['rgbmckyw'];
tauMat = [0,0;0.15,0.5;0.3,0.5;0,0;0.15,0.5;0.3,0.5];
gammaLimMat = [-pi/4,-pi/4,-pi/4,-deg2rad(35),-deg2rad(35),-deg2rad(35),];
for i = 1:6

Ld = 2;
    color =colorMat(i);
path = [xPath;yPath];
crossTrackErrorMat = zeros(numTimesteps,2);

tau_gamma = tauMat(i,1);
tau_v = tauMat(i,1);

Umin(1) = gammaLimMat(i);
Umax = -Umin;

% Qmin = [-inf,-inf,-inf,-pi/4,-1]';
% Qmax = -Qmin;
% Umin = [Qmin(4),Qmin(5)]';
% Umax = -Umin;

%Ld = LdMat(i);
Q=Q0;
for j = 1:numTimesteps
     [gammaD,endDistance,crossTrackError,crossTrackErrorInterpolated] = purePursuit(Q,L,Ld,path);
    crossTrackErrorMat(j,1)=crossTrackError;
    crossTrackErrorMat(j,2)=crossTrackErrorInterpolated;
    if endDistance<0.35
        QAll = QAll(1:(j-1)*integrationStepsPerTimeStep,:);
        crossTrackErrorMat=crossTrackErrorMat(1:j,:);
        break
    end
    U = [gammaD;1];
    [QNext] = robot_bike_dyn(Q,U,Umin,Umax,Qmin,Qmax,L,tau_gamma,tau_v);
    Q = QNext(end,:)';
    QAll((j-1)*integrationStepsPerTimeStep+1:(j-1)*integrationStepsPerTimeStep+integrationStepsPerTimeStep,:) = QNext;
end


lineLength = linspace(0,1);
thetaLine = [QAll(end,1),QAll(end,2)]+[(lineLength*cos(QAll(end,3)))',(lineLength*sin(QAll(end,3)))'];
hold on;


theta = linspace(0,2*pi)';
%desiredPath = rD*[cos(theta),sin(theta)]+[8,10];
%plot(desiredPath(:,1),desiredPath(:,2),'DisplayName','Desired Path')

limStringMat = ["\pi/4";"35\circ"];
plot(a2,QAll(:,1),QAll(:,2),'LineWidth',1.5,'DisplayName',strcat("Path for: \tau_\gamma = ",num2str(tau_gamma), ", \tau_v = ", num2str(tau_v),", \gamma_{lim} = ",limStringMat(floor(i/3.1)+1)));

%plot(a2,thetaLine(:,1),thetaLine(:,2),'LineWidth',2,'DisplayName','Ending Direction of Travel');
%scatter(a2,Q0(1),Q0(2),100,'o','DisplayName','Initial Position');
%scatter(a2,QAll(end,1),QAll(end,2),100,'*','DisplayName','Final Position');

% plot(QAll(:,1),QAll(:,2),'LineWidth',2,'DisplayName','Path');
% plot(thetaLine(:,1),thetaLine(:,2),'LineWidth',2,'DisplayName','Direction of Travel');
% plot(desiredPath(:,1),desiredPath(:,2),'--','LineWidth',1,'DisplayName','Desired Path')


xlabel(a2,'X []');
ylabel(a2,'Y []');
title(a2,"Comparing L_d Effects on Path");

legend(a2)


%plot(a3,[0:DT:(length(crossTrackErrorMat)-1)*DT],crossTrackErrorMat(:,1),DisplayName='Cross Track Error From Given Points')
plot(a3,[0:DT:(length(crossTrackErrorMat)-1)*DT],crossTrackErrorMat(:,2),'LineWidth',1.5,DisplayName=strcat("Cross Track Error for: \tau_\gamma = ",num2str(tau_gamma), ", \tau_v = ", num2str(tau_v),", \gamma_{lim} = ",limStringMat(floor(i/3.1)+1)))

xlabel(a3,'Time [s]');
ylabel(a3,'Cross Track Error [m]');
title(a3,"Cross Track Error vs. Time");

legend(a3)

if any([i==1,i==4])
bar(a4, strcat("\tau_\gamma = ",num2str(tau_gamma), ", \tau_v = ", num2str(tau_v),", \gamma_{lim} = ",limStringMat(floor(i/3.1)+1)), max(crossTrackErrorMat(:,2)),color)
% bar(a4, strcat("Mean for: L_d = ",num2str(Ld)), mean(crossTrackErrorMat(:,2)),color)
% bar(a4, strcat("95% for: L_d = ",num2str(Ld)), prctile(crossTrackErrorMat(:,2),95),color)
% bar(a4, strcat("RMS for: L_d = ",num2str(Ld)), rms(crossTrackErrorMat(:,2)),color)
ylabel(a4,'Cross Track Error [m]');
title(a4,"Cross Track Error for Changing \gamma_{lim}");
end
if any([i==1,i==2, i==3])
bar(a5, strcat("\tau_\gamma = ",num2str(tau_gamma), ", \tau_v = ", num2str(tau_v),", \gamma_{lim} = ",limStringMat(floor(i/3.1)+1)), max(crossTrackErrorMat(:,2)),color)
% bar(a4, strcat("Mean for: L_d = ",num2str(Ld)), mean(crossTrackErrorMat(:,2)),color)
% bar(a4, strcat("95% for: L_d = ",num2str(Ld)), prctile(crossTrackErrorMat(:,2),95),color)
% bar(a4, strcat("RMS for: L_d = ",num2str(Ld)), rms(crossTrackErrorMat(:,2)),color)
ylabel(a5,'Cross Track Error [m]');
title(a5,"Cross Track Error for Changing \tau_\gamma, \tau_v");
end
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











































