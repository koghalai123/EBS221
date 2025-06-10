%% final project main
% BR and KO
clear all;
close all;
clc;


%% global variables

global bitmap

global bitmaplaser

global W gamma_max L M K D X_first_tree Y_first_tree

global dt DT


%% add path
addpath 'C:\Users\kogha\OneDrive - University of California, Davis\Documents\My Documents\PhDClasses\3rd Quarter\EBS221\uploadedFiles\FINAL PROJECT/additionalFiles'
addpath 'C:\Users\kogha\OneDrive - University of California, Davis\Documents\My Documents\PhDClasses\3rd Quarter\EBS221\uploadedFiles\FINAL PROJECT/additionalFiles/geom2d/geom2d'
addpath 'C:\Users\kogha\OneDrive - University of California, Davis\Documents\My Documents\PhDClasses\3rd Quarter\EBS221\uploadedFiles\FINAL PROJECT'

%% variables
% gen nursery variables
X_first_tree = 20;
Y_first_tree = 20;
R = 500; C = 500; %numbers of rows and columns of bitmap
K = 5; % number of tree rows running south-north direction
M = 7; %maximum number of trees in each row
W = 3; % distance between tree rows (m)
D = 2; %distance between trees in the same row (m)
bitmap = zeros(R, C); %initialize as empty
[ Xmax, Ymax, x, y] = generateNurseryFunction(X_first_tree,Y_first_tree, M, K, D,W, R,C);


%%
% robot variables
gamma_max = deg2rad(55);
L = 3;
Qmin = [-inf,-inf,-inf,-gamma_max,-2]';
Qmax = -Qmin;
tau_gamma = 0.1;
tau_v = 0.1;
Q0 = [15;15;pi/2;0;0];
Qend = [0;0;0;0;0]; %[0;40;pi/2];
Umin = [Qmin(4),Qmin(5)]';
Umax = -Umin;
vD = 1; % speed of robot
U0 = [0;0];%[0;1];
U = U0;
Rmin = L/tan(gamma_max);
Ld = 0.2*Rmin;

% laser variables
angleSpan = deg2rad(180);
angleStep = deg2rad(0.125);%angleSpan/720;%
rangeMax = 20;%20;
bitmaplaser = 0.5* ones(R, C); %initialize as unknown-occupancy pixels

% sim variables

DT = 0.01;
timeToRun = 500;
numTimesteps = timeToRun/DT;





%% path gen

[desiredPath] = OptPathGen(Q0,Qend);
x_path = desiredPath(1,:);
y_path = desiredPath(2,:);

%% Prepare for EKF
[odoCov,GPSCov] = estimateCovariance();


syms xSym ySym thetaSym gammaDesSym VDesSym DTSym LSym tau_vSym tau_gammaSym VD gammaD v1 v2 v3 h1 h2 h3

QSym = [xSym;ySym;thetaSym];
USym = [gammaDesSym;VDesSym];
odoSym = [v1;v2];
noiseSym = [h1;h2;h3];
gammaActSym = gammaDesSym;
travelDistanceDes = VDesSym*DTSym;
thetaChangeDes = travelDistanceDes*tan(gammaActSym)/LSym;

xNewSym = xSym+(travelDistanceDes+v1)*cos(thetaSym);
yNewSym = ySym+(travelDistanceDes+v1)*sin(thetaSym);
thetaNewSym = thetaSym+thetaChangeDes+v2;
QNewSym = [xNewSym;yNewSym;thetaNewSym]+noiseSym;


FqSym = subs(jacobian(QNewSym,QSym),odoSym,[0;0]);
FuSym = jacobian(QNewSym,[USym]);
FvSym = jacobian(QNewSym,[odoSym]);

HqSym = subs(jacobian(QNewSym,[QSym]),odoSym,[0;0]);
HwSym = jacobian(QNewSym,[noiseSym]);
H = eye(3); 
P = eye(3);

disturbanceCov = odoCov;
sensorCov = GPSCov;


angleStep = deg2rad(2);

t = cputime;
crossTrackErrorMat = zeros(numTimesteps,2);
Q=Q0;
f1 = figure(1);
a1 = axes(f1);
pointOnPath = 1;
path = [x_path;y_path];
Q_estimate = Q0;
z = Q0(1:3);


QTrueMat = zeros(numTimesteps, length(Q));
QMeasMat = zeros(numTimesteps, 3);
QEstMat = zeros(numTimesteps, 3);
QEstMatFiltered = zeros(numTimesteps, 3);

subsVecSym = [QSym; USym; tau_vSym; tau_gammaSym; DTSym; LSym; v1; v2; v3; h1; h2; h3];
subsVecNum = [Q_estimate(1:3); U; tau_v; tau_gamma; DT; L; 0; 0; 0; 0; 0; 0];
Hw = eye(3);
Fq = double(subs(FqSym, subsVecSym, subsVecNum));
Fv = double(subs(FvSym, subsVecSym, subsVecNum));
Hq = double(subs(HqSym, subsVecSym, subsVecNum));
for j = 1:numTimesteps
    tic
    [slicedPath,pointOnPath] = pathSlicer(pointOnPath,path,Ld,Q);
    [gammaD,endDistance,crossTrackError,crossTrackErrorInterpolated] = purePursuit(Q,L,Ld,slicedPath);
    crossTrackErrorMat(j,1)=crossTrackError;
    crossTrackErrorMat(j,2)=crossTrackErrorInterpolated;
    
    U = [gammaD;vD];%[gammaD;1];
    
    %[QNext] = robot_bike_dyn(Q,U,Umin,Umax,Qmin,Qmax,L,tau_gamma,tau_v);
    %Q
    [QTrue, odometryInfo] = robot_odo(Q, U, Umin, Umax,Qmin, Qmax, L, tau_gamma, tau_v);
    
    Q = QTrue; 
    dQOdo = [cos(Q_estimate(3))*odometryInfo(1);sin(Q_estimate(3))*odometryInfo(1);odometryInfo(2)];
    Q_estimate = Q_estimate(1:3)+dQOdo;

    %Do this only every second when a new GPS measurement is acquired
    if mod(j,1/DT)==0
        [ xGPS, yGPS, theta_GPS ] = GPS_CompassNoisy( Q(1), Q(2), Q(3) );
        z = [ xGPS, yGPS, theta_GPS ]';
    
        Fq = [[1, 0, -DT*U(2)*sin(Q_estimate(3))];
            [0, 1,  DT*U(2)*cos(Q_estimate(3))];
            [0, 0,                            1]];
        Fv = [[cos(Q_estimate(3)), 0];
            [sin(Q_estimate(3)), 0];
            [            0, 1]];
        Hq = [[1, 0, -DT*U(2)*sin(Q_estimate(3))];
            [0, 1,  DT*U(2)*cos(Q_estimate(3))];
            [0, 0,                            1]];
        Hw = eye(3);

        
         [Q_estimate,z,P] = EKFStep(Q_estimate,Fq,Fv,Hw,Hq,QNewSym,subsVecSym,subsVecNum,z,P,H,disturbanceCov,sensorCov) ;
        QEstMatFiltered(floor(j/(1/DT)),:) = Q_estimate';



        % laser scan and update scan map
        Tl = SE2([Q(1),Ymax-Q(2),-Q(3)]);     % lateral motion along the x-axis

        p = laserScannerNoisy(angleSpan, angleStep, rangeMax, Tl.T, bitmap, Xmax, Ymax);

        filteredRange = medfilt1(p(:,2),5,'omitnan','truncate');

        for i=1:length(p)
            angle = p(i,1);
            % range = p(i,2);
            range = filteredRange(i,1);
            % possible infinite range is handled inside the update function
            n = updateLaserBeamGrid(angle, range, Tl.T, R, C, Xmax, Ymax);
        end
    


    end

    
    %QAll((j-1)*integrationStepsPerTimeStep+1:(j-1)*integrationStepsPerTimeStep+integrationStepsPerTimeStep,:) = QNext;
    % ends loop if runs for too long
    E = cputime-t;
    
    
    % clf(a1)
    % hold(a1,'on');
    % imagesc(a1,[0 Xmax], [0 Ymax],bitmaplaser./(1+bitmaplaser)); title('Binary occupancy grid')
    % plot(a1,QAll(:,1),QAll(:,2))
    % drawnow

    QTrueMat(j,:) = QTrue';
    QMeasMat(j,:) = z';
    QEstMat(j,:) = Q_estimate';

    if j>numTimesteps/10 && endDistance<0.35
        QTrueMat = QTrueMat(1:j,:);
        QMeasMat = QMeasMat(1:j,:);
        QEstMat = QEstMat(1:j,:);
        QEstMatFiltered = QEstMatFiltered(1:floor(j/(1/DT)),:);

        crossTrackErrorMat=crossTrackErrorMat(1:j,:);
        break
    end

    disp(toc);
end
E = cputime-t


%% find trees and estimate radius
fileID = fopen('results.txt','w');
fileIDTrue = fopen('true.txt','w');

% MeasureRadius = zeros(M, K);
% MeasureX(j,i) = zeros(M, K);
% MeasureY(j,i) = zeros(M, K);
r_check = 0.75;
for i=1:K
    fprintf(fileID,'%1d\n',i);
    fprintf(fileIDTrue,'%1d\n',i);
    m = 1;
    for j=1:M
        j;
        [XC, YC, RadiusC] = findTree(x(j,i),y(j,i),r_check,Xmax, Ymax, R, C);
        MeasureX(j,i) = XC;
        MeasureY(j,i) = YC;
        MeasureRadius(j,i) = RadiusC;
        if RadiusC>=0.1
            fprintf(fileID,'%1d, %6.4f, %6.4f, %6.5f\n',m,XC,YC,RadiusC);
            fprintf(fileIDTrue,'%1d, %6.4f, %6.4f, %6.5f\n',m,x(j,i),y(j,i),RadiusC);
            m=m+1;
        end      
    end
    fprintf(fileID,'\n');
    fprintf(fileIDTrue,'\n');
end


figure(1);
clf
hold on
imagesc([0 Xmax], [0 Ymax],bitmaplaser./(1+bitmaplaser)); 
title('Binary occupancy grid')
colorbar
scatter(x,y,'xr')
plot(x_path,y_path)
plot(QTrueMat(:,1),QTrueMat(:,2))

%%
figure(2);
clf
hold on
imagesc([0 Xmax], [0 Ymax], bitmap); %imagesc flips the bitmap rows, so correct this
set(gca,'YDir','normal');
colorbar
axis equal
scatter(x,y,'xr')





f3 = figure(3);
a3 = axes(f3);
hold on;
axis equal
plot(QTrueMat(:,1),QTrueMat(:,2),DisplayName="True Locations",Color='r')
plot(QMeasMat(:,1),QMeasMat(:,2),DisplayName="Measured Locations",Color='g')
plot(QEstMat(:,1),QEstMat(:,2),DisplayName="Estimated Locations",Color='b')
plot(QEstMatFiltered(:,1),QEstMatFiltered(:,2),DisplayName="Filtered Estimated Locations",Color='m')

legend
