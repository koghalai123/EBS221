%% final project main
% BR and KO
clear all;
close all;
clc;


%% global variables

global bitmap

global bitmaplaser

global W gamma_max L M K D X_first_tree Y_first_tree TreeRadius

global dt DT minTreeRadius maxTreeRadius


%% add path
addpath 'additionalFiles'
addpath 'additionalFiles/geom2d/geom2d'

%% variables
% gen nursery variables
X_first_tree = 20;
Y_first_tree = 20;
R = 1500; C = 1500; %numbers of rows and columns of bitmap
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
Q0 = [5;5;pi/2;0;0];
Qend = [5;5;0;0;0]; %[0;40;pi/2]; 
% having it start and end at (5,5) prevents the noise from making it
% negative and causing a bunch of issues

Umin = [Qmin(4),Qmin(5)]';
Umax = -Umin;
vD = 1; % speed of robot
U0 = [0;0];%[0;1];
U = U0;
Rmin = L/tan(gamma_max);
Ld = 0.2*Rmin;

% laser variables
angleSpan = deg2rad(180);
angleStep = deg2rad(0.125);%deg2rad(0.125);%angleSpan/720;%
rangeMax = 20;%20;
bitmaplaser = 0.5* ones(R, C); %initialize as unknown-occupancy pixels

% sim variables

dt = 0.001;
DT = 0.01;
timeToRun = 500;
numTimesteps = timeToRun/dt;
% integrationStepsPerTimeStep = DT/dt;
% numIntegrationSteps = round(numTimesteps*integrationStepsPerTimeStep);

% find trees variables
r_check = 0.75; % distance around pts to check for occupied bitmaplaser


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



%% path gen

[desiredPath] = OptPathGen(Q0,Qend);
x_path = desiredPath(1,:);
y_path = desiredPath(2,:);

%% simulation



t = cputime;
crossTrackErrorMat = zeros(numTimesteps,2);
Q=Q0;
pointOnPath = 1;
Q_estimate = Q0;
z = Q0(1:3);
path = [x_path;y_path];

QTrueMat = zeros(numTimesteps, length(Q));
QMeasMat = zeros(numTimesteps, 3);
QEstMat = zeros(numTimesteps, 3);
QEstMatFiltered = zeros(numTimesteps, 3);

subsVecSym = [QSym; USym; tau_vSym; tau_gammaSym; DTSym; LSym; v1; v2; v3; h1; h2; h3];
subsVecNum = [Q_estimate(1:3); U; tau_v; tau_gamma; 1; L; 0; 0; 0; 0; 0; 0];
Hw = eye(3);
Fq = double(subs(FqSym, subsVecSym, subsVecNum));
Fv = double(subs(FvSym, subsVecSym, subsVecNum));
Hq = double(subs(HqSym, subsVecSym, subsVecNum));
f1 = figure();
a1 = axes(f1);
hold on
axis equal
for j = 1:numTimesteps
    % [gammaD,endDistance,crossTrackError,crossTrackErrorInterpolated] = purePursuit(Q,L,Ld,desiredPath);
    % crossTrackErrorMat(j,1)=crossTrackError;
    % crossTrackErrorMat(j,2)=crossTrackErrorInterpolated;
    % if j>numTimesteps/10 && endDistance<0.35
    %     QAll = QAll(1:(j-1)*integrationStepsPerTimeStep,:);
    %     crossTrackErrorMat=crossTrackErrorMat(1:j,:);
    %     break
    % end
    % U = [gammaD;vD];%[gammaD;1];
    % [QNext] = robot_bike_dyn(Q,U,Umin,Umax,Qmin,Qmax,L,tau_gamma,tau_v);
    % Q = QNext(end,:)';

    [slicedPath,pointOnPath] = pathSlicer(pointOnPath,path,Ld,Q);
    if j>=numTimesteps/200 && endDistance<0.5
        j
        endDistance
        E
        percenttime = j/numTimesteps;
        QTrueMat = QTrueMat(1:j,:);
        QMeasMat = QMeasMat(1:j,:);
        QEstMat = QEstMat(1:j,:);
        QEstMatFiltered = QEstMatFiltered(1:floor(j/(1/DT)),:);

        crossTrackErrorMat=crossTrackErrorMat(1:j,:);
        break
    end


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

        try
                Tl = SE2([Q(1),Ymax-Q(2),-Q(3)]);
                p = laserScannerNoisy(angleSpan, angleStep, rangeMax, Tl.T, bitmap, Xmax, Ymax);
                filteredRange = medfilt1(p(:,2),3,'omitnan','truncate');
                Tl = SE2([Q_estimate(1),Ymax-Q_estimate(2),-Q_estimate(3)]);     % lateral motion along the x-axis
                
                %the robot had issues with sensing 
                for i=3:length(p)-3
                    angle = p(i,1);
                    % range = p(i,2);
                    range = filteredRange(i,1);
                    % possible infinite range is handled inside the update function
                    n = updateLaserBeamGrid(angle, range, Tl.T, R, C, Xmax, Ymax,rangeMax);
                end
            catch
                disp("Failed to scan")
        end
        % laser scan and update scan map
        clf(a1);
        imagesc(a1,[0 Xmax], [0 Ymax],bitmaplaser./(1+bitmaplaser)); 
        scatter(a1,Q_estimate(1),Q_estimate(2),DisplayName="Estimated Location")
        scatter(a1,QTrue(1),QTrue(2),DisplayName="Actual Location")
        
        %legend(a1);
        drawnow
    end

    % QAll((j-1)*integrationStepsPerTimeStep+1:(j-1)*integrationStepsPerTimeStep+integrationStepsPerTimeStep,:) = QNext;
    % ends loop if runs for too long
    E = cputime-t;
    if E > 60*10
        break
    end

    QTrueMat(j,:) = QTrue';
    QMeasMat(j,:) = z';
    QEstMat(j,:) = Q_estimate';
    
end
E = cputime-t















%% find trees and estimate radius
fileID = fopen('results.txt','w');
MeasureRadius = zeros(M, K);
MeasureX = zeros(M, K);
MeasureY = zeros(M, K);
for i=1:K
    fprintf(fileID,'%1d\n',i);
    m = 1;
    for j=1:M
        try
        [XC, YC, RadiusC] = findTree(x(j,i),y(j,i),r_check,Xmax, Ymax, R, C);
        MeasureX(j,i) = XC;
        MeasureY(j,i) = YC;
        MeasureRadius(j,i) = RadiusC;
        if RadiusC>=0.1
            fprintf(fileID,'%1d, %6.4f, %6.4f, %6.5f\n',m,XC,YC,RadiusC);
            m=m+1;
        end
        catch
            disp("Robot failed to find tree");
        end
    end
    fprintf(fileID,'\n');

end
fclose(fileID);
%% true file

fileIDTrue = fopen('true.txt','w');

for i=1:K
    fprintf(fileIDTrue,'%1d\n',i);
    m = 1;
    for j=1:M
        if TreeRadius(j,i)>=0.1

            fprintf(fileIDTrue,'%1d, %6.4f, %6.4f, %6.5f\n',m,x(j,i),y(j,i),TreeRadius(j,i));
            m=m+1;
        end
    end

    fprintf(fileIDTrue,'\n');
end

fclose(fileIDTrue);




%% other tree find
gridResolution = Xmax/R;
bitRmax = ceil(maxTreeRadius/gridResolution);
bitRmin = max(floor(minTreeRadius*0.95/gridResolution),1);% limits min r values to 1
[centersTree,radiiTree] = imfindcircles(bitmaplaser./(1+bitmaplaser),[bitRmin bitRmax],Sensitivity=0.94);%


%%

figure(1);
clf
hold on
imagesc([0 Xmax], [0 Ymax],bitmaplaser./(1+bitmaplaser)); 
title('Laser Scanner Binary occupancy grid')
colorbar
% scatter(x,y,'xr',DisplayName="True Tree centers")
plot(x_path,y_path,DisplayName="Desired Path",LineWidth=2)
plot(QTrueMat(:,1),QTrueMat(:,2),DisplayName="True Locations",Color='r',LineWidth=2)
plot(QMeasMat(:,1),QMeasMat(:,2),DisplayName="Measured Locations",Color='g',LineWidth=2)
plot(QEstMat(:,1),QEstMat(:,2),DisplayName="Estimated Locations",Color='y',LineWidth=2)
% viscircles(centersTree*gridResolution,radiiTree*gridResolution,'Color','g');
viscircles([reshape(MeasureX,[],1) reshape(MeasureY,[],1)],reshape(MeasureRadius,[],1),'Color','r','LineStyle','--');
% viscircles([reshape(x,[],1) reshape(y,[],1)],reshape(TreeRadius,[],1),'Color','g','LineStyle','--');

legend

%%
figure(2);
clf
hold on
imagesc([0 Xmax], [0 Ymax], bitmap); %imagesc flips the bitmap rows, so correct this
set(gca,'YDir','normal');
colorbar
axis equal
scatter(x,y,'xr')
% scatter(centersTree(:,1)*gridResolution,centersTree(:,2)*gridResolution,'b+')

viscircles([reshape(MeasureX,[],1) reshape(MeasureY,[],1)],reshape(MeasureRadius,[],1),'LineStyle','--');
% viscircles([reshape(XC2,[],1) reshape(YC2,[],1)],reshape(RadiusC2,[],1),'Color','c');


%%

figure(3);
clf
hold on;
axis equal
plot(QTrueMat(:,1),QTrueMat(:,2),DisplayName="True Locations",Color='r')
plot(QMeasMat(:,1),QMeasMat(:,2),DisplayName="Measured Locations",Color='g')
plot(QEstMat(:,1),QEstMat(:,2),DisplayName="Estimated Locations",Color='b')
plot(QEstMatFiltered(:,1),QEstMatFiltered(:,2),DisplayName="Filtered Estimated Locations",Color='m')

legend


%% read from files


% filename = 'results.txt';
% [MeasureReadX, MeasureReadY, MeasureReadRadius] = readTreeFile(filename);
% filenametrue = 'true.txt';
% [TrueReadX, TrueReadY, TrueReadRadius] = readTreeFile(filenametrue);




%% compare results


ErrorX = x-MeasureX;
ErrorY = y-MeasureY;
ErrorDiameter = 2*TreeRadius-2*MeasureRadius;


figure(4)
clf
hold on
histogram(ErrorX*100,10)
title('Error histogram X pos')
xlabel('Error [cm]')

figure(5)
clf
hold on
histogram(ErrorY*100,10)
title('Error histogram Y pos')
xlabel('Error [cm]')

figure(6)
clf
hold on
histogram(ErrorDiameter*100,10)
title('Error histogram Diameter')
xlabel('Error [cm]')



%errorStatistics


xMean = mean(abs(ErrorX(:)));
xStd = std(abs(ErrorX(:)));
xRms = rms(abs(ErrorX(:)));
xMin = min(abs(ErrorX(:)));
xMax = max(abs(ErrorX(:)));
x95 = prctile(abs(ErrorX(:)),95);

yMean = mean(abs(ErrorY(:)));
yStd = std(abs(ErrorY(:)));
yRms = rms(abs(ErrorY(:)));
yMin = min(abs(ErrorY(:)));
yMax = max(abs(ErrorY(:)));
y95 = prctile(abs(ErrorY(:)),95);

dMean = mean(abs(ErrorDiameter(:)));
dStd = std(abs(ErrorDiameter(:)));
dRms = rms(abs(ErrorDiameter(:)));
dMin = min(abs(ErrorDiameter(:)));
dMax = max(abs(ErrorDiameter(:)));
d95 = prctile(abs(ErrorDiameter(:)),95);


VarNames = {'x', 'y', 'diameter'};
MeanVals = 100*[xMean, yMean, dMean];
StdVals  = 100*[xStd,  yStd,  dStd];
RmsVals  = 100*[xRms,  yRms,  dRms];
MinVals  = 100*[xMin,  yMin,  dMin];
MaxVals  = 100*[xMax,  yMax,  dMax];
P95Vals  = 100*[x95,   y95,   d95];

T = table(MeanVals', StdVals', RmsVals', MinVals', MaxVals', P95Vals', ...
    'VariableNames', {'Mean', 'Std', 'RMS', 'Min', 'Max', 'P95'}, ...
    'RowNames', VarNames);

writetable(T, 'error_stats.csv', 'WriteRowNames', true);

