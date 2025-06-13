function [odoCov,GPSCov] = estimateCovariance()

global dt DT

dt = 0.001;
DT = 0.01;

% robot parameters
gamma_max = pi/3;
Qmin = [-inf,-inf,-inf,-gamma_max,-1]';
Qmax = -Qmin;

L = 3;
tau_gamma = 0.01;
tau_v = 0.01;


Umin = [Qmin(4),Qmin(5)]';
Umax = -Umin;
U0 = [0.5;0];

m = 2;
n = 10000;
measurementInterval = 100;
observations = zeros(n,m);
covStorage = zeros(round(n/measurementInterval),m,m);
measurementStorage = zeros(n,2*m);
QTrue = [0,0,0,0,0]'+rand(5,1);
QPrev = QTrue;
for i = 1:n
    % gammaD VD
    U = U0+[0.5;1].*rand(2,1);
    % x y theta gamma v
    Q_estimate = QTrue;
    QPrev = QTrue;
    %[QTrue, QOdo] = robot_bike_dyn(Q,U,Umin,Umax,Qmin,Qmax,L,tau_gamma,tau_v);
    
    
    for j = 1:1/0.01
        [QTrue, odometryInfo] = robot_odo(QTrue, U, Umin, Umax,Qmin, Qmax, L, tau_gamma, tau_v);
        dQOdo = [cos(Q_estimate(3))*odometryInfo(1);sin(Q_estimate(3))*odometryInfo(1);odometryInfo(2)];
        Q_estimate = Q_estimate(1:3)+dQOdo;
    end

    actualTravelDistance = (sum((QPrev(1:2)-QTrue(1:2)).^2))^0.5;
    estimatedTravelDistance = (sum((QPrev(1:2)-Q_estimate(1:2)).^2))^0.5;
    actualAngleChange = QTrue(3)-QPrev(3);
    estimatedAngleChange = Q_estimate(3)-QPrev(3);
    QTravel = [actualTravelDistance;actualAngleChange];
    QOdo = [estimatedTravelDistance;estimatedAngleChange];
    measurementStorage(i,:) = [QTravel' , QOdo'];
     
    observations(i,:) = QTravel - QOdo; %QTrue(1:m)-
    if mod(i,measurementInterval)==0
        covStorage(i/measurementInterval,:,:) = cov(observations(1:i,:));
    end
    
end
% 
% covElements = reshape(covStorage,round(n/measurementInterval),m^2);
% f1 = figure();
% a1 = axes(f1);
% hold on;
% for i = 1:size(covElements,2)
% plot(covElements(:,i),LineWidth=2, DisplayName=strcat("Intermediate Covariance Values for Entry: ", num2str(i)))
% yline(covElements(end,i),LineWidth=2,HandleVisibility='off',LineStyle="--")
% 
% end
% title("Covariance for Odometry");
% xlabel(strcat("Number of Samples(divided by ",num2str(measurementInterval),")"));
% ylabel("Covariance");
% l1 = legend;
% %cov(observations)
odoCov = reshape(covStorage(end,:,:),[m,m,1]);





m = 3;
n = 10000;
measurementInterval = 100;
observations = zeros(n,m);
covStorage = zeros(round(n/measurementInterval),m,m);
for i = 1:n
    % gammaD VD
    U = U0+rand(2,1);
    % x y theta gamma v
    QPrev = [0,0,0,0,0]+rand(5,1)';
    [ xGPS, yGPS, theta_GPS ] = GPS_CompassNoisy( QPrev(1), QPrev(2), QPrev(3) );
    observations(i,:) = QPrev(1:m)-[ xGPS, yGPS, theta_GPS ];
    if mod(i,measurementInterval)==0
        covStorage(i/measurementInterval,:,:) = cov(observations(1:i,:));
    end
end
% covElements = reshape(covStorage,round(n/measurementInterval),m^2);
% f1 = figure();
% a1 = axes(f1);
% hold on;
% for i = 1:size(covElements,2)
% plot(covElements(:,i),LineWidth=2,DisplayName=strcat("Intermediate Covariance Values for Entry: ", num2str(i)))
% yline(covElements(end,i),LineWidth=2,HandleVisibility='off',LineStyle="--")
% 
% end
% 
% title("Covariance for GPS");
% xlabel(strcat("Number of Samples(divided by ",num2str(measurementInterval),")"));
% ylabel("Covariance");
% legend
GPSCov = reshape(covStorage(end,:,:),[m,m,1]);


end



