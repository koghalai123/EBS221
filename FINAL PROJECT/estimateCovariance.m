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
U0 = [0;0];

m = 2;
n = 10000;
measurementInterval = 100;
observations = zeros(n,m);
covStorage = zeros(round(n/measurementInterval),m,m);
measurementStorage = zeros(n,2*m);
QTrue = [0,0,0,0,0]'+rand(5,1);
for i = 1:n
    % gammaD VD
    U = U0+rand(2,1);
    % x y theta gamma v
    Q = QTrue;
    [QTrue, QOdo] = robot_odo(Q, U, Umin, Umax,Qmin, Qmax, L, tau_gamma, tau_v);
    travelDistance = (sum((Q(1:2)-QTrue(1:2)).^2))^0.5;
    angleChange = QTrue(3)-Q(3);
    QTravel = [travelDistance;angleChange];
    measurementStorage(i,:) = [QTravel' , QOdo'];
     
    observations(i,:) = QTravel - QOdo; %QTrue(1:m)-
    if mod(i,measurementInterval)==0
        covStorage(i/measurementInterval,:,:) = cov(observations(1:i,:));
    end
end

% covElements = reshape(covStorage,round(n/measurementInterval),m^2);
% f1 = figure();
% a1 = axes(f1);
% hold on;
% for i = 1:size(covElements,2)
% plot(covElements(:,i),DisplayName=strcat("Intermediate Covariance Values for Entry: ", num2str(i)))
% yline(covElements(end,i),DisplayName=strcat("Final Covariance Value for Entry: ", num2str(i)))
% 
% end
% title("Covariance for Odometry");
% xlabel(strcat("Number of Samples(divided by ",num2str(measurementInterval),")"));
% ylabel("Covariance");
% legend
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
    Q = [0,0,0,0,0]+rand(5,1)';
    [ xGPS, yGPS, theta_GPS ] = GPS_CompassNoisy( Q(1), Q(2), Q(3) );
    observations(i,:) = Q(1:m)-[ xGPS, yGPS, theta_GPS ];
    if mod(i,measurementInterval)==0
        covStorage(i/measurementInterval,:,:) = cov(observations(1:i,:));
    end
end
% covElements = reshape(covStorage,round(n/measurementInterval),m^2);
% f1 = figure();
% a1 = axes(f1);
% hold on;
% for i = 1:size(covElements,2)
% plot(covElements(:,i),DisplayName=strcat("Intermediate Covariance Values for Entry: ", num2str(i)))
% yline(covElements(end,i),DisplayName=strcat("Final Covariance Value for Entry: ", num2str(i)))
% 
% end
% 
% title("Covariance for GPS");
% xlabel(strcat("Number of Samples(divided by ",num2str(measurementInterval),")"));
% ylabel("Covariance");
% legend
GPSCov = reshape(covStorage(end,:,:),[m,m,1]);


end



