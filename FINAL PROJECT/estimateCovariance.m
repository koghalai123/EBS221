

close all;
clear;


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
for i = 1:n
    % gammaD VD
    U = U0+rand(2,1);
    % x y theta gamma v
    Q = [0,0,0,0,0]'+rand(5,1);
    [QTrue, QOdo] = robot_odo(Q, U, Umin, Umax,Qmin, Qmax, L, tau_gamma, tau_v);
    observations(i,:) = QTrue(1:m)-QOdo;
    if mod(i,measurementInterval)==0
        covStorage(i/measurementInterval,:,:) = cov(observations(1:i,:));
    end
end

covElements = reshape(covStorage,round(n/measurementInterval),m^2);
f1 = figure();
a1 = axes(f1);
hold on;
for i = 1:size(covElements,2)
plot(covElements(:,i),DisplayName=strcat("Intermediate Covariance Values for Entry: ", num2str(i)))
yline(covElements(end,i),DisplayName=strcat("Final Covariance Value for Entry: ", num2str(i)))

end
title("Covariance for Odometry");
xlabel(strcat("Number of Samples(divided by ",num2str(measurementInterval),")"));
ylabel("Covariance");
legend
%cov(observations)






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
covElements = reshape(covStorage,round(n/measurementInterval),m^2);
f1 = figure();
a1 = axes(f1);
hold on;
for i = 1:size(covElements,2)
plot(covElements(:,i),DisplayName=strcat("Intermediate Covariance Values for Entry: ", num2str(i)))
yline(covElements(end,i),DisplayName=strcat("Final Covariance Value for Entry: ", num2str(i)))

end

title("Covariance for GPS");
xlabel(strcat("Number of Samples(divided by ",num2str(measurementInterval),")"));
ylabel("Covariance");
legend





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
    observations(i,:) = Q(1:3)-[ xGPS, yGPS, theta_GPS ];
    if mod(i,measurementInterval)==0
        covStorage(i/measurementInterval,:,:) = cov(observations(1:i,:));
    end
end
covElements = reshape(covStorage,round(n/measurementInterval),m^2);
f1 = figure();
a1 = axes(f1);
hold on;
for i = 1:size(covElements,2)
plot(covElements(:,i),DisplayName=strcat("Intermediate Covariance Values for Entry: ", num2str(i)))
yline(covElements(end,i),DisplayName=strcat("Final Covariance Value for Entry: ", num2str(i)))

end

title("Covariance for GPS");
xlabel(strcat("Number of Samples(divided by ",num2str(measurementInterval),")"));
ylabel("Covariance");
legend






