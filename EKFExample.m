addpath('additionalFiles')
map = LandmarkMap(20, 10);
V = diag([0.02, 0.5*pi/180].^2); %covariance of model uncertainty
veh = Bicycle('covar', V);
veh.add_driver( RandomPath(map.dim) );
W = diag([0.1, 2*pi/180].^2); % covariance of sensor uncertainty σr=0.1, σβ=2ο.
sensor = RangeBearingSensor(veh, map, 'covar', W, 'angle', [-pi/2 pi/2], 'range', 4);
P0 = diag([0.005, 0.005, 0.001].^2); % initialize system uncertainty
ekf = EKF(veh, V, P0, sensor, W, map);
ekf.run(300); map.plot()
veh.plot_xy('b');
ekf.plot_xy('r'); %true
ekf.plot_ellipse('k');
%EKF estimated
figure(2); ekf.plot_P(); %Plot total uncertainty as a function of time