

% Track object with noisy acceleration and
% noisy measurements using kalman filter

clear all
%% simulation duration and sampling rate
duration = 20; % (s)
dt = .1;  % (s) sampling rate

%% Define update equations (Coefficent matrices): A Newtonian based model for 
%% where we expect the object to be [state transition (state + velocity)] + [input control (acceleration)]
F = [1,dt;0,1] ; % state transition matrix:  expected motion of the object (state prediction)
B = [dt^2/2;dt]; %input control matrix:  expected effect of the input accceleration on the state.
H = [1,0]; % measurement matrix

%% define main variables
u = 1.5; % define acceleration magnitude
X= [0; 0]; %initialized state--it has two components: [position; velocity]
Accel_noise_mag = 5; % process noise σw; the variability in how fast the object is speeding up (stdv of acceleration: meters/sec^2)
Q = [Accel_noise_mag^2*(B*B')]; % Compute the covariance matrix from the standard deviation of the process noise

Sensor_noise_mag = 10;  %measurement noise: (stdv of location, in meters)
R = [Sensor_noise_mag^2];% Compute the covariance matrix from the standard deviation of the measurement noise
X_estimate = X;  %x_estimate of initial location
P = Q; % Initialize covariance matrix P to model covariance

%% initialize result variables
X_true = []; % Array stores object's TRUE state trajectory
X_sensed = []; % Array stores object's SENSED state trajectory
X_loc_estimate = []; %  Array stores object's Kalman filter position estimate

%% generate actual and sensed trajectories
for t = 0 : dt: duration
    
    % Generate the object's true (noisy) trajectory
   
    w = Accel_noise_mag * randn;
    X = F*X+B*(u+w);
    % Generate what the sensor sees; noise is included.
    v = Sensor_noise_mag*randn;
    z = X(1)+v;
    
    % Prediction phase
    % predict next state of the object with the last state and predicted motion.
    X_estimate =  F*X+B*(u);
    %predict next error covariance
    P = F*P*F'+Q;
    
    % compute Kalman Gain
    K = P*H'*(H*P*H'+R)^-1;
    
    % Update the state estimate after measurement.
    X_estimate = X_estimate+K*(z-H*X_estimate);
    % update error covariance estimation.
    P =  P-K*H*P;
    %% End of Kalman filter equations
    
    %Store for plotting
    X_true = [X_true; X(1)]; %append the new true position
    X_sensed = [X_sensed; z]; % append the new measurement
    X_loc_estimate = [X_loc_estimate; X_estimate(1)];
    
end

% PLOT the results
tt = 0 : dt : duration;
%plot true path (blue)
plot(tt,X_true,'--b.'); hold on;
%plot sensed and estimated path when a running average filter is used (GREEN).
X_smooth = smooth(X_sensed);
plot(tt,X_sensed,'-k.', tt, X_smooth, '-g.');
%plot Kalman estimated path (RED) on same plot
plot( tt,X_loc_estimate,'-r.');
axis([0 20 -30 200]);
legend({'true','sensed', 'avg filter', 'Kalman'},'Location','southeast')

rms(X_true - X_smooth)
rms(X_true - X_loc_estimate)