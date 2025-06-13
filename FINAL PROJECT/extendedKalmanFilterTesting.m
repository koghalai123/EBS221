% 
% clear;
% close all;
% 
% [odoCov,GPSCov] = estimateCovariance();


% syms xSym ySym thetaSym gammaDesSym VDesSym DTSym LSym tau_vSym tau_gammaSym VD gammaD v1 v2 v3 h1 h2 h3
% 
% QSym = [xSym;ySym;thetaSym];
% USym = [gammaDesSym;VDesSym];
% odoSym = [v1;v2];
% noiseSym = [h1;h2;h3];
% gammaActSym = gammaDesSym;
% VActSym = VDesSym+v1;
% 
% 
% xDot = VActSym*cos(thetaSym);
% yDot = VActSym*sin(thetaSym);
% thetaDot = VActSym*tan(gammaActSym)/LSym;
% QDotSym = [xDot;yDot;thetaDot+v2/DTSym];
% QNewSym = QSym+QDotSym*DTSym+noiseSym;
% FqSym = jacobian(QNewSym,QSym);
% FuSym = jacobian(QNewSym,[USym]);
% FvSym = jacobian(QNewSym,[odoSym]);
% 
% HqSym = jacobian(QNewSym,[QSym]);
% HwSym = jacobian(QNewSym,[noiseSym]);
% Hw = eye(3);
% Hq = eye(3);



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









H = eye(3); % measurement matrix

global dt DT

dt = 0.01;
DT = 0.1;

Qmin = [-inf,-inf,-inf,-inf,-inf]';
Qmax = -Qmin;
Umin = [-inf,-inf]';
Umax = -Umin;
L = 1;
tau_gamma = 0.01;
tau_v = 0.01;

% x y theta gamma v
Q0 = [0;0;0;0;1];
% gammaD VD
U0 = [1;1];
Q = Q0;
U = U0;



u = rand(2,1); % define acceleration magnitude
%Q = [(0+Accel_noise_mag^2)*(B*B')]; % Compute the covariance matrix from the standard deviation of the process noise


subsVecSym = [QSym;USym;tau_vSym;tau_gammaSym;DTSym;LSym;v1;v2;v3];
subsVecNum = [Q(1:3);U;tau_v;tau_gamma;1;L;0;0;0];
B = subs(FuSym,subsVecSym,subsVecNum);
%disturbanceCov =  disturbanceMag^2 * eye(3);
disturbanceCov = odoCov;
disturbanceMag =diag(disturbanceCov^0.5); % process noise σw; the variability in how fast the object is speeding up (stdv of acceleration: meters/sec^2)
%disturbanceMag = zeros(2,1);
%sensorCov = Sensor_noise_mag^2 * eye(3);% Compute the covariance matrix from the standard deviation of the measurement noise
sensorCov = GPSCov;
Sensor_noise_mag = diag(sensorCov^0.5);  %measurement noise: (stdv of location, in meters)
%Sensor_noise_mag = zeros(3,1);
Q_estimate = Q(1:3);  %x_estimate of initial location
P = eye(3); % Initialize covariance matrix P to model covariance

%% initialize result variables
Q_true = []; % Array stores object's TRUE state trajectory
Q_sensed = []; % Array stores object's SENSED state trajectory
Q_loc_estimate = []; %  Array stores object's Kalman filter position estimate

%test


% numTimesteps = 50;
% numIntegrationSteps = numTimesteps*DT/dt;
% 
% QAll = zeros(numIntegrationSteps,length(Q));
% USetPoint = U;
% for j = 1:numTimesteps
%     w = disturbanceMag * randn(3,1);
%     v = Sensor_noise_mag*randn(3,1);
% 
%     U = USetPoint + w(1:2);
%     subsVecSym = [QSym;USym;tau_vSym;tau_gammaSym;DTSym;LSym;v1;v2;v3;h1;h2;h3];
%     subsVecNum = [Q_estimate;U;tau_v;tau_gamma;DT;L;0;0;0;0;0;0];
% 
%     Fq = double(subs(FqSym,subsVecSym,subsVecNum));
%     Fv = double(subs(FvSym,subsVecSym,subsVecNum));
% 
% 
% 
%     [QNext] = robot_bike_dyn(Q,U,Umin,Umax,Qmin,Qmax,L,tau_gamma,tau_v);
%     Q = QNext(end,:)';
% 
%     Q_estimate = double(subs(QNewSym, subsVecSym, subsVecNum));  % State prediction (noise-free)
%     P = Fq * P * Fq' + Fv * disturbanceCov * Fv';  % Covariance prediction (noise included via Fv)
% 
%     % --- EKF Update Step ---
%     z = H * Q(1:3) + v;  % Simulate noisy sensor measurement
%     nu = z - H * Q_estimate;  % Innovation
%     S = Hq * P * Hq' + Hw * sensorCov * Hw';  % Innovation covariance (noise included via Hw)
%     K = P * Hq' / S;  % Kalman gain
%     Q_estimate = Q_estimate + K * nu;  % State update
%     P = (eye(3)-K*Hq)*P*(eye(3)-K*Hq)' + K*disturbanceCov *K;  % Covariance update
% 
% 
%     QAll((j-1)*DT/dt+1:(j-1)*DT/dt+DT/dt,:) = QNext;
%     %Store for plotting
%     Q_true = [Q_true; Q']; %append the new true position
%     Q_sensed = [Q_sensed; z']; % append the new measurement
%     Q_loc_estimate = [Q_loc_estimate; Q_estimate'];
% end



numTimesteps = 22;
numIntegrationSteps = numTimesteps * DT / dt;

QAll = zeros(j, length(Q));
USetPoint = U;  % [gammaDes; VDes]
H = eye(3);     % Measurement matrix (GPS+compass observes [x; y; theta])

for j = 1:numTimesteps
     wTemp =  disturbanceMag .* randn(2, 1); 
    w = [cos(Q_estimate(3))*wTemp(1);sin(Q_estimate(3))*wTemp(1);wTemp(2)];
    v =  Sensor_noise_mag .* randn(3, 1);

    U = USetPoint; 
   
    % Substitute numerical values (NOISE TERMS SET TO ZERO)
    subsVecSym = [QSym; USym; tau_vSym; tau_gammaSym; DTSym; LSym; v1; v2; v3; h1; h2; h3];
    subsVecNum = [Q_estimate; U; tau_v; tau_gamma; 1; L; 0; 0; 0; 0; 0; 0];

   [QNext] = robot_bike_dyn(Q, U, Umin, Umax, Qmin, Qmax, L, tau_gamma, tau_v);
    Q = QNext; 
    z = H * Q(1:3) + v;
    Fq = double(subs(FqSym, subsVecSym, subsVecNum));
    Fv = double(subs(FvSym, subsVecSym, subsVecNum));

    Hq = double(subs(HqSym, subsVecSym, subsVecNum));
    Hw = double(subs(HwSym, subsVecSym, subsVecNum));
    % Hq = eye(3);
    % Hw = eye(3);

   [Q_estimate,z,P] = EKFStep(Fq,Fv,Hw,Hq,QNewSym,subsVecSym,subsVecNum,z,P,H,disturbanceCov,sensorCov) ;

    %nu
    % Store results
    QAll(j, :) = QNext;
    Q_true = [Q_true; Q'];          % Ground truth
    Q_sensed = [Q_sensed; z'];      % Noisy measurements
    Q_loc_estimate = [Q_loc_estimate; Q_estimate'];  % Filtered estimates
end

f1 = figure();
a1 = axes(f1);
lineLength = linspace(0,1);
thetaLine = [QAll(end,1),QAll(end,2)]+[(lineLength*cos(QAll(end,3)))',(lineLength*sin(QAll(end,3)))'];
hold on;
%plot(thetaLine(:,1),thetaLine(:,2),'DisplayName','Direction of Travel');
LW = 1.5;

plot(Q_true(:,1),Q_true(:,2),'DisplayName','True Locations',LineStyle="-",LineWidth=LW);
plot(Q_sensed(:,1),Q_sensed(:,2),'DisplayName','Sensed Locations',LineStyle="--",LineWidth=LW);
plot(Q_loc_estimate(:,1),Q_loc_estimate(:,2),'DisplayName','Kalman Filtered Locations',LineStyle=":",LineWidth=LW);

legend
axis equal


f2 = figure();
a2 = axes(f2);
hold on;
%plot(thetaLine(:,1),thetaLine(:,2),'DisplayName','Direction of Travel');
plot([1:length(Q_true)],Q_true(:,1),'DisplayName','True x',Color="r",LineStyle="-.",LineWidth=LW);
plot([1:length(Q_true)],Q_true(:,2),'DisplayName','True y',Color="r",LineStyle="--",LineWidth=LW);
plot([1:length(Q_true)],Q_true(:,3),'DisplayName',"True \theta",Color="r",LineStyle=":",LineWidth=LW);

plot([1:length(Q_true)],Q_sensed(:,1),'DisplayName','Sensed x',Color="b",LineStyle="-.",LineWidth=LW);
plot([1:length(Q_true)],Q_sensed(:,2),'DisplayName','Sensed y',Color="b",LineStyle="--",LineWidth=LW);
plot([1:length(Q_true)],Q_sensed(:,3),'DisplayName',"Sensed \theta",Color="b",LineStyle=":",LineWidth=LW);

plot([1:length(Q_true)],Q_loc_estimate(:,1),'DisplayName','Estimated x',Color="g",LineStyle="-.",LineWidth=LW);
plot([1:length(Q_true)],Q_loc_estimate(:,2),'DisplayName','Estimated y',Color="g",LineStyle="--",LineWidth=LW);
plot([1:length(Q_true)],Q_loc_estimate(:,3),'DisplayName',"Estimated \theta",Color="g",LineStyle=":",LineWidth=LW);


legend
function [Q_estimate,z,P] = EKFStep(Fq,Fv,Hw,Hq,QNewSym,subsVecSym,subsVecNum,z,P,H,disturbanceCov,sensorCov) 


Q_estimate = double(subs(QNewSym, subsVecSym, subsVecNum)); 
P = Fq * P * Fq' + Fv * disturbanceCov * Fv';

nu = z - H * Q_estimate; 
S = Hq * P * Hq' + Hw * sensorCov * Hw';  
K = P * Hq' / S; 
Q_estimate = Q_estimate + K * nu;  
Q_process = diag([1e-4, 1e-4, 1e-6]);  % Small artificial noise to prevent P from going to 0
P = (eye(3) - K*Hq) * P * (eye(3) - K*Hq)' + K * sensorCov * K'+ Q_process;  

end