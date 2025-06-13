
%clear all;

global dT; global DT;
global epsilon_dist; %distance from goal point considered close enough to stop robot (goal reached)
global epsilon_speed; %almost zero speed
global epsilon_angle; % almost zero angle

epsilon_dist = 0.05; % (m)
epsilon_speed = 0.02; % (m/s)
epsilon_angle = 0.1; %rads
STEER_INPUT = 1; SPEED_INPUT=2; %just a name instead of using index
ROBOT_SPEED = 5; % speed is 5th element of state vector

dT = 0.01; % model integration step
DT =  0.1; %controller integration step
T = 60.0; % maximum simulation time
N = T/DT; % simulation steps

q = zeros(5, N); % state vectors over time
u=zeros(2, N);
e = zeros(1, N); %cross track error

%vehicle parameters
buildTractor; %a triangle!
L = 2.5 ; % robot wheelbase (distance between front an rear wheels) in meters
Vmax = 1;  %maximum vehicle speed
gammaMax = 45*pi/180;  % max steering angle
tau_g = .15; % steering time lag
tau_v = 0.5; % velocity time lag

% state is
% q(1) -> x
% q(2) -> y
% q(3) -> theta (orientation in world frame)
% q(4) -> phi (steering angle)
% q(5) -> linear velocity

% state constraints
Qmax(1) = Inf; Qmax(2) = Inf; Qmax(3) = Inf;
Qmax(4) = gammaMax; Qmax(5) = Vmax;
Qmin = -Qmax;

% control inputs are:
% u(1) -> desired steering angle
% u(2) -> desired linear velocity

% Control constraints
Umax=[gammaMax Vmax]';
Umin= - Umax;

%controller gains
Kv = 0.5;
Ka = 6; Kb=-4;
% disturbances
delta1 = 0.0; delta2 = 0.; slip = 0.;

% initial state
q(1,1)=10; q(2,1)=3; q(3,1)=pi/2;

% DESIRED state
xd = 3;  yd= 8;  th_d = pi/2;

figure(1); hold on; axis equal; wTr=eye(3,3); wTr_old=wTr;
k=1;
for t=0:DT:T-DT
    %navigation controller
    ex = xd - q(1,k);  ey = yd - q(2,k); %position errors
    rho = sqrt(ex^2 + ey^2);
    alpha = atan2(ey, ex) - q(3,k);
    beta = -q(3,k) - alpha + th_d;
    
    if (abs(ex) + abs(ey) > epsilon_dist) %move if robot position is not close enough to the goal position
        u(SPEED_INPUT,k) = Kv *  rho; %(m/s)
    else
        if abs(q(3,k)-th_d) < epsilon_angle %if heading was also reached, stop
            u(SPEED_INPUT,k) =0.0;
        end
    end
    u(STEER_INPUT,k) = atan2(L * (Ka * alpha + Kb* beta), q(ROBOT_SPEED,k));
    
    %sends speed and angle setpoints to low-level steer and speed controllers
    q(:, k+1) = robot_bike_dyn(q(:,k), u(:,k), Umin, Umax, Qmin, Qmax, L, tau_g, tau_v, delta1, delta2, slip);
    plot(q(1,k), q(2,k),'.');  % plot trace of robot's frame origin
    k=k+1;
    if (abs(ex) + abs(ey) < epsilon_dist && abs(q(3,k)-th_d) < epsilon_angle && q(ROBOT_SPEED,k) < epsilon_speed)
        break; % exit main loop (stop moving) because robot is close enough to the goal position
    end
    % animate tractor
    wTr=[cos(q(3,k)) -sin(q(3,k)) q(1,k)
        sin(q(3,k))  cos(q(3,k)) q(2,k)
        0 0 1]; % transformation of pose at step k
    plotTractor(tractor, wTr_old, 'w'); %erase previously plotted robot at step k-1 (draw in white color!)
    plotTractor(tractor, wTr, 'r'); pause(0.001); % draw robot at k step
    wTr_old = wTr; %keep the transformation at pose k, to erase it in the next time step.
end

%draw tractor at initial pose (it was deleted during animation)
wTr=[cos(q(3,1)) -sin(q(3,1)) q(1,1)
    sin(q(3,1))  cos(q(3,1)) q(2,1)
    0 0 1];
plotTractor(tractor, wTr, 'g');

X=sprintf('Errors (cm): x = %.2f, y = %.2f, theta =  %.2f deg\nError distance = %.2f', 100*ex, 100*ey, 180*(q(3,k)-th_d)/pi, 100*sqrt(ex^2+ey^2));
disp(X);



