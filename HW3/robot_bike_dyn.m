function [QNext] = robot_bike_dyn(Q,U,Umin,Umax,Qmin,Qmax,L,tau_gamma,tau_v)
global dt DT

U = max(U,Umin);
Q = max(Q,Qmin);
U = min(U,Umax);
Q = min(Q,Qmax);

x = Q(1);
y = Q(2);
theta = Q(3);
gamma = Q(4);
V = Q(5);

gammaD = U(1);
VD = U(2);

steps = (DT/dt-1);
QNext = zeros(steps+1,length(Q));
QNext(1,:) = Q;

if tau_gamma == 0
tau_gamma= dt;
end
if tau_v == 0
tau_v= dt;
end

for i = 1:steps
    x = Q(1);
    y = Q(2);
    theta = Q(3);
    gamma = Q(4);
    V = Q(5);

    xDot = V*cos(theta);
    yDot = V*sin(theta);
    thetaDot = V*tan(gamma)/L;
    VDot = (-V+VD)/tau_v;
    gammaDot = (-gamma+gammaD)/tau_gamma;
    QDot = [xDot;yDot;thetaDot;gammaDot;VDot];
    QNew = Q+QDot*dt;
    Q = QNew;

    QNext(i+1,:) = QNew;
end








end