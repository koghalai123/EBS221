close all;
clear;


global dt DT

dt = 0.001;
DT = 0.01;


xPath= [linspace(0,10),linspace(10,10),linspace(10,20)];
yPath= [linspace(0,0),linspace(0,5),linspace(5,5)];

% robot parameters
gamma_max = pi/3;
Qmin = [-inf,-inf,-inf,-gamma_max,-1]';
Qmax = -Qmin;

L = 3;
tau_gamma = 0.0;
tau_v = 0.0;

% gammaD VD
Umin = [Qmin(4),Qmin(5)]';
Umax = -Umin;
U0 = [0;1];
U = U0;


% sim parameters

timeToRun = 600;
numTimesteps = timeToRun/DT;
integrationStepsPerTimeStep = DT/dt;
numIntegrationSteps = round(numTimesteps*integrationStepsPerTimeStep);

% field paramters

N = 10;
RL = 20;
W = 2.5;
HUGE = 10^100;
% x y theta gamma v

Q0 = [-3*W;RL/2;0;0;0];
Qend = [-3*W;RL/2;0;0;0];
Q = Q0;

QAll = zeros((numIntegrationSteps),length(Q));

Rmin = L/tan(gamma_max);
Ld = 0.2*Rmin;

DMAT = nan(2+3*N);
turnMat = string(zeros(2+3*N));
noFinalPos = 0;
fishtailTurns = 0;



x_i = Q0(1);
y_i = Q0(2);
x_f = Qend(1);
y_f = Qend(2);
x = [x_i, (((2:N+1)-1)*W-1.5*W),(((2:N+1)-1)*W-1.5*W),(((2:N+1)-1)*W-1.5*W),x_f];
y = [y_i,(2:N+1)*0,(2:N+1)*0+RL,repmat(RL/2,[1,N]),y_f];
xy = [x',y'];
for i =1:length(DMAT)
    DMAT(i,i) = 0;
    turnMat(i,i) = "same place";
end

for i=2:N+1

    for j=N+2:2*N+1
        if (j-i)==N
            DMAT(i,j)=0;
            DMAT(j,i)=0;
            DMAT(i+2*N,i)=0;
            DMAT(i,i+2*N)=0;
            DMAT(i+2*N,i+N)=0;
            DMAT(i+N,i+2*N)=0;

            turnMat(i,j) = "straight";
            turnMat(j,i)= "straight";
            turnMat(i+2*N,i)= "straight";
            turnMat(i,i+2*N)= "straight";
            turnMat(i+2*N,i+N)= "straight";
            turnMat(i+N,i+2*N)= "straight";
            % DMAT(i+2*N,j+2*N)=0;
            % DMAT(j+2*N,i+2*N)=0;
        else
            DMAT(i,j)=HUGE;
            DMAT(j,i)=HUGE;
            turnMat(i,j) = "not allowed";
            turnMat(j,i)= "not allowed";
        end
    end
end

for i=2:N
    for j=i+1:N+1
        d = abs(i-j);
        if (2*Rmin<=d*W)
            % pi turn
            turnMat(i,j) = "pi";
            DMAT(i,j) = Rmin*pi+(d-1)*W;
        elseif (2*Rmin>d*W && ~fishtailTurns)
            turnMat(i,j) = "omega";
            gamma = acos(1-((2*Rmin+d*W)^2)/(8*Rmin^2));
            DMAT(i,j) = (3*pi-2*gamma)*Rmin;% + HUGE;% forcing it to
            % always choose pi turns for testing current
            % path generation code
        elseif (2*Rmin>d*W && fishtailTurns)
            turnMat(i,j) = "fishtail";
            startPose = [0 0 pi/2]; goalPose= [ W 0 -pi/2];
            reedsConnObj = reedsSheppConnection;
            reedsConnObj.MinTurningRadius = Rmin;
            [pathSegObj,pathCosts] = connect(reedsConnObj,startPose,goalPose);
            %show(pathSegObj{1}); axis equal;
            DMAT(i,j) = pathSegObj{1}.Length;

        end
        DMAT(j,i) = DMAT(i,j);
        DMAT(i+N,j+N) = DMAT(i,j);
        DMAT(j+N,i+N) = DMAT(i,j);

        turnMat(j,i) = turnMat(i,j);
        turnMat(i+N,j+N) = turnMat(i,j);
        turnMat(j+N,i+N) = turnMat(i,j);
    end
end
for i=2:2*N+1
    if i==2 || i==N+2   % based on currrent configuration of start and end
        % want to prevent the robot from cutting through fields
        % so must start or return from top/bottom of the first row but
        % I dont want to completly prevent the option so I only add 100
        % rather than HUGE the orginal distance is about 18
        DMAT(1,i) = abs(x(1)-x(i))+abs(y(1)-y(i));
        DMAT(i,1) = DMAT(1,i);
        DMAT(3*N+2,i)=abs(x(2*N+2)-x(i))+abs(y(2*N+2)-y(i));
        DMAT(i,3*N+2)=DMAT(3*N+2,i);

    else
        DMAT(1,i) = abs(x(1)-x(i))+abs(y(1)-y(i))+100;
        DMAT(i,1) = DMAT(1,i);
        DMAT(3*N+2,i)=abs(x(2*N+2)-x(i))+abs(y(2*N+2)-y(i))+100;
        DMAT(i,3*N+2)=DMAT(3*N+2,i);
    end
    turnMat(1,i) = "start";
    turnMat(i,1) = "start";
    turnMat(3*N+2,i)="end";
    turnMat(i,3*N+2)="end";
end
DMAT(1,3*N+2) = HUGE;
DMAT(3*N+2,1) = HUGE;
DMAT(isnan(DMAT)) = HUGE;


turnMat(1,3*N+2) = "not allowed";
turnMat(3*N+2,1) = "not allowed";

t = cputime;

resultStruct = tspof_ga('xy',xy,'DMAT',DMAT,'SHOWPROG',false,'SHOWRESULT',true,'SHOWWAITBAR',true);

E = cputime-t

if noFinalPos
    route = [1 resultStruct.optRoute];


else
    route = [1 resultStruct.optRoute 3*N+2];


end

resultStruct.minDist

waypointsX = [];
waypointsY = [];



% % waypoint path gen from opt route
% for i=1:length(route)-1
%     clear next_segmentX next_segmentY
%     if x(route(i))==x(route(i+1))
%         % check if straight segment
%         next_segmentX = linspace(x(route(i)),x(route(i+1)));
%         next_segmentY = linspace(y(route(i)),y(route(i+1)));
%         waypointsX = [waypointsX, next_segmentX];
%         waypointsY = [waypointsY, next_segmentY];
%     elseif y(route(i))==y(route(i+1))
%         % check for turn
%         % check type of turn
%         d = abs(route(i)-route(i+1));
%         if (Rmin<=d*W/2)
%             % pi turn
%             if y(route(i))==RL
%                 % top row turn
%                 if x(route(i))<x(route(i+1))
%                     % turning right
%                     theta1 = linspace(pi,pi/2);
%                     theta2 = linspace(pi/2,0);
%                     arcx1 = x(route(i))+Rmin+ Rmin*cos(theta1);
%                     arcy1 = y(route(i))+ Rmin*sin(theta1);
%                     lengthx = linspace(x(route(i))+ Rmin,x(route(i+1)) - Rmin);
%                     lengthy = linspace(y(route(i))+ Rmin,y(route(i+1)) + Rmin);
%                     arcx2 = x(route(i+1))-Rmin + Rmin*cos(theta2);
%                     arcy2 = y(route(i+1)) + Rmin*sin(theta2);
%                     next_segmentX = [arcx1,lengthx,arcx2];
%                     next_segmentY = [arcy1,lengthy,arcy2];
%                     waypointsX = [waypointsX, next_segmentX];
%                     waypointsY = [waypointsY, next_segmentY];
%                 elseif x(route(i))>x(route(i+1))
%                     % turn left
%                     theta1 = linspace(0,pi/2);
%                     theta2 = linspace(pi/2,pi);
%                     arcx1 = x(route(i))-Rmin+ Rmin*cos(theta1);
%                     arcy1 = y(route(i))+ Rmin*sin(theta1);
%                     lengthx = linspace(x(route(i))- Rmin,x(route(i+1)) + Rmin);
%                     lengthy = linspace(y(route(i))+ Rmin,y(route(i+1)) + Rmin);
%                     arcx2 = x(route(i+1))+Rmin + Rmin*cos(theta2);
%                     arcy2 = y(route(i+1)) + Rmin*sin(theta2);
%                     next_segmentX = [arcx1,lengthx,arcx2];
%                     next_segmentY = [arcy1,lengthy,arcy2];
%                     waypointsX = [waypointsX, next_segmentX];
%                     waypointsY = [waypointsY, next_segmentY];
%                 else
%                     print('error in pi turn top row: 1')
%                 end
%             elseif y(route(i))==0
%                 % bottom row
%                 if x(route(i))<x(route(i+1))
%                     % turning left
%                     theta1 = linspace(pi,3*pi/2);
%                     theta2 = linspace(3*pi/2,2*pi);
%                     arcx1 = x(route(i))+Rmin+ Rmin*cos(theta1);
%                     arcy1 = y(route(i))+ Rmin*sin(theta1);
%                     lengthx = linspace(x(route(i))+ Rmin,x(route(i+1)) - Rmin);
%                     lengthy = linspace(y(route(i))- Rmin,y(route(i+1)) - Rmin);
%                     arcx2 = x(route(i+1))-Rmin + Rmin*cos(theta2);
%                     arcy2 = y(route(i+1)) + Rmin*sin(theta2);
%                     next_segmentX = [arcx1,lengthx,arcx2];
%                     next_segmentY = [arcy1,lengthy,arcy2];
%                     waypointsX = [waypointsX, next_segmentX];
%                     waypointsY = [waypointsY, next_segmentY];
%                 elseif x(route(i))>x(route(i+1))
%                     % turn right
%                     theta1 = linspace(2*pi,3*pi/2);
%                     theta2 = linspace(3*pi/2,pi);
%                     arcx1 = x(route(i))-Rmin+ Rmin*cos(theta1);
%                     arcy1 = y(route(i))+ Rmin*sin(theta1);
%                     lengthx = linspace(x(route(i))- Rmin,x(route(i+1)) + Rmin);
%                     lengthy = linspace(y(route(i))- Rmin,y(route(i+1)) - Rmin);
%                     arcx2 = x(route(i+1))+Rmin + Rmin*cos(theta2);
%                     arcy2 = y(route(i+1)) + Rmin*sin(theta2);
%                     next_segmentX = [arcx1,lengthx,arcx2];
%                     next_segmentY = [arcy1,lengthy,arcy2];
%                     waypointsX = [waypointsX, next_segmentX];
%                     waypointsY = [waypointsY, next_segmentY];
%                 else
%                     print('error in pi turn bottom row: 2')
%                 end
%             end
%         else
%             % omega turn
%             if y(route(i))==RL
%                 % top row turn
%                 if x(route(i))<x(route(i+1))
%                     % turning right
%                     gamma = acos(1-((2*Rmin+d*W)^2)/(8*Rmin^2));
%                     alpha = (pi-gamma)/2;
%                     theta1 = linspace(0,alpha);
%                     theta2 = linspace(pi+alpha,0-alpha);
%                     theta3 = linspace(pi-alpha,pi);
%                     arcx1 = x(route(i))-Rmin+ Rmin*cos(theta1);
%                     arcy1 = y(route(i))+ Rmin*sin(theta1);
%                     arcx2 = (x(route(i))-Rmin+Rmin*cos(alpha))+Rmin*cos(alpha)+Rmin*cos(theta2);
%                     arcy2 = (y(route(i))+ Rmin*sin(alpha))+Rmin*sin(alpha)+Rmin*sin(theta2);
%                     arcx3 = x(route(i+1))+Rmin+Rmin*cos(theta3);
%                     arcy3 = y(route(i+1))+Rmin*sin(theta3);
%                     next_segmentX = [arcx1,arcx2,arcx3];
%                     next_segmentY = [arcy1,arcy2,arcy3];
%                     waypointsX = [waypointsX, next_segmentX];
%                     waypointsY = [waypointsY, next_segmentY];
%                 elseif x(route(i))>x(route(i+1))
%                     % turn left
%                     gamma = acos(1-((2*Rmin+d*W)^2)/(8*Rmin^2));
%                     alpha = (pi-gamma)/2;
%                     theta1 = linspace(pi,pi-alpha);
%                     theta2 = linspace(0-alpha,pi+alpha);
%                     theta3 = linspace(alpha,0);
%                     arcx1 = x(route(i))+Rmin+ Rmin*cos(theta1);
%                     arcy1 = y(route(i))+ Rmin*sin(theta1);
%                     arcx2 = (x(route(i))+Rmin+Rmin*cos(pi-alpha))-Rmin*cos(alpha)+Rmin*cos(theta2);
%                     arcy2 = (y(route(i))+ Rmin*sin(pi-alpha))+Rmin*sin(alpha)+Rmin*sin(theta2);
%                     arcx3 = x(route(i+1))-Rmin+Rmin*cos(theta3);
%                     arcy3 = y(route(i+1))+Rmin*sin(theta3);
%                     next_segmentX = [arcx1,arcx2,arcx3];
%                     next_segmentY = [arcy1,arcy2,arcy3];
%                     waypointsX = [waypointsX, next_segmentX];
%                     waypointsY = [waypointsY, next_segmentY];
%                 else
%                     print('error in omega turn top row: 3')
%                 end
%             elseif y(route(i))==0
%                 % bottom row
%                 if x(route(i))<x(route(i+1))
%                     % turning left
%                     gamma = acos(1-((2*Rmin+d*W)^2)/(8*Rmin^2));
%                     alpha = (pi-gamma)/2;
%                     theta1 = linspace(0,-alpha);
%                     theta2 = linspace(pi-alpha,2*pi+alpha);
%                     theta3 = linspace(pi+alpha,pi);
%                     arcx1 = x(route(i))-Rmin+ Rmin*cos(theta1);
%                     arcy1 = y(route(i))+ Rmin*sin(theta1);
%                     arcx2 = (x(route(i))-Rmin+Rmin*cos(-alpha))+Rmin*cos(alpha)+Rmin*cos(theta2);
%                     arcy2 = (y(route(i))+ Rmin*sin(-alpha))-Rmin*sin(alpha)+Rmin*sin(theta2);
%                     arcx3 = x(route(i+1))+Rmin+Rmin*cos(theta3);
%                     arcy3 = y(route(i+1))+Rmin*sin(theta3);
%                     next_segmentX = [arcx1,arcx2,arcx3];
%                     next_segmentY = [arcy1,arcy2,arcy3];
%                     waypointsX = [waypointsX, next_segmentX];
%                     waypointsY = [waypointsY, next_segmentY];
%                 elseif x(route(i))>x(route(i+1))
%                     % turn right
%                     gamma = acos(1-((2*Rmin+d*W)^2)/(8*Rmin^2));
%                     alpha = (pi-gamma)/2;
%                     theta1 = linspace(pi,pi+alpha);
%                     theta2 = linspace(alpha,-pi-alpha);
%                     theta3 = linspace(-alpha,0);
%                     arcx1 = x(route(i))+Rmin+ Rmin*cos(theta1);
%                     arcy1 = y(route(i))+ Rmin*sin(theta1);
%                     arcx2 = (x(route(i))+Rmin+Rmin*cos(pi+alpha))-Rmin*cos(alpha)+Rmin*cos(theta2);
%                     arcy2 = (y(route(i))+ Rmin*sin(pi+alpha))-Rmin*sin(alpha)+Rmin*sin(theta2);
%                     arcx3 = x(route(i+1))-Rmin+Rmin*cos(theta3);
%                     arcy3 = y(route(i+1))+Rmin*sin(theta3);
%                     next_segmentX = [arcx1,arcx2,arcx3];
%                     next_segmentY = [arcy1,arcy2,arcy3];
%                     waypointsX = [waypointsX, next_segmentX];
%                     waypointsY = [waypointsY, next_segmentY];
%
%                 else
%                     print('error in omega turn bottom row: 4')
%                 end
%             end
%         end
%     else
%         if i==1
%             startOrientation = Q0(3);
%             endOrientation = atan2(y(route(i+2))-y(route(i+1)),x(route(i+2))-x(route(i+1)));
%         else
%
%             startOrientation = atan2(y(route(i))-y(route(i-1)),x(route(i))-x(route(i-1)));
%             endOrientation = atan2(y(route(i+1))-y(route(i)),x(route(i+1))-x(route(i)));
%
%         end
%         startPose = [x(route(i)) y(route(i)) startOrientation]; goalPose= [ x(route(i+1)) y(route(i+1)) endOrientation];
%         dubConnObj = dubinsConnection;
%         dubConnObj.MinTurningRadius = Rmin;
%         [pathSegObj, pathCosts] =connect(dubConnObj,startPose,goalPose);
%
%         lengths = 0:pathSegObj{1}.Length/100:pathSegObj{1}.Length;
%         poses = interpolate(pathSegObj{1},lengths);
%         next_segmentX = poses(:,1)';
%         next_segmentY = poses(:,2)';
%         % figure
%         % show(pathSegObj{1}); axis equal;
%
%         % start finish paths
%         % next_segmentX = linspace(x(route(i)),x(route(i+1)));
%         % next_segmentY = linspace(y(route(i)),y(route(i+1)));
%         waypointsX = [waypointsX, next_segmentX];
%         waypointsY = [waypointsY, next_segmentY];
%     end
% end


% waypoint path gen from opt route
reverseDirectionMat = [];
for i=1:length(route)-1
    clear next_segmentX next_segmentY
    %disp(turnMat(route(i),route(i+1)))
    if strcmpi(turnMat(route(i),route(i+1)),"straight")
        % check if straight segment
        next_segmentX = linspace(x(route(i)),x(route(i+1)));
        next_segmentY = linspace(y(route(i)),y(route(i+1)));
        waypointsX = [waypointsX, next_segmentX];
        waypointsY = [waypointsY, next_segmentY];
    elseif strcmpi(turnMat(route(i),route(i+1)),"pi") %y(route(i))==y(route(i+1))
        % check for turn
        % check type of turn
        d = abs(route(i)-route(i+1));
        if (Rmin<=d*W/2)
            % pi turn
            if y(route(i))==RL
                % top row turn
                if x(route(i))<x(route(i+1))
                    % turning right
                    theta1 = linspace(pi,pi/2);
                    theta2 = linspace(pi/2,0);
                    arcx1 = x(route(i))+Rmin+ Rmin*cos(theta1);
                    arcy1 = y(route(i))+ Rmin*sin(theta1);
                    lengthx = linspace(x(route(i))+ Rmin,x(route(i+1)) - Rmin);
                    lengthy = linspace(y(route(i))+ Rmin,y(route(i+1)) + Rmin);
                    arcx2 = x(route(i+1))-Rmin + Rmin*cos(theta2);
                    arcy2 = y(route(i+1)) + Rmin*sin(theta2);
                    next_segmentX = [arcx1,lengthx,arcx2];
                    next_segmentY = [arcy1,lengthy,arcy2];
                    waypointsX = [waypointsX, next_segmentX];
                    waypointsY = [waypointsY, next_segmentY];
                elseif x(route(i))>x(route(i+1))
                    % turn left
                    theta1 = linspace(0,pi/2);
                    theta2 = linspace(pi/2,pi);
                    arcx1 = x(route(i))-Rmin+ Rmin*cos(theta1);
                    arcy1 = y(route(i))+ Rmin*sin(theta1);
                    lengthx = linspace(x(route(i))- Rmin,x(route(i+1)) + Rmin);
                    lengthy = linspace(y(route(i))+ Rmin,y(route(i+1)) + Rmin);
                    arcx2 = x(route(i+1))+Rmin + Rmin*cos(theta2);
                    arcy2 = y(route(i+1)) + Rmin*sin(theta2);
                    next_segmentX = [arcx1,lengthx,arcx2];
                    next_segmentY = [arcy1,lengthy,arcy2];
                    waypointsX = [waypointsX, next_segmentX];
                    waypointsY = [waypointsY, next_segmentY];
                else
                    print('error in pi turn top row: 1')
                end
            elseif y(route(i))==0
                % bottom row
                if x(route(i))<x(route(i+1))
                    % turning left
                    theta1 = linspace(pi,3*pi/2);
                    theta2 = linspace(3*pi/2,2*pi);
                    arcx1 = x(route(i))+Rmin+ Rmin*cos(theta1);
                    arcy1 = y(route(i))+ Rmin*sin(theta1);
                    lengthx = linspace(x(route(i))+ Rmin,x(route(i+1)) - Rmin);
                    lengthy = linspace(y(route(i))- Rmin,y(route(i+1)) - Rmin);
                    arcx2 = x(route(i+1))-Rmin + Rmin*cos(theta2);
                    arcy2 = y(route(i+1)) + Rmin*sin(theta2);
                    next_segmentX = [arcx1,lengthx,arcx2];
                    next_segmentY = [arcy1,lengthy,arcy2];
                    waypointsX = [waypointsX, next_segmentX];
                    waypointsY = [waypointsY, next_segmentY];
                elseif x(route(i))>x(route(i+1))
                    % turn right
                    theta1 = linspace(2*pi,3*pi/2);
                    theta2 = linspace(3*pi/2,pi);
                    arcx1 = x(route(i))-Rmin+ Rmin*cos(theta1);
                    arcy1 = y(route(i))+ Rmin*sin(theta1);
                    lengthx = linspace(x(route(i))- Rmin,x(route(i+1)) + Rmin);
                    lengthy = linspace(y(route(i))- Rmin,y(route(i+1)) - Rmin);
                    arcx2 = x(route(i+1))+Rmin + Rmin*cos(theta2);
                    arcy2 = y(route(i+1)) + Rmin*sin(theta2);
                    next_segmentX = [arcx1,lengthx,arcx2];
                    next_segmentY = [arcy1,lengthy,arcy2];
                    waypointsX = [waypointsX, next_segmentX];
                    waypointsY = [waypointsY, next_segmentY];
                else
                    print('error in pi turn bottom row: 2')
                end
            end
        end
    elseif strcmpi(turnMat(route(i),route(i+1)),"omega")
        % omega turn
        startOrientation = atan2(y(route(i))-y(route(i-1)),x(route(i))-x(route(i-1)));
        endOrientation = atan2(y(route(i+2))-y(route(i+1)),x(route(i+2))-x(route(i+1)));
        startPose = [x(route(i)) y(route(i)) startOrientation]; goalPose= [ x(route(i+1)) y(route(i+1)) endOrientation];
        dubConnObj = dubinsConnection;
        dubConnObj.MinTurningRadius = Rmin;
        [pathSegObj, pathCosts] =connect(dubConnObj,startPose,goalPose);

        lengths = 0:pathSegObj{1}.Length/100:pathSegObj{1}.Length;
        poses = interpolate(pathSegObj{1},lengths);
        next_segmentX = poses(:,1)';
        next_segmentY = poses(:,2)';
        % figure
        % show(pathSegObj{1}); axis equal;

        % start finish paths
        % next_segmentX = linspace(x(route(i)),x(route(i+1)));
        % next_segmentY = linspace(y(route(i)),y(route(i+1)));
        waypointsX = [waypointsX, next_segmentX];
        waypointsY = [waypointsY, next_segmentY];

        % figure(2)
        % clf
        % hold on
        % plot(waypointsX,waypointsY)
        % 1


    elseif strcmpi(turnMat(route(i),route(i+1)),"fishtail")
        startOrientation = atan2(y(route(i))-y(route(i-1)),x(route(i))-x(route(i-1)));
        endOrientation = atan2(y(route(i+2))-y(route(i+1)),x(route(i+2))-x(route(i+1)));
        startPose = [x(route(i)) y(route(i)) startOrientation]; goalPose= [ x(route(i+1)) y(route(i+1)) endOrientation];
        reedsConnObj = reedsSheppConnection;
        reedsConnObj.MinTurningRadius = Rmin;
        [pathSegObj, pathCosts] =connect(reedsConnObj,startPose,goalPose);
        pathPoints = cumsum(pathSegObj{1}.MotionLengths);
        pathDistances = cumsum(sqrt( sum( abs( diff( poses(:,1:2) ) ).^2, 2 ) ));
        [M,I] = min(abs(pathPoints(1:3)-pathDistances));   
        reverseDirectionMat = [reverseDirectionMat;I(1:2)'+size(waypointsX,2)];

        lengths = 0:pathSegObj{1}.Length/100:pathSegObj{1}.Length;
        poses = interpolate(pathSegObj{1},lengths);
        next_segmentX = poses(:,1)';
        next_segmentY = poses(:,2)';
        % figure
        % show(pathSegObj{1}); axis equal;

        % start finish paths
        % next_segmentX = linspace(x(route(i)),x(route(i+1)));
        % next_segmentY = linspace(y(route(i)),y(route(i+1)));
        waypointsX = [waypointsX, next_segmentX];
        waypointsY = [waypointsY, next_segmentY];

%         figure();
%         clf
%         hold on
%         plot(waypointsX,waypointsY)
% 1
    elseif strcmpi(turnMat(route(i),route(i+1)),"start")

        startOrientation = Q0(3);
        endOrientation = atan2(y(route(i+2))-y(route(i+1)),x(route(i+2))-x(route(i+1)));
        startPose = [x(route(i)) y(route(i)) startOrientation]; goalPose= [ x(route(i+1)) y(route(i+1)) endOrientation];
        dubConnObj = dubinsConnection;
        dubConnObj.MinTurningRadius = Rmin;
        [pathSegObj, pathCosts] =connect(dubConnObj,startPose,goalPose);

        lengths = 0:pathSegObj{1}.Length/100:pathSegObj{1}.Length;
        poses = interpolate(pathSegObj{1},lengths);
        next_segmentX = poses(:,1)';
        next_segmentY = poses(:,2)';
        % figure
        % show(pathSegObj{1}); axis equal;

        % start finish paths
        % next_segmentX = linspace(x(route(i)),x(route(i+1)));
        % next_segmentY = linspace(y(route(i)),y(route(i+1)));
        waypointsX = [waypointsX, next_segmentX];
        waypointsY = [waypointsY, next_segmentY];
        % figure(2)
        % clf
        % hold on
        % plot(waypointsX,waypointsY)

    elseif strcmpi(turnMat(route(i),route(i+1)),"end")

        startOrientation = atan2(y(route(i))-y(route(i-1)),x(route(i))-x(route(i-1)));
        endOrientation = atan2(y(route(i+1))-y(route(i)),x(route(i+1))-x(route(i)));
        startPose = [x(route(i)) y(route(i)) startOrientation]; goalPose= [ x(route(i+1)) y(route(i+1)) endOrientation];
        dubConnObj = dubinsConnection;
        dubConnObj.MinTurningRadius = Rmin;
        [pathSegObj, pathCosts] =connect(dubConnObj,startPose,goalPose);

        lengths = 0:pathSegObj{1}.Length/100:pathSegObj{1}.Length;
        poses = interpolate(pathSegObj{1},lengths);
        next_segmentX = poses(:,1)';
        next_segmentY = poses(:,2)';
        % figure
        % show(pathSegObj{1}); axis equal;

        % start finish paths
        % next_segmentX = linspace(x(route(i)),x(route(i+1)));
        % next_segmentY = linspace(y(route(i)),y(route(i+1)));
        waypointsX = [waypointsX, next_segmentX];
        waypointsY = [waypointsY, next_segmentY];
        % figure(2)
        % clf
        % hold on
        % plot(waypointsX,waypointsY)

    else
        disp("Error in path planning.");
    end

end

% figure(2)
% clf
% hold on
% plot(waypointsX,waypointsY)

VD = 1;
Umin = [-pi/6,Qmin(5)]';
Umax = -Umin;
U0 = [0;1];
U = U0;



path = [waypointsX;waypointsY];
crossTrackErrorMat = zeros(numTimesteps,2);
Q=Q0;
pointOnPath = 1;
reverseMultiplier = (-1).^([0:length(reverseDirectionMat)])';
for j = 1:numTimesteps
    try
    [slicedPath,pointOnPath,reversePoint] = pathSlicer(pointOnPath,path,Ld,Q,reverseDirectionMat);
    [gammaD,endDistance,crossTrackError,crossTrackErrorInterpolated] = purePursuit(Q,L,Ld,slicedPath);

    if reversePoint
        nextReverseInd = find(reversePoint==reverseDirectionMat);
        Q(5) = abs(Q(5))*reverseMultiplier(nextReverseInd);
        if reverseMultiplier(nextReverseInd) == -1
            %gammaD = -gammaD;
            VD = -1;
        end
    end

    catch
1
    end
    timeMat(j) = t;
    crossTrackErrorMat(j,1)=crossTrackError;
    crossTrackErrorMat(j,2)=crossTrackErrorInterpolated;
    if j>numTimesteps/10 && endDistance<0.35
        if reversePoint

        else
            QAll = QAll(1:(j-1)*integrationStepsPerTimeStep,:);
            crossTrackErrorMat=crossTrackErrorMat(1:j,:);
            break
        end
    end
    U = [gammaD;VD];
    [QNext] = robot_bike_dyn(Q,U,Umin,Umax,Qmin,Qmax,L,tau_gamma,tau_v);
    Q = QNext(end,:)';
    QAll((j-1)*integrationStepsPerTimeStep+1:(j-1)*integrationStepsPerTimeStep+integrationStepsPerTimeStep,:) = QNext;
end
tMat = dt*[1:j];



figure(2)
clf
hold on
plot(waypointsX,waypointsY)
scatter(x(route(:)),y(route(:)),'rx')
plot(QAll(:,1),QAll(:,2))
legend('Scheduled Path','Waypoints','Traversed Path')
%scatter(path(1,reverseDirectionMat),path(2,reverseDirectionMat))
axis equal
xlabel("X Location [m]");
ylabel("Y Location [m]");


QSampled = QAll(1:DT/dt:end,:);
xCutoff = 5;
turnStartInd = round(find(QSampled(:,2)<0 & QSampled(:,1)>xCutoff,1,'first'));
turnEndInd = round(find(QSampled(turnStartInd:end,2)>0,1,'first')+turnStartInd);

turnPathStartInd = find(path(2,:)<0 & path(1,:)>xCutoff,1,'first');
turnPathEndInd = find(path(2,turnPathStartInd:end)>0,1,'first')+turnPathStartInd-2;

figure(3)
clf
subplot(1,2,1)
hold on

CTESegment = crossTrackErrorMat(turnStartInd:turnEndInd,2);
turnRMS  = rms(CTESegment);

plot(tMat(turnStartInd:turnEndInd),CTESegment,DisplayName="Error During Turn")
legend()
xlabel("Time [s]");
ylabel("Cross Track Error [m]");

subplot(1,2,2)
hold on
plot(QSampled(turnStartInd:turnEndInd,1),QSampled(turnStartInd:turnEndInd,2),DisplayName="Turn Path Traversed")
plot(path(1,turnPathStartInd:turnPathEndInd),path(2,turnPathStartInd:turnPathEndInd),DisplayName="Turn Path Desired")

legend()
xlabel("X Location [m]");
ylabel("Y Location [m]");
axis equal



xCutoff = 5;
straightStartInd = round(find(QSampled(:,2)<10 & QSampled(:,1)>xCutoff,1,'first'));
straightEndInd = find(QSampled(straightStartInd:end,2)<0,1,'first')+straightStartInd;

straightPathStartInd = find(path(2,:)<10 & path(1,:)>xCutoff,1,'first');
straightPathEndInd = find(path(2,straightPathStartInd:end)<0,1,'first')+straightPathStartInd;

figure(4)
clf
subplot(1,2,1)
hold on

CTESegment = crossTrackErrorMat(straightStartInd:straightEndInd,2);
straightRMS  = rms(CTESegment);

plot(tMat(straightStartInd:straightEndInd),CTESegment,DisplayName="Error During Straight")
legend()
xlabel("Time [s]");
ylabel("Cross Track Error [m]");

subplot(1,2,2)
hold on
plot(QSampled(straightStartInd:straightEndInd,1),QSampled(straightStartInd:straightEndInd,2),DisplayName="Turn Path Traversed")
plot(path(1,straightPathStartInd:straightPathEndInd),path(2,straightPathStartInd:straightPathEndInd),DisplayName="Turn Path Desired")

legend()
xlabel("X Location [m]");
ylabel("Y Location [m]");
axis equal


function [slicedPath,newPointOnPath,reversePoint] = pathSlicer(pointOnPath,path,Ld,Q,reverseDirectionMat)
pointsFromReverse = reverseDirectionMat-pointOnPath;
pointsFromReverse = pointsFromReverse(pointsFromReverse>0);
[N,I] = min([pointsFromReverse',100,length(path)-pointOnPath]);
reversePoint = [];
if I<length(pointsFromReverse)
reversePoint = reverseDirectionMat(I);
end
if all([~isempty(reversePoint) , N==1])

reverseDirectionMat(I) = [];
pointsFromReverse = reverseDirectionMat-pointOnPath;
pointsFromReverse = pointsFromReverse(pointsFromReverse>0);
[N,I] = min([pointsFromReverse',100,length(path)-pointOnPath]);
reversePoint = [];
if I<length(reverseDirectionMat)
reversePoint = reverseDirectionMat(I);
end
end
%N =100
slicedPath=path(:,pointOnPath:pointOnPath+N);
squaredDistanceMat = sum((slicedPath-Q(1:2)).^2);
[M,I] = min(squaredDistanceMat);
newPointOnPath = pointOnPath+I-1;



end

function [commandMat] = movementCommands(timeToRun,DT,inputCommandMat)
%specify input commands as a matrix where the first and second columns are
%the commands and the third column is the time at which it starts
timeSteps = [inputCommandMat(1,end):DT:inputCommandMat(end,end)];
%commandMat = interp1()
commandMat = [];
for m = 1:size(inputCommandMat,1)-1
    A = inputCommandMat(m,:);
    n = (inputCommandMat(m+1,end)-inputCommandMat(m,end))/DT;
    t = [inputCommandMat(m,end):DT:inputCommandMat(m+1,end)-DT]';
    B = repmat(A,[n,1]);
    B(:,end) = t;
    commandMat = [commandMat;B];
end
end











































