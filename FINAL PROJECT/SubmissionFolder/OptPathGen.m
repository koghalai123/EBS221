function [desiredPath] = OptPathGen(Q0,Qend)

global W gamma_max L M K D X_first_tree Y_first_tree


RL = D*M;
N=K+1;

HUGE = 10^100;

Rmin = L/tan(gamma_max);


DMAT = nan(2+3*N);
turnMat = string(zeros(2+3*N));
noFinalPos = 0;
fishtailTurns = 0;

x_i = Q0(1);
y_i = Q0(2);
x_f = Qend(1);
y_f = Qend(2);
x = [x_i, (X_first_tree-W:W:X_first_tree-W+W*(K)),...
    (X_first_tree-W:W:X_first_tree-W+W*(K)),...
    (X_first_tree-W:W:X_first_tree-W+W*(K)),x_f];
y = [y_i,repmat(Y_first_tree,[1,N]),repmat(Y_first_tree+RL,[1,N]),...
    repmat(Y_first_tree+RL/2,[1,N]),y_f];
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
            firstailerror = 'firstailerror'
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

resultStruct = tspof_ga('xy',xy,'DMAT',DMAT,'SHOWPROG',false,'SHOWRESULT',false,'SHOWWAITBAR',true);

E = cputime-t;

if noFinalPos
    route = [1 resultStruct.optRoute];

else
    route = [1 resultStruct.optRoute 3*N+2];


end

resultStruct.minDist;

waypointsX = [];
waypointsY = [];




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
            if y(route(i))==(RL+Y_first_tree)
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
            elseif y(route(i))==(0+Y_first_tree)
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

        waypointsX = [waypointsX, next_segmentX];
        waypointsY = [waypointsY, next_segmentY];



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

        waypointsX = [waypointsX, next_segmentX];
        waypointsY = [waypointsY, next_segmentY];


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

        waypointsX = [waypointsX, next_segmentX];
        waypointsY = [waypointsY, next_segmentY];


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

        waypointsX = [waypointsX, next_segmentX];
        waypointsY = [waypointsY, next_segmentY];


    else
        disp("Error in path planning.");
    end

end


desiredPath = [waypointsX;waypointsY];


end
