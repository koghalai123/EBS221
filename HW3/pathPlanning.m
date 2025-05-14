clear;
close all

% 




rowLength = 20;
rowWidth = 2.5;
N = 10;
L = 3;
Q0 = [15;5;pi/2;0;0];
% gammaD VD
U0 = [0;1];

Qmin = [-inf,-inf,-inf,-deg2rad(60),-1]';
Qmax = -Qmin;
Umin = [Qmin(4),Qmin(5)]';
Umax = -Umin;
tau_gamma = 0.0;
tau_v = 0.0;

% x y theta gamma v

% gammaD = atan(L/rD);
rD = L/tan(Qmax(4));
%rD = L/tan(pi/4);

x = rowWidth* [[0:1:N-1],[0:1:N-1]]';
y = [repmat(0,[N,1]);repmat(rowLength,[N,1])];
posMat = [x,y];


DMat = NaN(2*N,2*N);

bigNum = 10000;
DMat(N+1:end,1:N) = bigNum;
DMat(1:N,N+1:end) = bigNum;

for i = 1:N
    DMat(i+N,i) = 0;
    DMat(i,i+N) = 0;
    DMat(i,i) = bigNum;
    DMat(i+N,i+N) = bigNum;
end
for i = 1:N-1
    for j = i+1:N
        % check for turn type
        d = abs(i-j);
        if d*rowWidth>=2*rD
            %pi turn
            distance = d*rowWidth+(pi-2)*rD;
            DMat(i,j) = distance;
            DMat(j,i) = distance;
            DMat(i+N,j) = distance;
            DMat(j+N,i) = distance;
        else
            %omega turn
            gamma = acos(1-(2*rD+d*rowWidth)^2/(8*rD^2));
            alpha = (pi-gamma)/2;
            distance = (3*pi-2*gamma)/rD;
            
            DMat(i,j) = distance;
            DMat(j,i) = distance;
            DMat(i+N,j) = distance;
            DMat(j+N,i) = distance;

        end
        
    end
end

xy = zeros(length(DMat),2);
resultStruct = tspof_ga('XY',xy,'DMAT', DMat);
route = [1 resultStruct.optRoute 2*N+2]; % extract node sequence
resultStruct.minDist %print computed minimum distance

