
%test laser scanner


clear all; clear workspace;
% create robot workspace
R = 1000; C = 1000; % a grid (array) of R rows and C columns (RxC pixels)
%map=makemap(R); % the environment's map is represented as a grid

Xmax = 100; Ymax = 100;  % maximum dimensions

map=zeros(R, C);
%test rectangular obstacle
Xsw=70; Ysw = 30;
Xne=Xsw + 30; Yne= Ysw + 20;
[Isw, Jsw] = XYtoIJ(Xsw, Ysw, Xmax, Ymax, R, C);
[Ine, Jne] = XYtoIJ(Xne, Yne, Xmax, Ymax, R, C);
map(Ine:Isw, Jsw:Jne) = 1;

angleSpan=pi;angleStep=pi/100;
rangeMax=100;

%current laser pose (coordinate frame) wrt world
Tl=SE2([85 0.1 pi/2]); 


p = laserScanner(angleSpan, angleStep, rangeMax, Tl.T, map, Xmax, Ymax); % contains angle and range readings

figure(1); % the figure's Y axis is reversed, so up is actually south and down is north.
imagesc([0 Xmax], [Ymax 0],map); title('Binary occupancy grid')
colorbar


figure(2);
p(isinf(p(:,2)), 2)=-1; % replace all infinite/out of range values with -1 to plot them
% plot angle and range. x-axis is displayed from -angleSpan/2 to +angleSpan/2
plot(p(:,1)*180/pi, p(:,2)); 

