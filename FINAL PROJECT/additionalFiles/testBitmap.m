clear all;
global bitmap;
global rangeMax;

%lidar values
rangeMax = 200; % meters
angleSpan = pi; angleStep = angleSpan/720; 

Xmax = 150; Ymax = 150; %physical dimensions of space (m)

R = 500; C = 500; %rows and columns; discretization of physical space in a grid
map=zeros(R, C);

% bitmap is the robot's perceived environment, not the actual environment
bitmap = 0.5* ones(R, C); %initialize as unknown-occupancy pixels

%create test rectangular obstacle in the actual environment (the 'map' array)
Xsw=70; Ysw = 20;
Xne=Xsw + 30; Yne= Ysw + 20;
[Isw, Jsw] = XYtoIJ(Xsw, Ysw, Xmax, Ymax, R, C);
[Ine, Jne] = XYtoIJ(Xne, Yne, Xmax, Ymax, R, C);
map(Ine:Isw, Jsw:Jne) = 1; %obstacle pixels
figure(1)
imagesc([0 Xmax], [Ymax 0],map); title('The world');

%now move the laser scanner to sense the world (map) to generate a binary occupancy grid (bitmap)
for k=1:12
    Tl = SE2([10+10*k  5 pi/2]);     % lateral motion along the x-axis 
    p = laserScanner(angleSpan, angleStep, rangeMax, Tl.T, map, Xmax, Ymax);  
    for i=1:length(p)
        angle = p(i,1); range = p(i,2);
        % possible infinite range is handled inside the update function
        n = updateLaserBeamBitmap(angle, range, Tl.T, R, C, Xmax, Ymax);
    end
end
figure(2);
imagesc([0 Xmax], [Ymax 0],bitmap); title('Binary occupancy grid')
colorbar
