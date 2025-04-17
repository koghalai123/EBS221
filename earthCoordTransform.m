clear;
close all


addpath('additionalFiles');
run('llh_path.m')
Lat = points_llh(:,1);
Lon = points_llh(:,2);

[x,y,utmzone] = deg2utm(Lat,Lon);


points_utm = [x,y];


f1 = figure();
a1 = axes(f1);
plot(x,y)
axis equal
title('UTM')

f2 = figure();
a2 = axes(f2);
plot(Lon,Lat)
axis equal
title('LLH')

