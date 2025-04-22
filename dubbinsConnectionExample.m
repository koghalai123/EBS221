
clear;
close all

 rmin =1.0;
 w=1;
 d=3;
startPose = [0 0 pi/2]; goalPose= [ 3 0 -pi/2];
dubConnObj = dubinsConnection;
[pathSegObj, pathCosts] =connect(dubConnObj,startPose,goalPose);
show(pathSegObj{1}); axis equal;








