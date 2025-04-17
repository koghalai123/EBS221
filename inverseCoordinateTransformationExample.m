clear;
close all



%Robot
rX=5; rY=4; rTheta = 45;
cX=2; cY=-0.2; cTheta =-30;

oTr = transl2(rX, rY) * trot2(rTheta, 'deg');
rTc = transl2(cX, cY) * trot2(cTheta, 'deg');


cameraFrameData = [-1.7326 4.6739 -1.8295 -1.6129 3.0871 -1.4017 6.0614 6.1569 4.9803 -0.9346;...
-2.7593 -2.8955 -2.4566 -2.7695 -2.6233 -2.9581 -2.9787 -2.5992 -3.3158 -3.2678];
cameraFrameHomo = padarray(cameraFrameData,[1,0],[1],'post');

unitVec = [1;1;1];
oTr*unitVec
f1 = figure();
a1 = axes(f1);
hold on;
axis equal
trplot2(oTr, 'frame', 'r', 'color', 'r');
trplot2(oTr*rTc, 'frame', 'c', 'color', 'b');
worldFrameHomo = oTr*rTc*cameraFrameHomo;
worldFramePoints = worldFrameHomo(1:2,:);
scatter(worldFramePoints(1,:)',worldFramePoints(2,:)');

%xlim([0,10]);

%plot()





























































