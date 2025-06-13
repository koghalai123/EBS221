function [XC, YC, RadiusC] = findTree(X_aprox,Y_aprox,r_check,Xmax, Ymax, R, C)
global bitmaplaser 
testmap = bitmaplaser./(1+bitmaplaser); % puts in proper scale

if X_aprox<=0 && Y_aprox<=0
    XC = X_aprox;
    YC = Y_aprox;
    RadiusC =0;
else

% bounds to check for tree
% lower left and upper right Ymax - 
[ i1, j1 ] = XYtoIJ(X_aprox-r_check, Ymax -(Y_aprox-r_check), Xmax, Ymax, R, C);
[ i2, j2 ] = XYtoIJ(X_aprox+r_check, Ymax -(Y_aprox+r_check), Xmax, Ymax, R, C);

% create empty map and only change check area
onetreemap = zeros(R,C);
onetreemap(i1:i2,j1:j2) = testmap(i1:i2,j1:j2);

% % debug
% global bitmap
% bitmap(i1:i2,j1:j2)=1;

% finds all points that are not empty
[row,col]=find(onetreemap>=0.6);
pts = [row,col];

% check if there is no tree
% prevents errors
if length(row)<2
    XC = X_aprox;
    YC = Y_aprox;
    RadiusC = -1;
else

% finds points that are likely noise as they are not near other points
[dists, neighInds] = nndist(pts);
deNoisyIndex = find(dists<2.1);

% only use points that are not noise to find x y and r
circle1 = enclosingCircle(pts(deNoisyIndex,:));

% converts from IJ to XY
[ XC, YCtemp ] = IJtoXY(circle1(1), circle1(2), Xmax, Ymax, R, C);
YC = Ymax - YCtemp;
gridResolution = Xmax/R;
RadiusC = circle1(3)*gridResolution;
end

end