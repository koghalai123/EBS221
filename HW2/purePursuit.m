function [gammaD,endDistance,crossTrackError,crossTrackErrorInterpolated] = purePursuit(Q,L,Ld,path)
xPath = path(1,:);
yPath = path(2,:);


globalRobotLoc = [Q(1:2)];
distanceMatRobot= (sum((Q(1:2)-[xPath;yPath]).^2).^.5);
[Dmin,minPointIndex] = min(distanceMatRobot,[],2);
nearestPathPoint = [xPath(minPointIndex);yPath(minPointIndex)];
futurePathMat = [xPath(minPointIndex:end);yPath(minPointIndex:end)];
distanceMatPath = (sum((nearestPathPoint-futurePathMat).^2).^.5);
targetPointIndex = find(distanceMatPath>Ld,1);
globalTargetPoint = futurePathMat(:,targetPointIndex);
globalDistanceVector = globalTargetPoint-globalRobotLoc;
robotDistanceVector = trot2(Q(3), 'rad')'*[globalDistanceVector;1];
% close all
% plot(xPath,yPath)
% hold on
% scatter(globalRobotLoc(1),globalRobotLoc(2))
% scatter(nearestPathPoint(1),nearestPathPoint(2))
% scatter(globalTargetPoint(1),globalTargetPoint(2))
% axis equal
k=2*robotDistanceVector(2)/Ld^2;
gammaD = atan(k*L);
endDistance=distanceMatRobot(end);
crossTrackError=Dmin;
numInterpolatedPoints = 50;
interpolationIndices = linspace(0,200,numInterpolatedPoints);
nearestPathX =xPath([minPointIndex-1,minPointIndex,minPointIndex+1]);
nearestPathY = yPath([minPointIndex-1,minPointIndex,minPointIndex+1]);
interpolatedPointsX = interp1([0,100,200],nearestPathX,interpolationIndices);
interpolatedPointsY = interp1([0,100,200],nearestPathY,interpolationIndices);

distanceMatInterpolated= (sum((Q(1:2)-[interpolatedPointsX;interpolatedPointsY]).^2).^.5);
[crossTrackErrorInterpolated,minPointIndexCrossTrackInterpolated] = min(distanceMatInterpolated,[],2);

end