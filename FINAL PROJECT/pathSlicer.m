function [slicedPath,newPointOnPath] = pathSlicer(pointOnPath,path,Ld,Q)
% pointsFromReverse = reverseDirectionMat-pointOnPath;
% pointsFromReverse = pointsFromReverse(pointsFromReverse>0);
% [N,I] = min([pointsFromReverse',100,length(path)-pointOnPath]);
% reversePoint = [];
% if I<length(pointsFromReverse)
% reversePoint = reverseDirectionMat(I);
% end
% if all([~isempty(reversePoint) , N==1])
% 
% reverseDirectionMat(I) = [];
% pointsFromReverse = reverseDirectionMat-pointOnPath;
% pointsFromReverse = pointsFromReverse(pointsFromReverse>0);
% [N,I] = min([pointsFromReverse',100,length(path)-pointOnPath]);
% reversePoint = [];
% if I<length(reverseDirectionMat)
% reversePoint = reverseDirectionMat(I);
% end
% end
%N =1000;

[N,I] = min([100,length(path)-pointOnPath]);
slicedPath=path(:,pointOnPath:pointOnPath+N);
squaredDistanceMat = sum((slicedPath-Q(1:2)).^2);
[M,I] = min(squaredDistanceMat);
newPointOnPath = pointOnPath+I-1;



end
