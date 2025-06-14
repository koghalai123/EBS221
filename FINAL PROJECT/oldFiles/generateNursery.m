function [bitmap_true] = generateNursery()

%clear all;
rng('shuffle'); %init random generator
global bitmap_true;

addpath('additionalFiles');


R = 5000; C = 5000; %numbers of rows and columns of bitmap
bitmap_true = zeros(R, C); %initialize as empty

K = 5; % number of tree rows running south-north direction
M = 7; %maximum number of trees in each row
W = 3; % distance between tree rows (m)
D = 2; %distance between trees in the same row (m)
Xmax = W*(K-1)+30; Ymax = Xmax;
gridResolution = Xmax/R;
x_im=[0 Xmax]; y_im=[0 Ymax]; % to display image with proper axes

x = zeros(M, K); y= zeros(M, K); %allocate arrays that hold tree center coordinates
maxTreeRadius = 0.5; %(m)
minTreeRadius = 0.2; %(m)

x(1,1) = 20-W/2; y(1,1) = 20; %coordinates of bottom-left tree center (m)
j=1; % current tree row
while (j<=K)
    for i = 2:M %create coordinates of centers of all trees in the row
        y(i,j) = y(i-1,j) + D; 
        x(i,j) = x(i-1,j);
    end
     % next row
    j = j+1;
    x(1,j) = x(1,j-1) + W; y(1,j) = 20; 
end
% assign a random radius to each tree
for j=1:K
    for i=1:M
        radius = (minTreeRadius + rand*(maxTreeRadius-minTreeRadius))/ (Xmax/C); %(m)
        [I, J] = XYtoIJ(x(i,j), y(i,j), Xmax, Ymax, R, C);
        if rand > 0.02 
            [bitmap_true] = draw_disc(I, J, radius, R, C, bitmap_true); %plot tree trunk
        end
    end
end

figure(1); 
imagesc(x_im, y_im, bitmap_true); %imagesc flips the bitmap rows, so correct this
%imagesc(x_im, y_im, flipud(bitmap)); %imagesc flips the bitmap rows, so correct this
set(gca,'YDir','normal');
axis equal