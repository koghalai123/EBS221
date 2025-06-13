function [ Xmax, Ymax,x,y,radiusMat] = generateNurseryFunction(X_first_tree,Y_first_tree, M, K, D,W, R,C)
global bitmap
rng('shuffle'); %init random generator


% R = 5000; C = 5000; %numbers of rows and columns of bitmap
% bitmap = zeros(R, C); %initialize as empty
% bitmap is the robot's perceived environment, not the actual environment

% K = 5; % number of tree rows running south-north direction
% M = 7; %maximum number of trees in each row
% W = 3; % distance between tree rows (m)
% D = 2; %distance between trees in the same row (m)
Xmax = W*(K-1)+30; Ymax = Xmax;
gridResolution = Xmax/R;
x_im=[0 Xmax]; y_im=[0 Ymax]; % to display image with proper axes

x = zeros(M, K); y= zeros(M, K); %allocate arrays that hold tree center coordinates
maxTreeRadius = 0.5; %(m)
minTreeRadius = 0.2; %(m)

x(1,1) = X_first_tree-W/2; y(1,1) = Y_first_tree; %coordinates of bottom-left tree center (m)
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
x=x(:,1:K);
y=y(:,1:K);

% assign a random radius to each tree
radiusMat = zeros(M,K);
for j=1:K
    for i=1:M
        radius = (minTreeRadius + rand*(maxTreeRadius-minTreeRadius))/ (Xmax/C); %(m)
        radiusMat(i,j) = radius;
        [I, J] = XYtoIJ(x(i,j), y(i,j), Xmax, Ymax, R, C);
        if rand > 0.02 
            draw_disc(I, J, radius, R, C); %plot tree trunk
        end
    end
end


% figure(1);
% clf
% hold on
% imagesc(x_im, y_im, bitmap); %imagesc flips the bitmap rows, so correct this
% %imagesc(x_im, y_im, flipud(bitmap)); %imagesc flips the bitmap rows, so correct this
% set(gca,'YDir','normal');
% axis equal
% scatter(x,y,'or')


end