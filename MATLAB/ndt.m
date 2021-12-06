%% Point Cloud
% -----------------------------------------------------------------------------
% Ground truth Point Cloud Image
ptCloudGround = pcread('./1637535634.411389000.pcd');
figure(1)
pcshow(ptCloudGround,'MarkerSize',20); title('Ground');

% -----------------------------------------------------------------------------
% Rotating Ground to make an Unmatch Point Cloud
theta = pi/4;
rot = [cos(theta) sin(theta) 0; ...
    -sin(theta) cos(theta) 0; ...
    0 0 1];
tform = [0, 0, 0];
D = rigid3d(rot, tform);
ptCloudUnmatch = pctransform(ptCloudGround, D);

% ptCloudUnmatch = pcread('./1637535634.411389000-transform.pcd');

figure(2)
% subplot(2,3,2); pcshow(ptCloudUnmatch); title('Unmatch');
pcshow(ptCloudUnmatch,'MarkerSize',20); title('Unmatch');

% -----------------------------------------------------------------------------
% Displaying both Ground and Unmatch Point Cloud
figure(3)
pcshowpair(ptCloudGround, ptCloudUnmatch,'MarkerSize',20); title('Ground and Unmatch');

% Write to PCD File
pcwrite(ptCloudUnmatch, './1637535634.411389000-transform.pcd');

% Run NDT through command prompt

%%
ptCloudNewTransform = pcread('./transformed.pcd');
figure(4)
pcshow(ptCloudNewTransform, 'MarkerSize', 20); title('transform');
