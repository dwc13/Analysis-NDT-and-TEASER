%% NDT
clc

% Ground Point Cloud
ptCloudSrc = pcread('./record3pcd/1637535759.680210000.pcd');

% Rotating Source to create new Point Cloud
theta = pi/4;
rot = [cos(theta) sin(theta) 0; ...
    -sin(theta) cos(theta) 0; ...
    0 0 1];
tform = [0, 0, 0];
D = rigid3d(rot, tform);
ptCloudRef = pctransform(ptCloudSrc, D);

% Downsample Point Clouds
% ptCloudSrcDS = pcdownsample(ptCloudSrc,'gridAverage', 0.3);
% ptCloudRefDS = pcdownsample(ptCloudRef,'gridAverage', 0.3);

% Write Downsample Point Cloud to PCD Files
% pcwrite(ptCloudSrcDS, './ptCloudSrc.pcd');
% pcwrite(ptCloudRefDS, './ptCloudRef.pcd');
pcwrite(ptCloudSrc, './ptCloudSrc.pcd');
pcwrite(ptCloudRef, './ptCloudRef.pcd');

% Display Source Point Cloud
% figure(1)
% pcshow(ptCloudSrcDS,'MarkerSize',20); title('Source');
% pcshow(ptCloudSrc,'MarkerSize',20); title('Source');

% Display Reference Point Cloud
% figure(2)
% pcshow(ptCloudRefDS,'MarkerSize',20); title('Reference');
% pcshow(ptCloudRef,'MarkerSize',20); title('Reference');

% Displaying both Source and Reference Point Cloud
% figure(3)
% pcshowpair(ptCloudSrcDS, ptCloudRefDS,'MarkerSize',20);
% pcshowpair(ptCloudSrc, ptCloudRef,'MarkerSize',20); title('Source and Reference');

% NDT System Command
status = system('./ndt ndt'); % Outputs transformedCloud.pcd file

% Display Transform PCD file from NDT
ptCloudNewTransform = pcread('./transformedCloud.pcd');
% figure(4)
% pcshowpair(ptCloudNewTransform, ptCloudRefDS, 'MarkerSize', 20); title('NDT Registered');
% pcshowpair(ptCloudNewTransform, ptCloudRef, 'MarkerSize', 20); title('NDT Registered');
