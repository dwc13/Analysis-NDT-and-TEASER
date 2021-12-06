%% TEASER
clc
addpath('./TEASER-plusplus/build/matlab/')

% Display Source Point Cloud
ptCloudSrc = pcread('./record3pcd/1637535759.680210000.pcd');
% figure(1)
% pcshow(ptCloudSrc,'MarkerSize',20); title('Source');

% Rotating Source to create Reference Point Cloud
theta = pi/2;
rot = [cos(theta) sin(theta) 0; ...
    -sin(theta) cos(theta) 0; ...
    0 0 1];
tform = [0, 0, 0];
D = rigid3d(rot, tform);
ptCloudRef = pctransform(ptCloudSrc, D); % Reference Point Cloud

% Display Reference Point Cloud
% figure(2)
% pcshow(ptCloudRef,'MarkerSize',20); title('Reference');

% Displaying both Source and Reference Point Cloud
% figure(3)
% pcshowpair(ptCloudSrc, ptCloudRef,'MarkerSize',20); title('Source and Reference');

% Teaser
cbar2 = 1;
noise_bound = 0.01;
estimate_scaling = true; % we know there's no scale difference
rot_alg = 0; % use GNC-TLS, set to 1 for FGR
rot_gnc_factor = 1.4;
rot_max_iters = 100;
rot_cost_threshold = 1e-12;

src = ptCloudSrc.Location.'; % All the point from Point Cloud Source
dst = ptCloudRef.Location.'; % All the point from Point Cloud Reference
src(:,[1000:end]) = [];
src = double(src);
dst(:,[1000:end]) = [];
dst = double(dst);
% Align
[s, R, t, time_taken] = teaser_solve(src, dst, 'Cbar2', cbar2, 'NoiseBound', noise_bound, ...
                                     'EstimateScaling', estimate_scaling, 'RotationEstimationAlgorithm', rot_alg, ...
                                  'RotationGNCFactor', rot_gnc_factor, 'RotationMaxIterations', 100, ...
                                  'RotationCostThreshold', rot_cost_threshold);

% tform2 = rigid3d(R,t.'); % Transform
% ptCloudTeaserGound = pointCloud(src.'); % New Point Cloud
% % Transform Reference Point Cloud by Transform got from TEASER
% movingReg = pctransform(ptCloudRef,tform2); 
% 
% % Display Transformed Point Cloud
% figure(4)
% pcshowpair(movingReg,ptCloudTeaserGound,'MarkerSize',20); title('TEASER Registration');



