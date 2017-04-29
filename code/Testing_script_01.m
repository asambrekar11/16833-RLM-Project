%For testing the various functions

%Load data
load('inputData.mat');
theta1 = 4;
theta2 = 1;
theta3 = 0.5;
theta4 = 0.3;
%g = 5cm;

%Projected point cloud
[pc_map, pc_range] = projPCtoImg(inputData{1});

%Pt cloud linkage calculation
pc_linkage = compute_linkage( pc_map,pc_range, theta1, theta2);

%Compute Normals
pc_normals = computeNormal(pc_map,pc_linkage);

%Compute surface confidence
pc_confidence = compute_confidence( pc_map,pc_normals,pc_linkage,theta3 );
