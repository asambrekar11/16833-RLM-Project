%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  16833 Robot Localization and Mapping  % 
%  Final Project                         %
%  Velodyne SLAM                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all
clc
clear
%==== Only load data when needed ====
if ~exist('seq_param_loaded')
    clear
    clc
    tic
    fprintf('Loading data...\n');
    load('inputData.mat')
    fprintf('Time spent on loading: %.2f sec \n', toc);
    seq_param_loaded = true;
end


%==== TEST: Debug ICP or point-based fusion (0: false, 1: true)====
is_debug_vel_mapping = 0;
is_debug_icp = 0;
is_eval = 1;

%==== Set parameters ====
T = size(inputData, 1);
grid_size = 0.005;
theta1 = 4;
theta2 = 1;
theta3 = 0.5;
theta4 = 0.3;
pt_th = 0.0001; %need to fine tune this
positions = zeros(T, 3);

%==== Set downsampling ratio for ICP ====
ds_ratio = 9;

for t = 1:10:242
   
    %==== Display frame# ====
    fprintf('Frame#: %d', t);
    
    %==== Get Point Cloud in frame====%
    [pc_map, pc_range, flag] = projPCtoImg(inputData{t});
    
    %==== Get linkages for normals in frame====%
    [ pc_linkage ] = compute_linkage( pc_map,pc_range, theta1, theta2);
    
    %==== Get normals in frame====%
    normals = computeNormal(pc_map,pc_linkage);
    normals(flag) = 0; 
    
    %==== Get normal confidance counter in frame====%
    [ pc_ccount ] = compute_confidence( pc_map,normals,pc_linkage,theta3);
    l = length(flag)/3;
    pc_ccount(flag(1:l)) = 0; 
    
    pointcloud = pointCloud(pc_map);
    %==== Downsample input data ====
%     [ds_pointcloud, ds_normals] = downsampleData(pointcloud, normals, ds_ratio);

     %==== If this is the very first frame ====
    if t == 1

        %==== Initialize the fusion map ====            
        fusion_map = initMap(pointcloud, normals, pc_ccount, pc_range);
         
        %==== Initialize the first 4-by-4 pose matrix to identity ====
        current_pose = affine3d(eye(4));
        
        %==== Initialize the reference pointcloud and normals ====
        ref_pointcloud = pointcloud;
        ref_normals = normals;
        fprintf(' (Velodyne Slam initialized)\n');
        
    else
        
        %==== Transform the input pointcloud by the last transformation for better registration ====
        %==== (Notice: because of the format of pctransform(), last_pose[] is defined in the format of right-multiplication) ====
        new_pointcloud = pctransform(pointcloud, last_pose);
        new_points = new_pointcloud.Location;
        clear new_pointcloud;
        new_points(flag) = 0;
        new_pointcloud= pointCloud(new_points);
        
         if is_debug_vel_mapping == 0
            [tform inliers error] = getRigidTransform(new_pointcloud, ref_pointcloud, ref_normals, flag);
            fprintf(' (ICP final iteration: inliers = %d, RMSE = %d)', inliers, error);
        
        %==== DEBUG 1): A built-in registration function for debugging point-based fusion only ====
        else
            tform = pcregrigid(new_pointcloud, ref_pointcloud, 'Metric', 'pointToPlane', 'Extrapolate', true);
        end
        fprintf('\n');
        
        %==== Update the transformation ====
        %==== (Notice: because of the format of affine3d(), the transformation is defined in the format of right-multiplication) ====
        current_pose = affine3d(tform.T*last_pose.T);
        
        %==== Record the position for later plot ====
        positions(t, :) = current_pose.T(4, 1:3)';
        
        %==== Set the input data for mapping ====
        input_data = struct('pointcloud', pointcloud, 'normals', normals, 'ccount', pc_ccount, 'range', pc_range);
        
        %==== 2) Apply velo slam mapping to get a global fusion map and the next reference data ====
        if is_debug_icp == 0
            [fusion_map, ref_points, ref_normals] = velo_mapping(fusion_map, input_data, current_pose, flag, theta4, pt_th);
            re_pointcloud = pointCloud(ref_points);
        
        %==== DEBUG 2): A built-in pointcloud merging function for debugging ICP registration only
        else
            trans_pointcloud = pctransform(pointcloud, current_pose); %<= **** Please add this line ****
            trans_points = trans_pointcloud.Location;
            trans_points(flag) = 0;
            clear trans_pointcloud
            trans_pointcloud = pointCloud(trans_points);
   
            fusion_map.pointcloud = pcmerge(fusion_map.pointcloud, trans_pointcloud, grid_size);
            
            ref_pointcloud = pctransform(pointcloud, current_pose);
            ref_points = ref_pointcloud.Location;
            ref_points(flag) = 0;
            clear ref_pointcloud
            ref_pointcloud = pointCloud(ref_points);
            
            ref_normals = nvRotate(normals, current_pose);
            ref_normals(flag) = 0;
        end
        
    end
    last_pose = current_pose;
    
end

plotTrajAndMap(positions,fusion_map);

