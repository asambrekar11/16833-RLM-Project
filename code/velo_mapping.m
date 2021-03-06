function [fusion_map, next_ref_points, next_ref_normals] = velo_mapping(fusion_map, input_data, tform, flag_invalid, theta4, pt_th)     

    %====flag is invalid vertex location
    
    %==== Set variables ====
    input_points = input_data.pointcloud.Location;
    h = size(input_points, 1);
    w = size(input_points, 2);
    
    %==== Transform the input data for fusion ====
    trans_pointcloud = pctransform(input_data.pointcloud, tform); 
    trans_points = trans_pointcloud.Location;
    trans_points(flag_invalid) = 0;
    valid_idx = find(trans_points(:,:,1)~=0); %get index of valid points

    %========get valid points==============%
    trans_pointcloud_vec = reshape(trans_points, h*w, 3);
    trans_pointcloud_vec = pointCloud(trans_pointcloud_vec(valid_idx,:));
    
    %=========get valid normals==============%
    trans_normals = nvRotate(input_data.normals, tform);
    trans_normals(flag_invalid) = 0; 
    trans_normals_vec = reshape(trans_normals,h*w,3);
    trans_normals_vec = trans_normals_vec(valid_idx,:);
    
    %========get valid ccount & range==============%
    input_ccount = reshape(input_data.ccount, h*w, 1);
    input_ccount = input_ccount(valid_idx);
    input_range = reshape(input_data.range, h*w, 1);
    input_range = input_range(valid_idx);
    
    %===============Input to shreyans function============%
    trans_data = struct('pointcloud', trans_pointcloud_vec, 'normals', trans_normals_vec,'ccounts', input_ccount, 'range', input_range);
    
    %==========KD Tree Object =============%
    Mdl = KDTreeSearcher(fusion_map.pointcloud.Location);
    knn_points = trans_pointcloud_vec.Location;
    [Idx,D] = knnsearch(Mdl,knn_points,'K',4,'IncludeTies',false);
    
    %==========Shreyans: call your function here(You can use fusion_map, trans_data, Idx, flag, valid_idx, theta4, pt_th(for projected point matching))==========%
    [ fusion_map, next_ref_data ] = update_fusion_map( fusion_map, trans_data, Idx, theta4, pt_th);
    
    next_ref_points = zeros(h*w,3);
    next_ref_points(valid_idx,:) = next_ref_data.pointcloud.Location;
    next_ref_points = reshape(next_ref_points,[h,w,3]); 
    
    next_ref_normals = trans_normals;
    
    
end