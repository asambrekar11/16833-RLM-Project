function fusion_map = initMap(pointcloud, normals, pc_ccount, pc_range)
    
    h = size(pointcloud.Location,1);
    w = size(pointcloud.Location,2);
    %==== Initialize the point-based fusion map ====
    map_pointcloud = pointcloud.Location;
    valid_idx = find(map_pointcloud(:,:,1)~=0);
    map_pointcloud = reshape(map_pointcloud,h*w,3);
    map_pointcloud = pointCloud(map_pointcloud(valid_idx,:));
    map_normals = reshape(normals, h*w, 3);
    map_normals = map_normals(valid_idx,:);
    map_ccount = reshape(pc_ccount, h*w,1);
    map_ccount = map_ccount(valid_idx);
    map_range = reshape(pc_range, h*w,1);
    map_range = map_range(valid_idx);

    %==== Output the initial point-based fusion map in a struct ====
    fusion_map = struct('pointcloud', map_pointcloud, 'normals', map_normals,'ccounts', map_ccount,'range', map_range);
       
end