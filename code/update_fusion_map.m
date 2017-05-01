function [ new_fusion_map, ref_vec_pt_cloud ] = update_fusion_map( fusion_map,vec_pt_cloud,KD_tree_idx,theta4, thresh_dist2 )
%Updating the fusion map with current point cloud
% For reference:
% fusion_map = struct('pointcloud', map_pointcloud, 'normals', map_normals,'ccounts', map_ccount,'range', map_range);


%Defining size of the weights- 
w_ij = zeros(size(vec_pt_cloud.normals,1),size(KD_tree_idx,2));

%Confidence counts multiplication
Ci_cj = repmat(vec_pt_cloud.ccounts,[1,size(KD_tree_idx,2)]).*fusion_map.ccounts(KD_tree_idx);
%Normal dot product
vector_KD_idx = KD_tree_idx';
vector_KD_idx = vector_KD_idx(:);
Ni_nj = dot(kron(vec_pt_cloud.normals,[1:size(KD_tree_idx,2)]'),fusion_map.normals(vector_KD_idx,:),2);
Ni_nj = reshape(Ni_nj,[size(KD_tree_idx,2),size(vec_pt_cloud.normals,1)])';
w_ij = Ci_cj.*Ni_nj;

Pi_j_dot_nj = dot(kron(vec_pt_cloud.pointcloud,ones(size(KD_tree_idx,2),1))-fusion_map.pointcloud(vector_KD_idx,:),fusion_map.normals(vector_KD_idx,:),2);
Pi_j_dot_nj = reshape(Pi_j_dot_nj,[size(KD_tree_idx,2),size(vec_pt_cloud.normals,1)])';

a = sum(2*w_ij.*Pi_j_dot_nj.*Ni_nj,2)./sum(2*w_ij.*Ni_nj.^2,2);

moved_vec_pt_cloud = vec_pt_cloud.pointcloud + repmat(a,[1 3]).*vec_pt_cloud.normals;


%Need to add condition for invalid points
dist_to_KD_matches = sum((fusion_map.pointcloud(vector_KD_idx,:)-kron(moved_vec_pt_cloud,ones(size(KD_tree_idx,2),1))).^2,2);
dist_to_KD_matches = reshape(dist_to_KD_matches,[size(KD_tree_idx,2),size(vec_pt_cloud.normals,1)])';

%Finding the closest KNN
[min_dist,min_idx] = min(dist_to_KD_matches,[],1);

linear_idx = sub2ind(size(dist_to_KD_matches),[1:size(dist_to_KD_matches,1)],min_idx)';
closest_KD = dist_to_KD_matches(linear_idx);

%Range check
flag_range = (1-(fusion_map.range(KD_tree_idx(linear_idx))./vec_pt_cloud.range) + (vec_pt_cloud.ccounts - fusion_map.ccounts(linear_idx))) > theta4;

%Update the point if valid, close enough neighbors in the surface and high
%confidence in the range measurement
flag_update = and(vec_pt_cloud.flag,and(closest_KD < thresh_dist2,flag_range));
flag_new_points = and(vec_pt_cloud.flag,~and(closest_KD < thresh_dist2,flag_range));
idx_update_points = find(flag_update);
idx_new_points = find(flag_new_points);

%Update fusion map
flag_fusion_to_keep = ones(size(fusion_map.ccounts,1),1);

%Indexes to fusion map
idx_fusion_list = KD_tree_idx(linear_idx);
flag_fusion_to_keep(idx_fusion_list(find(flag_update))) = 0;

idx_fusion_to_keep = find(flag_fusion_to_keep);
idx_pt_cloud_to_update = find(flag_update);

%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%
% To Do- 
% normals, ccounts and range are not averaged
% Should be done like ICP fusion?
%%%%%%%%%%%%%%%%%%%%%%%%

%Fusion map to be updated based on flag_fusion_to_keep, adding new points
%and adding updated points
map_pointcloud = [fusion_map.pointcloud(idx_fusion_to_keep,:);vec_pt_cloud.pointcloud(idx_new_points,:);moved_vec_pt_cloud(idx_update_points,:)];
map_normals = [fusion_map.normals(idx_fusion_to_keep,:);vec_pt_cloud.normals(idx_new_points,:);vec_pt_cloud.normals(idx_update_points,:)];
map_ccount = [fusion_map.ccounts(idx_fusion_to_keep);vec_pt_cloud.ccounts(idx_new_points);vec_pt_cloud.ccounts(idx_update_points)];
map_range = [fusion_map.range(idx_fusion_to_keep);vec_pt_cloud.range(idx_new_points);vec_pt_cloud.range(idx_update_points)];
new_fusion_map = struct('pointcloud', map_pointcloud, 'normals', map_normals,'ccounts', map_ccount,'range', map_range);

%Refined point cloud to be updated based on flag_update
frame_pointcloud = vec_pt_cloud.pointcloud; 
frame_pointcloud(idx_update_points,:) = moved_vec_pt_cloud(idx_update_points,:);
frame_normals = vec_pt_cloud.normals;
frame_ccount = vec_pt_cloud.ccounts;
frame_range = vec_pt_cloud.range;

ref_vec_pt_cloud = struct('pointcloud', frame_pointcloud, 'normals', frame_normals,'ccounts', frame_ccount,'range', frame_range);


end

