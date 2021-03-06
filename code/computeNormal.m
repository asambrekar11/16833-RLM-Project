function normals = computeNormal(pc_map,linkage)

h = size(pc_map,1);
w = size(pc_map,2);

distance_1 = [pc_map(:,2:end,:) pc_map(:,1,:)] - pc_map; %right
distance_3 = [pc_map(:,end,:) pc_map(:,1:end-1,:)] - pc_map; %left
distance_2 = [zeros(1,w,3);pc_map(1:end-1,:,:)] - pc_map; %up
distance_4 = [pc_map(2:end,:,:);zeros(1,w,3)] - pc_map; %bottom

cross_1 = repmat(linkage(:,:,1).*linkage(:,:,2),[1 1 3]).*cross(distance_1,distance_2,3);
cross_2 = repmat(linkage(:,:,2).*linkage(:,:,3),[1 1 3]).*cross(distance_2,distance_3,3);
cross_3 = repmat(linkage(:,:,3).*linkage(:,:,4),[1 1 3]).*cross(distance_3,distance_4,3);
cross_4 = repmat(linkage(:,:,4).*linkage(:,:,1),[1 1 3]).*cross(distance_4,distance_1,3);

normals = cross_1 + cross_2 + cross_3 + cross_4;

norm_normals = sqrt(sum(normals.^2,3));

normals = normals./repmat(norm_normals,[1 1 3]);

normals(isnan(normals)) = 0;


end