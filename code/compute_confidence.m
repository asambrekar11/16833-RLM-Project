function [ pc_confidence ] = compute_confidence( pc_map,pc_normals,pc_linkage,theta3 )
%Computing confidence of the projected point cloud

%Lateral points augmentation
aug_pc_map = [pc_map(:,end,:) pc_map pc_map(:,1,:)];

%Distance between four neighbors
D_i_i1 = zeros(size(pc_map));
D_i_i2 = zeros(size(pc_map));
D_i_i3 = zeros(size(pc_map));
D_i_i4 = zeros(size(pc_map));

%Confidence in plain assumption 
C_i_i1 = zeros(size(pc_map,1),size(pc_map,2));
C_i_i2 = zeros(size(C_i_i1));
C_i_i3 = zeros(size(C_i_i1));
C_i_i4 = zeros(size(C_i_i1));

%Populating the distance vectors
D_i_i1 = aug_pc_map(:,3:end,:) - aug_pc_map(:,2:end-1,:);
D_i_i2(2:end,:,:) = pc_map(1:end-1,:,:)-pc_map(2:end,:,:);
D_i_i3 = aug_pc_map(:,1:end-2,:) - aug_pc_map(:,2:end-1,:);
D_i_i4(1:end-1,:,:) = pc_map(2:end,:,:)-pc_map(1:end-1,:,:);

%Unit distance vectors
D_i_i1_unit = D_i_i1./repmat(sqrt(sum(D_i_i1.^2,3)),[1 1 3]);
D_i_i2_unit = D_i_i2./repmat(sqrt(sum(D_i_i2.^2,3)),[1 1 3]);
D_i_i3_unit = D_i_i3./repmat(sqrt(sum(D_i_i3.^2,3)),[1 1 3]);
D_i_i4_unit = D_i_i4./repmat(sqrt(sum(D_i_i4.^2,3)),[1 1 3]);

%Neighborhood confidence
C_i_i1 = exp(-theta3*asin(abs(dot(D_i_i1_unit,pc_normals,3))).^2);
C_i_i2 = exp(-theta3*asin(abs(dot(D_i_i2_unit,pc_normals,3))).^2);
C_i_i3 = exp(-theta3*asin(abs(dot(D_i_i3_unit,pc_normals,3))).^2);
C_i_i4 = exp(-theta3*asin(abs(dot(D_i_i4_unit,pc_normals,3))).^2);

%Sanitizing confidence
C_i_i1(isnan(C_i_i1)) = exp(-theta3*pi^2/4);
C_i_i2(isnan(C_i_i2)) = exp(-theta3*pi^2/4);
C_i_i3(isnan(C_i_i3)) = exp(-theta3*pi^2/4);
C_i_i4(isnan(C_i_i4)) = exp(-theta3*pi^2/4);

%Linkage maximum confidence

L_max =max(max(pc_linkage(:,:,1).*pc_linkage(:,:,2),pc_linkage(:,:,2).*pc_linkage(:,:,3)),max(pc_linkage(:,:,3).*pc_linkage(:,:,4),pc_linkage(:,:,4).*pc_linkage(:,:,1)));

%Output confidence at a pixel
pc_confidence = min(L_max,max(C_i_i1.*C_i_i3,C_i_i2.*C_i_i4));

end

