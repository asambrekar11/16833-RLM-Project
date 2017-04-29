function [ pc_linkage ] = compute_linkage( pc_map,pc_range, theta1, theta2)
%Calculate linkage based on the following code
% proj_linkage = proj_linkage(:,:,right(1)||left(2)||bottom(3)||up(4));

pc_linkage = zeros(size(pc_map,1),size(pc_map,2),4);

aug_range = [pc_range(:,end) pc_range(:,end-1) pc_range(:,:) pc_range(:,1) pc_range(:,2)];

%Linkages for l, r, u and b
proj_linkage_l_1 = zeros(size(pc_map,1),size(pc_map,2));
proj_linkage_l_2 = zeros(size(pc_map,1),size(pc_map,2));
proj_linkage_r_1 = zeros(size(pc_map,1),size(pc_map,2));
proj_linkage_r_2 = zeros(size(pc_map,1),size(pc_map,2));
proj_linkage_u_1 = zeros(size(pc_map,1),size(pc_map,2));
proj_linkage_u_2 = zeros(size(pc_map,1),size(pc_map,2));
proj_linkage_b_1 = zeros(size(pc_map,1),size(pc_map,2));
proj_linkage_b_2 = zeros(size(pc_map,1),size(pc_map,2));

%Right linkage factors
% proj_linkage_r_1(:,2:end-1) = abs((pc_range(:,2:end-1)-pc_range(:,3:end)-(pc_range(:,1:end-2)-pc_range(:,2:end-1)))./(pc_range(:,1:end-2)-pc_range(:,2:end-1)));
% proj_linkage_r_2(:,1:end-2) = abs((pc_range(:,1:end-2)-pc_range(:,2:end-1)-(pc_range(:,2:end-1)-pc_range(:,3:end)))./(pc_range(:,2:end-1)-pc_range(:,3:end)));

proj_linkage_r_1 = abs((aug_range(:,3:end-2)-aug_range(:,4:end-1))./(aug_range(:,2:end-3)-aug_range(:,3:end-2))-1);
proj_linkage_r_2 = abs((aug_range(:,3:end-2)-aug_range(:,4:end-1))./(aug_range(:,4:end-1)-aug_range(:,5:end))-1);


sig_r_1 = 0.5 - 0.5*(proj_linkage_r_1-theta1)*theta2./(sqrt(1+(proj_linkage_r_1-theta1).^2*theta2^2));
sig_r_1 = sig_r_1(:);
sig_r_2 = 0.5 - 0.5*(proj_linkage_r_2-theta1)*theta2./(sqrt(1+(proj_linkage_r_2-theta1).^2*theta2^2));
sig_r_2 = sig_r_2(:);

proj_linkage_r_1 = reshape(min(sig_r_1,sig_r_2),size(proj_linkage_r_1));
proj_linkage_r_1(isnan(proj_linkage_r_1)) = 0;

%Left linkage factors
% proj_linkage_l_1(:,2:end-1) = abs(pc_range(:,2:end-1)-pc_range(:,1:end-2)-(pc_range(:,3:end)-pc_range(:,2:end-1)))./(pc_range(:,3:end)-pc_range(:,2:end-1));
% proj_linkage_l_2(:,3:end) = abs(pc_range(:,3:end)-pc_range(:,2:end-1)-(pc_range(:,2:end-1)-pc_range(:,1:end-2)))./(pc_range(:,2:end-1)-pc_range(:,1:end-2));

proj_linkage_l_1 = abs((aug_range(:,3:end-2)-aug_range(:,2:end-3))./(aug_range(:,4:end-1)-aug_range(:,3:end-2))-1);
proj_linkage_l_2 = abs((aug_range(:,3:end-2)-aug_range(:,2:end-3))./(aug_range(:,2:end-3)-aug_range(:,1:end-4))-1);

sig_l_1 = 0.5 - 0.5*(proj_linkage_l_1-theta1)*theta2./(sqrt(1+(proj_linkage_l_1-theta1).^2*theta2^2));
sig_l_1 = sig_l_1(:);
sig_l_2 = 0.5 - 0.5*(proj_linkage_l_2-theta1)*theta2./(sqrt(1+(proj_linkage_l_2-theta1).^2*theta2^2));
sig_l_2 = sig_l_2(:);

proj_linkage_l_1 = reshape(min(sig_l_1,sig_l_2),size(proj_linkage_l_1));
proj_linkage_l_1(isnan(proj_linkage_l_1)) = 0;


%Bottom linkage factors
% proj_linkage_b_1(2:end-1,:) = abs(pc_range(2:end-1,:)-pc_range(3:end,:)-(pc_range(1:end-2,:)-pc_range(2:end-1,:)))./(pc_range(1:end-2,:)-pc_range(2:end-1,:));
% proj_linkage_b_2(1:end-2,:) = abs(pc_range(1:end-2,:)-pc_range(2:end-1,:)-(pc_range(2:end-1,:)-pc_range(3:end,:)))./(pc_range(2:end-1,:)-pc_range(3:end,:));

proj_linkage_b_1(2:end-1,:) = abs((pc_range(2:end-1,:)-pc_range(3:end,:))./(pc_range(1:end-2,:)-pc_range(2:end-1,:))-1);
proj_linkage_b_2(1:end-2,:) = abs((pc_range(1:end-2,:)-pc_range(2:end-1,:))./(pc_range(2:end-1,:)-pc_range(3:end,:))-1);

sig_b_1 = 0.5 - 0.5*(proj_linkage_b_1-theta1)*theta2./(sqrt(1+(proj_linkage_b_1-theta1).^2*theta2^2));
sig_b_1 = sig_b_1(:);
sig_b_2 = 0.5 - 0.5*(proj_linkage_b_2-theta1)*theta2./(sqrt(1+(proj_linkage_b_2-theta1).^2*theta2^2));
sig_b_2 = sig_b_2(:);

proj_linkage_b_1 = reshape(min(sig_b_1,sig_b_2),size(proj_linkage_b_1));
proj_linkage_b_1(isnan(proj_linkage_b_1)) = 0;

%Up linkage factors
% proj_linkage_u_1(2:end-1,:) = abs(pc_range(2:end-1,:)-pc_range(1:end-2,:)-(pc_range(3:end,:)-pc_range(2:end-1,:)))./(pc_range(3:end,:)-pc_range(2:end-1,:));
% proj_linkage_u_2(3:end,:) = abs(pc_range(3:end,:)-pc_range(2:end-1,:)-(pc_range(2:end-1,:)-pc_range(1:end-2,:)))./(pc_range(2:end-1,:)-pc_range(1:end-2,:));

proj_linkage_u_1(2:end-1,:) = abs((pc_range(2:end-1,:)-pc_range(1:end-2,:))./(pc_range(3:end,:)-pc_range(2:end-1,:))-1);
proj_linkage_u_2(3:end,:) = abs((pc_range(3:end,:)-pc_range(2:end-1,:))./(pc_range(2:end-1,:)-pc_range(1:end-2,:))-1);


sig_u_1 = 0.5 - 0.5*(proj_linkage_u_1-theta1)*theta2./(sqrt(1+(proj_linkage_u_1-theta1).^2*theta2^2));
sig_u_1 = sig_u_1(:);
sig_u_2 = 0.5 - 0.5*(proj_linkage_u_2-theta1)*theta2./(sqrt(1+(proj_linkage_u_2-theta1).^2*theta2^2));
sig_u_2 = sig_u_2(:);

proj_linkage_u_1 = reshape(min(sig_u_1,sig_u_2),size(proj_linkage_u_1));
proj_linkage_u_1(isnan(proj_linkage_u_1)) = 0;

%Final assignment of linkages
pc_linkage(:,:,1) = proj_linkage_r_1;
pc_linkage(:,:,2) = proj_linkage_l_1;
pc_linkage(:,:,3) = proj_linkage_b_1;
pc_linkage(:,:,4) = proj_linkage_u_1;
end

