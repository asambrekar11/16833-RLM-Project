function [pc_map, pc_range, flag] = projPCtoImg(msg) 
%function to convert pointcloud2 into azimuth-elevation image
%
resolution_azi = 0.1;
resolution_ver = 2;
h = 16;
w = 360/resolution_azi;

%convert pc to alpha, omega
alpha = round(wrapTo360(atan2(msg(:,1),msg(:,2))*180/pi)/resolution_azi);
omega = (16+atan2(msg(:,3),sqrt(msg(:,1).^2+msg(:,2).^2))*180/pi)/resolution_ver;
alpha(alpha==0) = 1;


coord = [alpha ceil(omega)];
lin_idx = sub2ind([h w],coord(:,2),coord(:,1));

pc_map = zeros(h*w,3);
pc_range = zeros(h*w,1);
pc_map(lin_idx,:) = msg;
pc_range(lin_idx,:) = sqrt(msg(:,1).^2+msg(:,2).^2+msg(:,3).^2);

pc_map = reshape(pc_map,[h w 3]);
pc_range = reshape(pc_range,[h w]);

flag = find(pc_map(:,:,1)==0);

flag = [flag;flag+h*w;flag+2*h*w];

% range = sqrt(msg(:,1).^2+msg(:,2).^2+msg(:,3).^2);
% scatter3(coord(:,2),coord(:,1),range,1);
% figure
% scatter3(msg(:,1),msg(:,2),msg(:,3),1);
end
