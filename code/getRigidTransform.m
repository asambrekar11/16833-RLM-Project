function [tform valid_pair_num error] = getRigidTransform(new_pointcloud, ref_pointcloud, ref_normals)
    
    %==== Initialize parameters ====
    iter_num = 6;
    d_th = 0.05;
    m = size(new_pointcloud.Location, 1);
    n = size(new_pointcloud.Location, 2);
    tform = affine3d(eye(4));
    
    %==== Main iteration loop ====
    for iter = 1:iter_num
        
        %==== Set variables ====
        new_pts = new_pointcloud.Location;
        ref_pts = ref_pointcloud.Location;
        
        %==== For each reference point, find the closest new point within a local patch of size 3-by-3 ====        
        %==== (Notice: assoc_pts[] has the same size and format as new_pts[] and ref_pts[]) ====
        %==== (Notice: assoc_pts[i, j, :] = [0 0 0] iff no point in new_pts[] matches to ref_pts[i, j, :]) ====
        assoc_pts = findLocalClosest(new_pts, ref_pts, m, n, d_th);
        
        %==== Set the sizes of matrix A[] and vertor b[] of normal equation: A'*A*x = A'*b ====
        A = zeros(m*n, 6);
        b = zeros(m*n, 1);
        
        %==== declare the number of point pairs that are used in this iteration ==== 
        valid_pair_num = 0;
    
        %==== TODO: Assign values to A[] and b[] ====
        %==== (Notice: the format of the desired 6-vector is: xi = [r_x r_y r_z t_x t_y t_z]') ====
        
        % Write your code here...
        assoc_pts_vec = assoc_pts(:);
        ref_pts_vec = ref_pts(:);
        ref_normals_vec = ref_normals(:);
        for i = 1:m*n
            V_k = [assoc_pts_vec(i);assoc_pts_vec(m*n + i);assoc_pts_vec(2*m*n + i)];
            if(sum(V_k) ~= 0)
                valid_pair_num = valid_pair_num + 1;
                normal_k_1 = [ref_normals_vec(i);ref_normals_vec(m*n + i);ref_normals_vec(2*m*n + i)];
                V_k_1 = [ref_pts_vec(i);ref_pts_vec(m*n + i);ref_pts_vec(2*m*n + i)];
                Gu = [toSkewSym(V_k) eye(3)];
                A(i,:) = (Gu'* normal_k_1)';
                b(i) = normal_k_1'*(V_k_1 - V_k);
            end
        end
        
        
        %==== TODO: Solve for the 6-vector xi[] of rigid body transformation from A[] and b[] ====
        
        % Write your code here...
        
        xi = A\b;
        
        %==== Coerce xi[] back into SE(3) ====
        %==== (Notice: tmp_tform[] is defined in the format of right-multiplication) ====
        R = toSkewSym(xi(1:3)) + eye(3);
        [U,S,V] = svd(R);
        R = U*V';
        T = [R [0 ; 0 ; 0] ; [xi(4:6)' 1]];
        tmp_tform = affine3d(T);
        
        %==== TODO: Update the transformation and the pointcloud for the next iteration ====
        %==== (Hint: use affine3d() and pctransform() functions) ====
        %==== (Hint: be careful of the format of tform[] and the affine3d() function) ====
       
        % Write your code here...
        tform.T = tform.T*tmp_tform.T;
%           tform.T = T;
        
        new_pointcloud = pctransform(new_pointcloud,tmp_tform);
        
    end
    
    %==== Find RMS error of point-plane registration ====
    error = sqrt(sum((A*xi - b).^2)/valid_pair_num);
end
        