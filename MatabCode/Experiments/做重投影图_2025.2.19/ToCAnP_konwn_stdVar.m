function [R_est_noise_GN,t_est_noise_GN] = ToCAnP_konwn_stdVar(p_w,p_si_noise,stdVar_d,stdVar_theta)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % ToCAnP 算法 10.28日讨论后的新的公式
    % 修改时间：2024.10.29
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % 输入：p_w:3D点的世界坐标，3*n
    %      p_si_noise: 测量得到的2D投影坐标,2*n
    %
    
    % 处理p_si_noise 获得 d_noise 和 tan_theta_noise
    tan_theta_noise = p_si_noise(2,:)./p_si_noise(1,:);
    cos_theta_noise = p_si_noise(1,:)./vecnorm(p_si_noise(1:2,:));
    sin_theta_noise = p_si_noise(2,:)./vecnorm(p_si_noise(1:2,:));
    theta_noise = atan(tan_theta_noise);
    
    d_noise = p_si_noise(1,:)./cos_theta_noise;
    
    % 获得点数
    num_points = size(p_w, 2);
    
    % ----------------------------------
    % 估计平移矢量 t
    % ----------------------------------
    
    % 构造矩阵 A
    A_noise = [-2 * p_w', ones(num_points, 1)];
    % 构造向量 b
    b_noise = (d_noise.^2 - vecnorm(p_w, 2, 1).^2)'; % 估计噪声方差时，b的形式改变，不需要减去噪声方差
    
    % 求解 x = [[t^T ||t||] + \sigma^2]^T
    x_noise = (A_noise' * A_noise) \ (A_noise' * b_noise);
    t_est_noise = x_noise(1:3);

    
    stdVar_noise_d_est =  stdVar_d;


    

    % 估计 R
    % 构造矩阵 B
    B_noise = [tan_theta_noise' .* (p_w - t_est_noise)', -(p_w - t_est_noise)'];
    
    % 定义 Q 矩阵
    Q = B_noise' * B_noise / num_points;

    % 定义 S 矩阵
    S = [1/num_points * (p_w - t_est_noise) * (p_w - t_est_noise)' zeros(3,3); zeros(3,6)];
    
    
    % 输出估计的 sigma^2
    stdVar_noise_theta_est = stdVar_theta;

    % 定义 C 矩阵
    C = stdVar_noise_theta_est^2*S;

    % 消除偏差
    Q_BE = Q-C;

    [eignvec_matrix, D] = eig(Q_BE);

    R_est_noise_1 = [sqrt(2) * eignvec_matrix(1:3, 1)'; sqrt(2) * eignvec_matrix(4:6, 1)'];
    R_est_noise_2 = [-sqrt(2) * eignvec_matrix(1:3, 1)'; -sqrt(2) * eignvec_matrix(4:6, 1)'];
    % 判断最终解
    p_s_est_noise_1 = R_est_noise_1 * (p_w(:, 1) - t_est_noise);
    
    if p_s_est_noise_1(1) * cos_theta_noise(1) > 0
        R_est_noise = R_est_noise_1;
    else
        R_est_noise = R_est_noise_2;
    end


    R_est_noise(3, :) = cross(R_est_noise(1, :), R_est_noise(2, :));
%     

     if ~isRotationMatrix(R_est_noise)
        R_est_noise = ToRotationMatrix(R_est_noise);
     else
        R_est_noise = R_est_noise;
    end
    R_est_noise = R_est_noise';
    
     % 高斯牛顿优化
    e_1 = [1 0 0]';
    e_2 = [0 1 0]';
    I = eye(3);
    for i = 1:num_points
        %残差
        Residuals_R(2*i-1,1) = (d_noise(1,i)-norm(p_w(:,i)-t_est_noise))/stdVar_noise_d_est;
        Residuals_R(2*i,1) = (tan_theta_noise(1,i)-(e_2'*R_est_noise'*(p_w(:,i)-t_est_noise))/(e_1'*R_est_noise'*(p_w(:,i)-t_est_noise)))/stdVar_noise_theta_est;
    
        J_phi = -[0,0,0;
                 0,0,1;
                 0,-1,0;
                 0,0,-1;
                 0,0,0;
                 1,0,0;
                 0,1,0;
                 -1,0,0;
                 0,0,0];


        R_est_noise_T = R_est_noise';
        vec_R = R_est_noise_T(:);  % 按列依次拼接
%         vec_R = vec_R(:);
%         ukronR = kron((p_w(:,i)-t_est_noise)',R_est_noise);
        ukronR = kron((p_w(:,i)-t_est_noise)'*R_est_noise,I); 
        g = e_2'*kron((p_w(:,i)-t_est_noise)',eye(3))*vec_R;
        h = e_1'*kron((p_w(:,i)-t_est_noise)',eye(3))*vec_R;
        % vec(R_est_noise_new) 改成了 vec_R
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        %雅可比矩阵
        J_R(2*i-1,1:3) = [0,0,0];
        J_R(2*i-1,4:6) = (p_w(:,i)-t_est_noise)'/norm(p_w(:,i)-t_est_noise)/stdVar_noise_d_est;
        J_R(2*i,1:3) = -(h*e_2'-g*e_1')*ukronR*J_phi/h^2/stdVar_noise_theta_est;
        J_R(2*i,4:6) = (h*e_2'-g*e_1')*R_est_noise'/h^2/stdVar_noise_theta_est;
    
    end
    %t_first=t_est_noise_Gau;
    temp_result = [0;0;0;t_est_noise] - inv(J_R'*J_R)*J_R'*Residuals_R;

    t_est_noise_GN = temp_result(4:6);
    s_new = temp_result(1:3);
    s_matrix = [0,-s_new(3),s_new(2);
                s_new(3),0,-s_new(1);
                -s_new(2),s_new(1),0];

   
   % R_first=R_est_noise_new;
    R_est_noise_GN = R_est_noise*expm(s_matrix);
end