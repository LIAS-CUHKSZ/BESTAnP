%  function [R_first,R_est_noise_new, t_first,t_est_noise_Gau] = final_algorithm(p_w, d_noise, Var_noise_d, Var_noise_theta, tan_theta_noise,cos_theta_noise)

% 带 Theta 的GN
function [R_est_noise_new,t_est_noise] = two_step_algorihtm(p_w, p_si,stdVar_noise_d,stdVar_noise_theta)
    % 先单独对t进行高斯牛顿迭代，再由整体函数对R和t进行高斯牛顿迭代
 

    tan_theta_noise = p_si(2,:)./p_si(1,:);
    cos_theta_noise = p_si(1,:)./vecnorm(p_si(1:2,:));
    sin_theta_noise = p_si(2,:)./vecnorm(p_si(1:2,:));
    theta_noise = atan(tan_theta_noise);

    d_noise = p_si(1,:)./cos_theta_noise;


    % 计算 t 的初始估计值
    num_points = size(p_w, 2);
    
    % 构造矩阵 A
    A_noise = [-2 * p_w', ones(num_points, 1)];
    
    % 构造向量 b
    b_noise = (d_noise.^2 - vecnorm(p_w, 2, 1).^2)';
    
    % 求解 x = [t^T ||t||]^T
    x_noise = (A_noise' * A_noise) \ (A_noise' * b_noise);
    t_est_noise = x_noise(1:3);
    
    % 高斯牛顿迭代
%     residuals = d_noise.^2 - vecnorm(p_w - t_est_noise, 2, 1).^2 - Var_noise_d;
%     J = 2 * (p_w - t_est_noise)';
%     
%     delta_t = (J' * J) \ (J' * residuals');
%     t_est_noise_Gau = t_est_noise - delta_t;


%     t_est_noise_Gau = t_est_noise;

    % 估计 R
    % 构造矩阵 B
    B_noise = [tan_theta_noise' .* (p_w - t_est_noise)', -(p_w - t_est_noise)'];
    
    temp_B_noise = B_noise' * B_noise / num_points;
    
    % 修正矩阵
    temp_matrix = zeros(3, 3);
    for i = 1:num_points
        temp_matrix = temp_matrix + (p_w(:, i) - t_est_noise) * (p_w(:, i) - t_est_noise)';
    end
    temp_matrix = temp_matrix * stdVar_noise_theta^2 / num_points;
    
    modify_matrix = zeros(6, 6);
    modify_matrix(1:3, 1:3) = temp_matrix;
    
    temp_B_modify = temp_B_noise - modify_matrix;
    
    % 求矩阵 temp_B 的特征值和特征向量
    [C_modify, D_modify] = eig(temp_B_modify);
    
    % 解有两种
    
    R_est_noise_1 = [sqrt(2) * C_modify(1:3, 1)'; sqrt(2) * C_modify(4:6, 1)'];
    R_est_noise_2 = [-sqrt(2) * C_modify(1:3, 1)'; -sqrt(2) * C_modify(4:6, 1)'];
    
    % 判断最终解
    p_s_est_noise_1 = R_est_noise_1 * (p_w(:, 1) - t_est_noise);
    
    if p_s_est_noise_1(1) * cos_theta_noise(1) > 0
        R_est_noise = R_est_noise_1;
    else
        R_est_noise = R_est_noise_2;
    end


    R_est_noise(3, :) = cross(R_est_noise(1, :), R_est_noise(2, :));
   
    R_est_noise_new = R_est_noise;

     if ~isRotationMatrix(R_est_noise_new)
        R_est_noise_new = ToRotationMatrix(R_est_noise_new);
     else
        R_est_noise_new = R_est_noise_new;
    end

    R_est_noise_new = R_est_noise_new';



  
end