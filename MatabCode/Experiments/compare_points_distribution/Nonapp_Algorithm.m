function [R_final,t_final] = Nonapp_Algorithm(p_w,p_si_noise,phi_max,R_true,py_path)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %  修改时间：2024.10.30
    %  输入：p_w:3D点的世界坐标, 3*n
    %        p_si_noise: 测得的投影点坐标，2*n
    %        phi_max: 最大的俯仰角，单位：rad。用于tz和CIO
    %        R_true: 真实的旋转矩阵用于判断R的符号
    %  输出：R_final,t_final: 5Dof Nonapp algprithm + SLSQP算tz得到的R和t
    %      
    num = length(p_si_noise);
    for i = 1:num
        W_Nonapp(i, 1) = -p_si_noise(2, i);
        W_Nonapp(i, 2) = p_si_noise(1, i);
        H_Nonapp(i, 1:3) = -p_si_noise(2, i) * p_w(:, i)';
        H_Nonapp(i, 4:6) = p_si_noise(1, i) * p_w(:, i)';
    end
    
    % 计算矩阵 M_Noise_He
    M_Nonapp = W_Nonapp * inv(W_Nonapp' * W_Nonapp) * W_Nonapp' * H_Nonapp - H_Nonapp;
    % 
    % 对 M_Noise_He 进行 SVD 分解
    [U_Nonapp, S_Nonapp, V_Nonapp] = svd(M_Nonapp);
    
    % 计算 r_1 和 r_2
    r_1 = sqrt(2) * V_Nonapp(:, 6);
    r_2 = -sqrt(2) * V_Nonapp(:, 6);
    
    if r_1(1,:)*R_true(1,1) > 0
        r = r_1; 
        t_Nonapp = -inv(W_Nonapp' * W_Nonapp) * W_Nonapp' * H_Nonapp * r_1;
    else
        r = r_2; 
        t_Nonapp = -inv(W_Nonapp' * W_Nonapp) * W_Nonapp' * H_Nonapp * r_2;
    end
    % r=r_1;
    % t_S_Noise_He = inv(W_Noise_He'*W_Noise_He)*W_Noise_He'*H_Noise_He * r_2;
    R_Nonapp=zeros(3,3);
    R_Nonapp(1,1:3)=r(1:3)';
    R_Nonapp(2,1:3)=r(4:6)';
    R_Nonapp(3,1:3)=cross(r(1:3),r(4:6));
    
    
    
    % 判断是否为旋转矩阵并进行矫正
    if ~isRotationMatrix(R_Nonapp)
        R_Nonapp = ToRotationMatrix(R_Nonapp);
    else
        R_Nonapp = R_Nonapp;
    end
    
    r_opt = [R_Nonapp(1,1:3),R_Nonapp(2,1:3)]';
    t_Nonapp = -inv(W_Nonapp' * W_Nonapp) * W_Nonapp' * H_Nonapp * r_opt;
    
    t_Nonapp = [t_Nonapp;0.0];

    % 将包含 Python 脚本的目录添加到 Python 路径
%     py_path = 'D:\TeXstudio\A_what_I_Done\SLAM\IROS\IROS_V_2\参考代码\my_code\实验_2024.10.19\simulation_experiments\test_pointnumber_influence'; % 将此更改为你的脚本目录
    py.importlib.import_module('sys');
    py.sys.path().append(py_path);
    
    py_fun = py.importlib.import_module('calculate_tz');  % 导入模块

    result_Nonapp_t = py.calculate_tz.calculate_tz(R_Nonapp, t_Nonapp, p_w, p_si_noise, phi_max);

    t_Nonapp  = double(result_Nonapp_t)';

    R_final = R_Nonapp';
    t_final = -R_Nonapp'*t_Nonapp;

end

