function [R_final_CIO,t_final_CIO] = Combine_CIO_2(p_w,p_si_noise,phi_max,R_true,py_path)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % 修改时间2024.10.30
    %  输入：p_w:3D点的世界坐标, 3*n
    %        p_si_noise: 测得的投影点坐标，2*n
    %        phi_max: 最大的俯仰角，单位：rad。用于tz和CIO
    %        py_path：Python 脚本的目录添加到 Python 路径
    %  输出：R_final,t_final: 5Dof Nonapp algprithm + SLSQP算tz得到的R和t
    %       R_final_CIO,t_final_CIO: 再用CIO，优化得到的R和t
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %-------------------------------------------------------------------
    %Nonapp


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


    %-------------------------------------------------------------------
    % App
    % 得先通过转换得到以第一个点作为世界坐标系的坐标
    p_w_app = p_w - p_w(:,1);

    t_app = p_si_noise(:,1);

    A_app = [];
    b_app = [];
    for i = 2:length(p_w)
        A_app = [A_app ; [p_w_app(:,i)', 0, 0, 0; 0, 0, 0, p_w_app(:,i)']];
        b_app = [b_app ; p_si_noise(:,i) - t_app];

    end
    r_app = inv(A_app'*A_app) * A_app' * b_app;

    r_app(1:3,:) = r_app(1:3,:)/norm(r_app(1:3,:));
    r_app(4:6,:) = r_app(4:6,:)/norm(r_app(4:6,:));

    % 计算矩阵的秩
    rank_A = rank(A_app);
    min_dim_A = min(size(A_app));

    % 判断是否满秩
    if rank_A ~= min_dim_A
        disp('矩阵A_app不是满秩的');

        [U_Noise_He_app, S_Noise_He_app, V_Noise_He_app] = svd(A_app);
        v_1_1 = V_Noise_He_app(1:3, 5);
        v_1_2 = V_Noise_He_app(4:6, 5);
        v_2_1 = V_Noise_He_app(1:3, 6);
        v_2_2 = V_Noise_He_app(4:6, 6);

        % 构造矩阵F
        F = zeros(3, 3);
        F(1,1) = v_1_1' * v_1_1;
        F(1,2) = v_2_1' * v_2_1;
        F(1,3) = 2 * v_1_1' * v_2_1;
        F(2,1) = v_1_2' * v_1_2;
        F(2,2) = v_2_2' * v_2_2;
        F(2,3) = 2 * v_1_2' * v_2_2;
        F(3,1) = v_1_1' * v_1_2;
        F(3,2) = v_2_1' * v_2_2;
        F(3,3) = v_1_1' * v_2_2 + v_1_2' * v_2_1;

        c = [1; 1; 0];

        % 定义目标函数
        objective = @(x) norm(F * [x(1)^2; x(2)^2; x(1) * x(2)] - c, 2)^2;

        % 定义初始猜测值
        x0 = [0.5; 0.5];

        % 使用 fminunc 进行非线性优化求解
        options_a = optimoptions('fminunc', 'Algorithm', 'quasi-newton', 'Display', 'off');
        [x_opt, fval] = fminunc(objective, x0, options_a);
        alpha_1 = x_opt(1);
        alpha_2 = x_opt(2);

        r_app = r_app + alpha_1 * V_Noise_He_app(:, 5) + alpha_2 * V_Noise_He_app(:, 6);
    end

    R_app = zeros(3, 3);
    R_app(1, 1:3) = r_app(1:3);  
    R_app(2, 1:3) = r_app(4:6); 
    R_app(3, 1:3) = cross(r_app(1:3), r_app(4:6)); 

    % 判断是否为旋转矩阵并进行矫正
    if ~isRotationMatrix(R_app)
        R_app = ToRotationMatrix(R_app);
    end
    
    t_app = [t_app;0.0];


        % 调用 Python 函数
    % 将包含 Python 脚本的目录添加到 Python 路径
%     py_path = 'D:\TeXstudio\A_what_I_Done\SLAM\IROS\IROS_V_2\参考代码\my_code\实验_2024.10.19\simulation_experiments\test_pointnumber_influence'; % 将此更改为你的脚本目录
    py.importlib.import_module('sys');
    py.sys.path().append(py_path);
    
    py_fun = py.importlib.import_module('calculate_tz');  % 导入模块

    %计算 tz
    result_app_t = py.calculate_tz.calculate_tz(R_app, t_app, p_w_app, p_si_noise, phi_max);

    % t_app 是第一个3D点作为世界坐标系相对声呐坐标系的位移
    t_app  = double(result_app_t)';

    % t_app_w 是原始世界坐标系相对声呐坐标系的位移
    t_app_w = -R_app'*t_app;
    t_app_w = p_w(:,1) + t_app_w;

    %t_app_new 将 t_app_w重新变回app中迭代用的t
    t_app_new = -R_app*t_app_w;

    

    % 计算投影误差
    Temp_R_Nonapp = R_Nonapp';
    Temp_R_app = R_app';

    Temp_t_Nonapp = -R_Nonapp'*t_Nonapp;
    Temp_t_app = -R_app'*t_app_new;


    p_s_est_Nonapp = Temp_R_Nonapp*(p_w- Temp_t_Nonapp);
    d_est_Nonapp = vecnorm(p_s_est_Nonapp);
    tan_theta_Nonapp = p_s_est_Nonapp(2,:)./p_s_est_Nonapp(1,:);
    theta_est_Nonapp = atan(tan_theta_Nonapp);
    p_si_est_Nonapp(1,:) = d_est_Nonapp.*cos(theta_est_Nonapp);
    p_si_est_Nonapp(2,:) = d_est_Nonapp.*sin(theta_est_Nonapp);
    
    p_s_est_app = Temp_R_app*(p_w - Temp_t_app);
    d_est_app = vecnorm(p_s_est_app);
    tan_theta_app = p_s_est_app(2,:)./p_s_est_app(1,:);
    theta_est_app = atan(tan_theta_app);
    p_si_est_app(1,:) = d_est_app.*cos(theta_est_app);
    p_si_est_app(2,:) = d_est_app.*sin(theta_est_app);

    error_Nonapp = mean(vecnorm(p_si_est_Nonapp-p_si_noise));
    error_app = mean(vecnorm(p_si_est_app-p_si_noise));

    if error_Nonapp>error_app
        R = R_app;
        t = t_app_new;
    else
        R = R_Nonapp;
        t = t_Nonapp;
    end

    py_CIO_fun = py.importlib.import_module('CIO');  % 导入模块

    result_CIO = py.CIO.CIO(R, t, p_w, p_si_noise, phi_max);
    
    R_CIO = double(result_CIO{1});
    t_CIO  = double(result_CIO{2})';

    R_final_CIO  = R_CIO';
    t_final_CIO  = -R_CIO'*t_CIO;
end

