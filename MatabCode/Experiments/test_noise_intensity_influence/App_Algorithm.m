function [R_final,t_final] = App_Algorithm(p_w,p_si_noise,phi_max,py_path)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %  修改时间2024.10.30
    %  输入：p_w:3D点的世界坐标, 3*n
    %        p_si_noise: 测得的投影点坐标，2*n
    %        phi_max: 最大的俯仰角，单位：rad。用于tz和CIO
    %       py_path： 将包含 Python 脚本的目录添加到 Python 路径
    %  输出：R_final,t_final: 5Dof Nonapp algprithm + SLSQP算tz得到的R和t
    %     
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


    % 得先通过转换得到以第一个点作为世界坐标系的坐标
    p_w_app = p_w - p_w(:,1);

    t_S_Noise_He_app = p_si_noise(:,1);

    A_app = [];
    b_app = [];
    for i = 2:length(p_w)
        A_app = [A_app ; [p_w_app(:,i)', 0, 0, 0; 0, 0, 0, p_w_app(:,i)']];
        b_app = [b_app ; p_si_noise(:,i) - t_S_Noise_He_app];

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

    R_Noise_He_app = zeros(3, 3);
    R_Noise_He_app(1, 1:3) = r_app(1:3);  
    R_Noise_He_app(2, 1:3) = r_app(4:6); 
    R_Noise_He_app(3, 1:3) = cross(r_app(1:3), r_app(4:6)); 

    % 判断是否为旋转矩阵并进行矫正
    if ~isRotationMatrix(R_Noise_He_app)
        R_Noise_He_app = ToRotationMatrix(R_Noise_He_app);
    end
    
    t_S_Noise_He_app = [t_S_Noise_He_app;0.0];


        % 调用 Python 函数
    % 将包含 Python 脚本的目录添加到 Python 路径
%     py_path = 'D:\TeXstudio\A_what_I_Done\SLAM\IROS\IROS_V_2\参考代码\my_code\实验_2024.10.19\simulation_experiments\test_pointnumber_influence'; % 将此更改为你的脚本目录
    py.importlib.import_module('sys');
    py.sys.path().append(py_path);
    
    py_fun = py.importlib.import_module('calculate_tz');  % 导入模块

    %计算 tz
    result_app_t = py.calculate_tz.calculate_tz(R_Noise_He_app, t_S_Noise_He_app, p_w_app, p_si_noise, phi_max);
    t_S_Noise_He_app  = double(result_app_t)';

    t_S_Noise_He_app_w  = -R_Noise_He_app'*t_S_Noise_He_app;

    t_final = p_w(:,1)+t_S_Noise_He_app_w;
    R_final = R_Noise_He_app';
end

