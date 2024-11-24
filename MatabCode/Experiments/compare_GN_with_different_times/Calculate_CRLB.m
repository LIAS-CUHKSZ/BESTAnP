function [R_CRLB,t_CRLB] = Calculate_CRLB(p_w,R_true,t_true,stdVar_noise_d,stdVar_noise_theta)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %  输入：p_w:3D点的世界坐标, 3*n
    %        p_si_noise: 测得的投影点坐标，2*n
    %        R_true: 真实的旋转矩阵 3*3
    %        t_true: 真实的平移矢量 3*1
    %        stdVar_noise_d: 距离噪声标准差
    %        stdVar_noise_theta：方位角噪声标准差
    %  输出：
    %       R_CRLB,t_CRLB: R和t的CRLB
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    num = length(p_w);

    e_1 = [1 0 0]';
    e_2 = [0 1 0]';
    I = eye(3);
    F = zeros(6,6);
    for i = 1:num

        phi_function = -[0,0,0;
             0,0,1;
             0,-1,0;
             0,0,-1;
             0,0,0;
             1,0,0;
             0,1,0;
             -1,0,0;
             0,0,0];
    
        u = p_w(:,i)-t_true;
    
        f_d_s = [0,0,0];
        f_d_t = u'/norm(u);


        R_true_T = R_true';
        vec_R_T = R_true_T(:);
        g = e_2'*kron((p_w(:,i)-t_true)',eye(3))*vec_R_T;
        h = e_1'*kron((p_w(:,i)-t_true)',eye(3))*vec_R_T;
    
%         ukronR = kron((p_w(:,i)-t_true)',R_true_T);
        ukronR = kron((p_w(:,i)-t_true)'*R_true,I); 
        f_theta_s = (g*e_1'-h*e_2')*ukronR*phi_function/h^2;
        f_theta_t = (h*e_2'-g*e_1')*R_true_T/h^2;
    
        A = 1/stdVar_noise_theta^2*f_theta_s'*f_theta_s;
        B = 1/stdVar_noise_theta^2*f_theta_s'*f_theta_t;
        C = 1/stdVar_noise_theta^2*f_theta_t'*f_theta_s;
        D = 1/stdVar_noise_d^2*f_d_t'*f_d_t+1/stdVar_noise_theta^2*f_theta_t'*f_theta_t;
    
%         A = f_theta_s'*f_theta_s;
%         B = f_theta_s'*f_theta_t;
%         C = f_theta_t'*f_theta_s;
%         D = f_d_t'*f_d_t+f_theta_t'*f_theta_t;
%     
        F = F + [A,B;C,D];

    
    
    end
    
    CRLB = inv(F);
    
    R_CRLB = trace(CRLB(1:3,1:3));
    t_CRLB = trace(CRLB(4:6,4:6));
end