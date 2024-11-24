clc;
clear;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 测试真实场景 Cube 中，BESTAnP, Non-app, app 和 Combine_CIO算法的重投影误差以及运行时间
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% 设置环境变量，允许 OpenMP 运行时重复加载
setenv('KMP_DUPLICATE_LIB_OK', 'TRUE');  %防止报错

% 将此更改为你的脚本目录
py_path = 'C:\Users\Lenovo\Desktop\Matab Code\Experiments\reprojection experiments'; 


d = [734.8068567	553.1355344	457.4443685	513.638646
756.8482929	576.5455273	491.8271107	564.559381
787.5916548	615.2932375	537.7823877	622.1255243
721.5814163	555.2233147	592.1032429	667.2715242
660.4829052	497.7731441	654.2472485	710.4606919
666.5610587	482.5995504	512.0674918	557.2513694
691.1024746	512.959746	551.2097915	610.22209
599.2827563	417.0733425	584.2535832	617.170785
623.3066798	453.8379727	613.5603705	659.1071782
]';

d =d/1000;
 
theta =[-5.05382605	-10.19720557	12.57390473	3.149020419
0.125648438	-2.640010419	5.422000315	-1.684684318
4.593178726	3.557052606	-0.530501317	-5.664963378
7.53368117	7.040039356	3.696556152	-0.712597329
10.71694656	12.3215906	7.287275832	4.018860742
-3.140261218	-6.318431101	16.37906004	8.564518643
2.477598868	1.112399616	10.05794994	3.430519167
-1.110859283	-2.737156305	19.4129489	13.21405128
5.194428908	5.877392607	13.13402231	8.252529047
]'/180*pi;

theta = -theta;
tan_theta = tan(theta);
cos_theta = cos(theta);
sin_theta = sin(theta);


num = size(d,1)
for i=1:num
    p_si(2*i-1,:) = d(i,:).* cos_theta(i,:);
    p_si(2*i,:) = d(i,:).* sin_theta(i,:);
end



% p_si = [d.*cos_theta;d.*sin_theta];

p_w = [0	0	0	0	0	0	0	0	0	0	0	0
73.5	0	0	73.5	0	0	73.5	0	0	73.5	0	0
147	0	0	147	0	0	147	0	0	147	0	0
147	-73.5	0	147	-73.5	0	147	-73.5	0	147	-73.5	0
147	-147	0	147	-147	0	147	-147	0	147	-147	0
0	-73.5	73.5	0	-73.5	73.5	0	-73.5	73.5	0	-73.5	73.5
73.5	-73.5	73.5	73.5	-73.5	73.5	73.5	-73.5	73.5	73.5	-73.5	73.5
0	-147	73.5	0	-147	73.5	0	-147	73.5	0	-147	73.5
73.5	-147	73.5	73.5	-147	73.5	73.5	-147	73.5	73.5	-147	73.5
]';

p_w = p_w/1000;

var = 0;

phi_max = 6*pi/180;

num_group = size(p_w,1)/3;

num_points = size(p_w,2);

M=100;

BESTAnP_time = 0;
Nonapp_time = 0;
App_time = 0;
Combine_CIO_time= 0;

for j = 1:M
    for i = 1:1
        tic;
        [R,t] = BESTAnP(p_w(3*i-2:3*i,:),p_si(2*i-1:2*i,:),0.0034,0.0033);
        BESTAnP_time = BESTAnP_time + toc;
    end
end
BESTAnP_time = BESTAnP_time/100*1000;



for j = 1:M
    for i = 1:1
        tic;
        [R_Nonapp,t_Nonapp] = Nonapp_Algorithm(p_w(3*i-2:3*i,:),p_si(2*i-1:2*i,:),phi_max,R,py_path);
        Nonapp_time = Nonapp_time+toc;
    end
end
Nonapp_time = BESTAnP_time/100*1000;


for j = 1:M
    for i = 1:1
        tic;
        [R_app,t_app] = App_Algorithm(p_w(3*i-2:3*i,:),p_si(2*i-1:2*i,:),phi_max,py_path);
        App_time = App_time+toc;
    end   
end    
App_time = App_time/100*1000;


for j = 1:M
    for i = 1:1
            tic;
         [R_Combine_CIO,t_Combine_CIO] = Combine_CIO(p_w(3*i-2:3*i,:),p_si(2*i-1:2*i,:),phi_max,R,py_path);
         Combine_CIO_time =Combine_CIO_time+toc;
    end
end
Combine_CIO_time = Combine_CIO_time/100*1000;


for i = 1:num_group

    [R,t] = BESTAnP(p_w(3*i-2:3*i,:),p_si(2*i-1:2*i,:),0.0034,0.0033);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    p_s_est = R'*(p_w(3*i-2:3*i,:) - t);
    d_est = vecnorm(p_s_est);
    tan_theta = p_s_est(2,:)./p_s_est(1,:);
    theta_est = atan(tan_theta);
    p_si_est(1,:) = d_est.*cos(theta_est);
    p_si_est(2,:) = d_est.*sin(theta_est);

    error(i,:) = norm(p_si_est-p_si(2*i-1:2*i,:))/num_points;
    error_d(i,:) = norm(d_est--d(i,:))/num_points;
    error_theta(i,:) = norm(theta_est - theta(i,:))/num_points;
    
    
   
    d_est = d_est';
    theta_est = theta_est';
    
end


for i = 1:num_group
    [R_Nonapp,t_Nonapp] = Nonapp_Algorithm(p_w(3*i-2:3*i,:),p_si(2*i-1:2*i,:),phi_max,R,py_path);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    p_s_est_Nonapp = R_Nonapp'*(p_w(3*i-2:3*i,:) - t_Nonapp);
    d_est_Nonapp = vecnorm(p_s_est_Nonapp);
    tan_theta_Nonapp = p_s_est_Nonapp(2,:)./p_s_est_Nonapp(1,:);
    theta_est_Nonapp = atan(tan_theta_Nonapp);
    p_si_est_Nonapp(1,:) = d_est_Nonapp.*cos(theta_est_Nonapp);
    p_si_est_Nonapp(2,:) = d_est_Nonapp.*sin(theta_est_Nonapp);

    error_Nonapp(i,:)  = norm(p_si_est_Nonapp-p_si(2*i-1:2*i,:))/num_points;
    error_Nonapp_d(i,:) = norm(d_est_Nonapp--d(i,:))/num_points;
    error_Nonapp_theta(i,:) = norm(theta_est_Nonapp - theta(i,:))/num_points;

    

    d_est_Nonapp = d_est_Nonapp';
    theta_est_Nonapp = theta_est_Nonapp';
   
end


for i = 1:num_group
    [R_app,t_app] = App_Algorithm(p_w(3*i-2:3*i,:),p_si(2*i-1:2*i,:),phi_max,py_path);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    p_s_est_app = R_app'*(p_w(3*i-2:3*i,:) - t_app);
    d_est_app = vecnorm(p_s_est_app);
    tan_theta_app = p_s_est_app(2,:)./p_s_est_app(1,:);
    theta_est_app = atan(tan_theta_app);
    p_si_est_app(1,:) = d_est_app.*cos(theta_est_app);
    p_si_est_app(2,:) = d_est_app.*sin(theta_est_app);

    error_app(i,:)  = norm(p_si_est_app-p_si(2*i-1:2*i,:))/num_points;
    error_app_d(i,:) = norm(d_est_app--d(i,:))/num_points;
    error_app_theta(i,:) = norm(theta_est_app - theta(i,:))/num_points;
   

    d_est_app = d_est_app';
    theta_est_app = theta_est_app';
    
end   


for i = 1:num_group
    [R_Combine_CIO,t_Combine_CIO] = Combine_CIO(p_w(3*i-2:3*i,:),p_si(2*i-1:2*i,:),phi_max,R,py_path);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    p_s_est_Combine_CIO = R_Combine_CIO'*(p_w(3*i-2:3*i,:) - t_Combine_CIO);
    d_est_Combine_CIO = vecnorm(p_s_est_Combine_CIO);
    tan_theta_Combine_CIO = p_s_est_Combine_CIO(2,:)./p_s_est_Combine_CIO(1,:);
    theta_est_Combine_CIO = atan(tan_theta_Combine_CIO);
    p_si_est_Combine_CIO(1,:) = d_est_Combine_CIO.*cos(theta_est_Combine_CIO);
    p_si_est_Combine_CIO(2,:) = d_est_Combine_CIO.*sin(theta_est_Combine_CIO);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
    error_Combine_CIO(i,:)  = norm(p_si_est_Combine_CIO-p_si(2*i-1:2*i,:))/num_points;
    error_Combine_CIO_d(i,:) = norm(d_est_Combine_CIO--d(i,:))/num_points;
    error_Combine_CIO_theta(i,:) = norm(theta_est_Combine_CIO - theta(i,:))/num_points;

    d_est_Combine_CIO = d_est_Combine_CIO';
    theta_est_Combine_CIO =theta_est_Combine_CIO';
end





   

error_d = error_d';
error_Nonapp_d = error_Nonapp_d';
error_app_d = error_app_d';
error_Combine_CIO_d = error_Combine_CIO_d';

error_theta = error_theta';
error_Nonapp_theta = error_Nonapp_theta';
error_app_theta = error_app_theta';
error_Combine_CIO_theta = error_Combine_CIO_theta';



error = error';
error_Nonapp  = error_Nonapp';
error_app = error_app';
error_Combine_CIO = error_Combine_CIO';


