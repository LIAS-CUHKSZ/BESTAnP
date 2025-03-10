clc;
clear;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 测试方块 长的数据 10.31
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% 设置环境变量，允许 OpenMP 运行时重复加载
setenv('KMP_DUPLICATE_LIB_OK', 'TRUE');  %防止报错

% 将此更改为你的脚本目录
py_path = 'D:\TeXstudio\A_what_I_Done\SLAM\IROS\IROS_V_2\参考代码\my_code\实验_2024.10.19\做重投影图_2024.10.31'; 


d = [652.1829029	656.8106925	661.4998092	625.9814353	428.4215788	498.138267
682.1758486	686.0576446	634.2334171	568.3671964	490.8655819	564.0834281
711.9732248	713.5771401	611.9779144	515.6379296	560.8681629	633.8641147
654.421965	654.9773802	545.1366312	469.7916442	577.6792038	651.468668
593.183968	596.9799398	477.2758335	432.7496478	604.6644104	678.2535603
575.0504427	575.9933962	602.526204	570.9471195	432.1381043	513.5501447
603.697793	603.9601181	565.4784207	513.3569981	497.6070906	587.5612443
507.8865294	508.2741971	539.4880262	542.7487405	464.4041535	545.452375
538.9669347	537.8233659	496.3654682	478.7453976	524.7123815	615.6871026
586.9279572	589.3261533	602.526204	585.4617394	453.5282997	526.517922
616.574613	619.8520188	565.4784207	526.8971099	513.9362203	587.5612443
524.504963	527.9678224	539.4880262	556.105771	483.8964228	558.0146178
557.2217076	554.4313812	505.0254499	493.2840397	539.3680155	615.6871026
]';


d =d/1000;
 
theta =[-5.696084204	-3.476988073	3.020343324	11.93560254	-1.776044253	-4.969740728
0.418209941	2.357091289	8.426969021	16.10627412	-4.071813894	-7.438676242
5.484654097	7.753161875	15.09157268	21.52743295	-6.115503566	-9.191014044
8.4589124	10.51262717	13.02076731	14.52719786	0.9877604	-2.920721521
12.44186426	13.99760842	10.21573957	6.607133302	7.729750162	3.226494244
-2.978020652	-0.82553047	-0.6313329	6.174650628	7.725521825	2.964124547
3.625464683	5.835932351	5.727326717	10.61965528	4.208159848	-0.323701525
0.187240443	2.245742566	-4.587978124	-0.350428533	15.97352343	9.634295017
7.431407971	9.055357508	1.149593794	3.17983012	11.30993247	5.879652082
-3.404518299	-1.129629545	-0.6313329	5.694430718	7.359087682	2.167890279
2.93172392	5.069420133	5.727326717	9.610679669	3.703299029	-0.323701525
-0.543930933	1.441099292	-4.587978124	-1.026083358	14.70355182	9.07008339
5.812482383	7.74199124	2.448656584	2.121096397	10.99873335	5.879652082
]'/180*pi;

p_w = [0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0
73.5	0	0	73.5	0	0	73.5	0	0	73.5	0	0	73.5	0	0	73.5	0	0
147	0	0	147	0	0	147	0	0	147	0	0	147	0	0	147	0	0
147	-73.5	0	147	-73.5	0	147	-73.5	0	147	-73.5	0	147	-73.5	0	147	-73.5	0
147	-147	0	147	-147	0	147	-147	0	147	-147	0	147	-147	0	147	-147	0
0	-73.5	73.5	0	-73.5	73.5	0	-73.5	73.5	0	-73.5	73.5	0	-73.5	73.5	0	-73.5	73.5
73.5	-73.5	73.5	73.5	-73.5	73.5	73.5	-73.5	73.5	73.5	-73.5	73.5	73.5	-73.5	73.5	73.5	-73.5	73.5
0	-147	73.5	0	-147	73.5	0	-147	73.5	0	-147	73.5	0	-147	73.5	0	-147	73.5
73.5	-147	73.5	73.5	-147	73.5	73.5	-147	73.5	73.5	-147	73.5	73.5	-147	73.5	73.5	-147	73.5
0	-73.5	0	0	-73.5	0	0	-73.5	0	0	-73.5	0	0	-73.5	0	0	-73.5	0
73.5	-73.5	0	73.5	-73.5	0	73.5	-73.5	0	73.5	-73.5	0	73.5	-73.5	0	73.5	-73.5	0
0	-147	0	0	-147	0	0	-147	0	0	-147	0	0	-147	0	0	-147	0
73.5	-147	0	73.5	-147	0	73.5	-147	0	73.5	-147	0	73.5	-147	0	73.5	-147	0
]';


p_w = p_w/1000;

% k=6
% d = d(k,:);
% theta = theta(k,:);
% p_w = p_w(3*k-2:3*k,:);

theta = -theta;
tan_theta = tan(theta);
cos_theta = cos(theta);
sin_theta = sin(theta);

num = size(d,1);
for i=1:num
    p_si(2*i-1,:) = d(i,:).* cos_theta(i,:);
    p_si(2*i,:) = d(i,:).* sin_theta(i,:);
end



% p_si = [d.*cos_theta;d.*sin_theta];









var = 0;

phi_max = 6*pi/180;

num_group = size(p_w,1)/3;

num_points = size(p_w,2);


M=100;

BESTAnP_GN_time = 0;
Nonapp_time = 0;
App_time = 0;
Combine_CIO_time= 0;

BESTAnP_CIO_time = 0;
Initial_CEs_time = 0;

for j = 1:M
    for i = 1:1
        tic;
        [R_GN,t_GN] = ToCAnP_1GN_stdVar(p_w(3*i-2:3*i,:),p_si(2*i-1:2*i,:),0.0034,0.0033);
        BESTAnP_GN_time = BESTAnP_GN_time + toc;
    end
end
BESTAnP_GN_time = BESTAnP_GN_time/100*1000;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for j = 1:M
    for i = 1:1
        tic;
        [R_CEs,t_CEs] = Initial_CEs(p_w(3*i-2:3*i,:),p_si(2*i-1:2*i,:),0.0034,0.0033);
        Initial_CEs_time = Initial_CEs_time + toc;
    end
end
Initial_CEs_time = Initial_CEs_time/100*1000;


for j = 1:M
    for i = 1:1
        tic;
        [R_CIO,t_CIO] = BESTAnP_CIO(p_w(3*i-2:3*i,:),p_si(2*i-1:2*i,:),0.0034,0.0033,phi_max);
        BESTAnP_CIO_time = BESTAnP_CIO_time + toc;
    end
end
BESTAnP_CIO_time = BESTAnP_CIO_time/100*1000;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


for j = 1:M
    for i = 1:1
        tic;
        [R_Nonapp,t_Nonapp] = Nonapp_Algorithm_2(p_w(3*i-2:3*i,:),p_si(2*i-1:2*i,:),phi_max,R_GN,py_path);
        Nonapp_time = Nonapp_time+toc;
    end
end
Nonapp_time = BESTAnP_GN_time/100*1000;


for j = 1:M
    for i = 1:1
        tic;
        [R_app,t_app] = App_Algorithm_2(p_w(3*i-2:3*i,:),p_si(2*i-1:2*i,:),phi_max,py_path);
        App_time = App_time+toc;
    end   
end    
App_time = App_time/100*1000;


for j = 1:M
    for i = 1:1
            tic;
         [R_Combine_CIO,t_Combine_CIO] = Combine_CIO_2(p_w(3*i-2:3*i,:),p_si(2*i-1:2*i,:),phi_max,R_GN,py_path);
         Combine_CIO_time =Combine_CIO_time+toc;
    end
end
Combine_CIO_time = Combine_CIO_time/100*1000;

tic;
for j = 1:M
    for i = 1:1   
        [R_2GN,t_2GN] = ToCAnP_2GN_stdVar(p_w(3*i-2:3*i,:),p_si(2*i-1:2*i,:),0.0034,0.0033);
    end
end
ToCAnP_2GN_time = toc/100*1000;

tic;
for j = 1:M
    for i = 1:1   
        [R_3GN,t_3GN] = ToCAnP_3GN_stdVar(p_w(3*i-2:3*i,:),p_si(2*i-1:2*i,:),0.0034,0.0033);
    end
end
ToCAnP_3GN_time = toc/100*1000;

for i = 1:num_group

    [R_GN,t_GN] = ToCAnP_1GN_stdVar(p_w(3*i-2:3*i,:),p_si(2*i-1:2*i,:),0.0034,0.0033);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    p_s_est = R_GN'*(p_w(3*i-2:3*i,:) - t_GN);
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

    [R_CEs,t_CEs] = Initial_CEs(p_w(3*i-2:3*i,:),p_si(2*i-1:2*i,:),0.0034,0.0033);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    p_s_est_CEs = R_CEs'*(p_w(3*i-2:3*i,:) - t_CEs);
    d_est_CEs = vecnorm(p_s_est_CEs);
    tan_theta_CEs = p_s_est_CEs(2,:)./p_s_est_CEs(1,:);
    theta_est_CEs = atan(tan_theta_CEs);
    p_si_est_CEs(1,:) = d_est_CEs.*cos(theta_est_CEs);
    p_si_est_CEs(2,:) = d_est_CEs.*sin(theta_est_CEs);

    error_CEs(i,:) = norm(p_si_est_CEs-p_si(2*i-1:2*i,:))/num_points;
    error_CEs_d(i,:) = norm(d_est_CEs--d(i,:))/num_points;
    error_CEs_theta(i,:) = norm(theta_est_CEs - theta(i,:))/num_points;
    
    
   
    d_est_CEs = d_est_CEs';
    theta_est_CEs = theta_est_CEs';
    
end


for i = 1:num_group

    [R_CIO,t_CIO] = BESTAnP_CIO(p_w(3*i-2:3*i,:),p_si(2*i-1:2*i,:),0.0034,0.0033,phi_max);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    p_s_est_CIO = R_CIO'*(p_w(3*i-2:3*i,:) - t_CIO);
    d_est_CIO = vecnorm(p_s_est_CIO);
    tan_theta_CIO = p_s_est_CIO(2,:)./p_s_est_CIO(1,:);
    theta_est_CIO = atan(tan_theta_CIO);
    p_si_est_CIO(1,:) = d_est_CIO.*cos(theta_est_CIO);
    p_si_est_CIO(2,:) = d_est_CIO.*sin(theta_est_CIO);

    error_CIO(i,:) = norm(p_si_est_CIO-p_si(2*i-1:2*i,:))/num_points;
    error_CIO_d(i,:) = norm(d_est_CIO--d(i,:))/num_points;
    error_CIO_theta(i,:) = norm(theta_est_CIO - theta(i,:))/num_points;
    
    
   
    d_est_CIO = d_est_CIO';
    theta_est_CIO = theta_est_CIO';
    
end






%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for i = 1:num_group
    [R_Nonapp,t_Nonapp] = Nonapp_Algorithm_2(p_w(3*i-2:3*i,:),p_si(2*i-1:2*i,:),phi_max,R_GN,py_path);
    
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
    [R_app,t_app] = App_Algorithm_2(p_w(3*i-2:3*i,:),p_si(2*i-1:2*i,:),phi_max,py_path);

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
    [R_Combine_CIO,t_Combine_CIO] = Combine_CIO_2(p_w(3*i-2:3*i,:),p_si(2*i-1:2*i,:),phi_max,R_GN,py_path);

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



for i = 1:num_group    

    [R_2GN,t_2GN] = ToCAnP_2GN_stdVar(p_w(3*i-2:3*i,:),p_si(2*i-1:2*i,:),0.0034,0.0033);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    p_s_est_2GN = R_2GN'*(p_w(3*i-2:3*i,:) - t_2GN);
    d_est_2GN = vecnorm(p_s_est_2GN);
    tan_theta_2GN = p_s_est_2GN(2,:)./p_s_est_2GN(1,:);
    theta_est_2GN = atan(tan_theta_2GN);
    p_si_est_2GN(1,:) = d_est_2GN.*cos(theta_est_2GN);
    p_si_est_2GN(2,:) = d_est_2GN.*sin(theta_est_2GN);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   
    error_2GN(i,:) = norm(p_si_est_2GN-p_si(2*i-1:2*i,:))/num_points;
    error_d_2GN(i,:) = norm(d_est_2GN--d(i,:))/num_points;
    error_theta_2GN(i,:) = norm(theta_est_2GN - theta(i,:))/num_points;

    d_est_2GN = d_est_2GN';
    theta_est_2GN =theta_est_2GN';
end
 
for i = 1:num_group
    [R_3GN,t_3GN] = ToCAnP_3GN_stdVar(p_w(3*i-2:3*i,:),p_si(2*i-1:2*i,:),0.0034,0.0033);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    p_s_est_3GN = R_3GN'*(p_w(3*i-2:3*i,:) - t_3GN);
    d_est_3GN = vecnorm(p_s_est_3GN);
    tan_theta_3GN = p_s_est_3GN(2,:)./p_s_est_3GN(1,:);
    theta_est_3GN = atan(tan_theta_3GN);
    p_si_est_3GN(1,:) = d_est_3GN.*cos(theta_est_3GN);
    p_si_est_3GN(2,:) = d_est_3GN.*sin(theta_est_3GN);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    error_3GN(i,:) = norm(p_si_est_3GN-p_si(2*i-1:2*i,:))/num_points;
    error_d_3GN(i,:) = norm(d_est_3GN--d(i,:))/num_points;
    error_theta_3GN(i,:) = norm(theta_est_3GN - theta(i,:))/num_points;

    d_est_3GN = d_est_3GN';
    theta_est_3GN = theta_est_3GN';
end


   

error_d = error_d';

error_CEs_d = error_CEs_d';
error_CIO_d = error_CIO_d';

error_Nonapp_d = error_Nonapp_d';
error_app_d = error_app_d';
error_Combine_CIO_d = error_Combine_CIO_d';

error_theta = error_theta';

error_CEs_theta = error_CEs_theta';
error_CIO_theta = error_CIO_theta';

error_Nonapp_theta = error_Nonapp_theta';
error_app_theta = error_app_theta';
error_Combine_CIO_theta = error_Combine_CIO_theta';



error = error';

error_CEs = error_CEs';
error_CIO = error_CIO';

error_Nonapp  = error_Nonapp';
error_app = error_app';
error_Combine_CIO = error_Combine_CIO';

error_2GN=error_2GN';
error_d_2GN = error_d_2GN';
error_theta_2GN = error_theta_2GN';

error_3GN=error_3GN';
error_d_3GN = error_d_3GN';
error_theta_3GN = error_theta_3GN';


