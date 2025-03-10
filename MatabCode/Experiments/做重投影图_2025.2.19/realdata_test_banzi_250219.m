clc;
clear;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 测试板子的数据 11.05 记录时间 以及多次GN
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% 设置环境变量，允许 OpenMP 运行时重复加载
setenv('KMP_DUPLICATE_LIB_OK', 'TRUE');  %防止报错

% 将此更改为你的脚本目录
py_path = 'D:\TeXstudio\A_what_I_Done\SLAM\IROS\IROS_V_2\参考代码\my_code\实验_2024.10.19\做重投影图_2024.10.31'; 



% d = [721.5966869	717.0431836	1156.060384	1159.633973	1163.365903	661.2582278	1195.725933	888.3901732	696.512303,
% 735.2996819	732.7476579	1161.019281	1162.163555	1166.6763	770.7627202	1093.877932	815.2486921	674.5208612,
% 772.274863	768.4987849	1202.742457	1206.932336	1209.671602	646.1123214	1284.439281	976.3301196	760.8288617,
% 781.5171772	773.9584776	1217.094428	1214.540896	1219.935078	693.682616	1253.535143	950.5172627	751.9862876,
% 801.4415139	800.4974263	1220.791738	1219.207743	1217.332062	871.1274116	1086.594524	828.3504659	717.6518587,
% 841.7278108	834.7508246	1265.41078	1269.502329	1270.658393	689.4682523	1351.865946	1043.829026	825.5889292,
% 796.7911403	797.4494906	1221.669219	1220.342602	1225.29652	832.0386745	1132.647234	858.8742545	723.6533557,
% 866.8366601	869.2406325	1275.208444	1281.715806	1284.490754	933.4900799	1128.389039	882.926713	772.5263019,
% 862.5038037	859.8535641	1295.305694	1295.577888	1303.542936	784.212606	1309.766533	1014.079361	824.1679244,
% 872.2793114	871.5273518	1301.635347	1302.033168	1301.470257	879.6696156	1220.64731	950.5172627	809.0498951,
% 932.4344836	921.5370624	1361.912453	1366.901467	1366.95185	790.6863019	1426.888845	1124.651271	915.1055068,
% 905.9078682	901.1515812	1338.674357	1340.406964	1342.526231	840.8453393	1333.590076	1043.888404	863.9845145,
% 911.5191919	906.9138469	1339.872511	1340.67822	1348.569933	899.3982443	1279.731554	1002.798721	853.0684394,
% 958.556159	957.9582059	1370.918187	1374.218733	1375.502087	1007.101042	1228.496099	982.5347643	870.2573423,
% ];
% 
% d =d/1000;
%  
% theta =[16.43416321	-0.530501317	5.767888898	17.31978024	22.83365418	7.499609819	7.657709694	19.9922519	18.90988412,
% 0.517329323	-16.17714483	-3.770487335	7.632494482	13.32453126	-5.808807362	-0.782443034	7.486727323	2.538544598,
% 24.9127465	8.445103466	11.13936367	23.07443951	28.25283625	18.10914368	11.25185264	25.25814983	26.84454057,
% 18.31951087	2.089262885	6.892423122	18.73207207	24.08912677	11.87453708	7.379077099	20.33356119	20.27482304,
% -7.257474889	-23.58856393	-9.231808469	2.262552914	7.599805616	-10.53918373	-7.548376473	-1.033278781	-7.174324236,
% 26.46400052	10.19276168	12.26796658	24.31996497	29.24360243	22.05753554	10.82707555	25.4241765	27.80145878,
% -0.954841254	-17.19040416	-5.222618711	6.168081181	12.04055106	-5.494512421	-3.696791504	4.322277059	-0.131412108,
% -8.145617052	-24.11800253	-10.27149619	1.409844677	6.45465234	-9.93126287	-10.252081	-3.448682169	-8.774239251,
% 15.400171	-0.884806561	5.36724628	17.36716085	22.61425322	12.09475708	4.214940729	15.76505906	16.13558866,
% 2.617413363	-13.77159972	-3.216303592	8.429644894	13.64636172	-0.108105116	-3.586053761	20.33356119	2.46913026,
% 24.05568644	7.867413407	11.17299254	23.01588775	28.02747205	22.8511562	8.158377946	21.83281303	24.42675401,
% 12.91593546	-3.378854299	3.554178236	15.29245305	20.63065985	10.57797047	1.997051188	13.32919685	13.10186579,
% 5.118865774	-10.65186201	-1.490631835	10.19699273	15.49013893	3.279536054	-2.973731082	6.748617546	5.022854953,
% -5.365085216	-21.22994544	-8.914926957	2.769100963	7.836781813	-5.48509435	-10.82412935	-3.292574668	-6.460865071,
% ]/180*pi;







d = [721.5966869	717.0431836	1156.060384	1159.633973	1163.365903	661.2582278	1195.725933	888.3901732	696.512303,
735.2996819	732.7476579	1161.019281	1162.163555	1166.6763	770.7627202	1093.877932	815.2486921	674.5208612,
772.274863	768.4987849	1202.742457	1206.932336	1209.671602	646.1123214	1284.439281	976.3301196	760.8288617,
781.5171772	773.9584776	1217.094428	1214.540896	1219.935078	693.682616	1253.535143	950.5172627	751.9862876,
801.4415139	800.4974263	1220.791738	1219.207743	1217.332062	871.1274116	1086.594524	828.3504659	717.6518587,
841.7278108	834.7508246	1265.41078	1269.502329	1270.658393	689.4682523	1351.865946	1043.829026	825.5889292,
796.7911403	797.4494906	1221.669219	1220.342602	1225.29652	832.0386745	1132.647234	858.8742545	723.6533557,
866.8366601	869.2406325	1275.208444	1281.715806	1284.490754	933.4900799	1128.389039	882.926713	772.5263019,
862.5038037	859.8535641	1295.305694	1295.577888	1303.542936	784.212606	1309.766533	1014.079361	824.1679244,
872.2793114	871.5273518	1301.635347	1302.033168	1301.470257	879.6696156	1220.64731	950.5172627	809.0498951,
932.4344836	921.5370624	1361.912453	1366.901467	1366.95185	790.6863019	1426.888845	1124.651271	915.1055068,
905.9078682	901.1515812	1338.674357	1340.406964	1342.526231	840.8453393	1333.590076	1043.888404	863.9845145,
911.5191919	906.9138469	1339.872511	1340.67822	1348.569933	899.3982443	1279.731554	1002.798721	853.0684394,
958.556159	957.9582059	1370.918187	1374.218733	1375.502087	1007.101042	1228.496099	982.5347643	870.2573423,
]';

d =d/1000;
 
theta =[16.43416321	-0.530501317	5.767888898	17.31978024	22.83365418	7.499609819	7.657709694	19.9922519	18.90988412,
0.517329323	-16.17714483	-3.770487335	7.632494482	13.32453126	-5.808807362	-0.782443034	7.486727323	2.538544598,
24.9127465	8.445103466	11.13936367	23.07443951	28.25283625	18.10914368	11.25185264	25.25814983	26.84454057,
18.31951087	2.089262885	6.892423122	18.73207207	24.08912677	11.87453708	7.379077099	20.33356119	20.27482304,
-7.257474889	-23.58856393	-9.231808469	2.262552914	7.599805616	-10.53918373	-7.548376473	-1.033278781	-7.174324236,
26.46400052	10.19276168	12.26796658	24.31996497	29.24360243	22.05753554	10.82707555	25.4241765	27.80145878,
-0.954841254	-17.19040416	-5.222618711	6.168081181	12.04055106	-5.494512421	-3.696791504	4.322277059	-0.131412108,
-8.145617052	-24.11800253	-10.27149619	1.409844677	6.45465234	-9.93126287	-10.252081	-3.448682169	-8.774239251,
15.400171	-0.884806561	5.36724628	17.36716085	22.61425322	12.09475708	4.214940729	15.76505906	16.13558866,
2.617413363	-13.77159972	-3.216303592	8.429644894	13.64636172	-0.108105116	-3.586053761	20.33356119	2.46913026,
24.05568644	7.867413407	11.17299254	23.01588775	28.02747205	22.8511562	8.158377946	21.83281303	24.42675401,
12.91593546	-3.378854299	3.554178236	15.29245305	20.63065985	10.57797047	1.997051188	13.32919685	13.10186579,
5.118865774	-10.65186201	-1.490631835	10.19699273	15.49013893	3.279536054	-2.973731082	6.748617546	5.022854953,
-5.365085216	-21.22994544	-8.914926957	2.769100963	7.836781813	-5.48509435	-10.82412935	-3.292574668	-6.460865071,
]'/180*pi;

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

p_w = [-104.5066676	81.01085799	223.9279401	-104.5066676	81.01085799	223.9279401	-104.5066676	81.01085799	223.9279401	-104.5066676	81.01085799	223.9279401	-104.5066676	81.01085799	223.9279401	-104.5066676	81.01085799	223.9279401	-104.5066676	81.01085799	223.9279401	-104.5066676	81.01085799	223.9279401	-104.5066676	81.01085799	223.9279401,
104.5066676	81.01085799	223.9279401	104.5066676	81.01085799	223.9279401	104.5066676	81.01085799	223.9279401	104.5066676	81.01085799	223.9279401	104.5066676	81.01085799	223.9279401	104.5066676	81.01085799	223.9279401	104.5066676	81.01085799	223.9279401	104.5066676	81.01085799	223.9279401	104.5066676	81.01085799	223.9279401,
-221.3038723	3.309550779	206.0137049	-221.3038723	3.309550779	206.0137049	-221.3038723	3.309550779	206.0137049	-221.3038723	3.309550779	206.0137049	-221.3038723	3.309550779	206.0137049	-221.3038723	3.309550779	206.0137049	-221.3038723	3.309550779	206.0137049	-221.3038723	3.309550779	206.0137049	-221.3038723	3.309550779	206.0137049,
-136.6920583	36.75758996	179.142352	-136.6920583	36.75758996	179.142352	-136.6920583	36.75758996	179.142352	-136.6920583	36.75758996	179.142352	-136.6920583	36.75758996	179.142352	-136.6920583	36.75758996	179.142352	-136.6920583	36.75758996	179.142352	-136.6920583	36.75758996	179.142352	-136.6920583	36.75758996	179.142352,
221.3038723	3.309550779	206.0137049	221.3038723	3.309550779	206.0137049	221.3038723	3.309550779	206.0137049	221.3038723	3.309550779	206.0137049	221.3038723	3.309550779	206.0137049	221.3038723	3.309550779	206.0137049	221.3038723	3.309550779	206.0137049	221.3038723	3.309550779	206.0137049	221.3038723	3.309550779	206.0137049,
-253.489263	-40.94371725	161.2281168	-253.489263	-40.94371725	161.2281168	-253.489263	-40.94371725	161.2281168	-253.489263	-40.94371725	161.2281168	-253.489263	-40.94371725	161.2281168	-253.489263	-40.94371725	161.2281168	-253.489263	-40.94371725	161.2281168	-253.489263	-40.94371725	161.2281168	-253.489263	-40.94371725	161.2281168,
136.6920583	36.75758996	179.142352	136.6920583	36.75758996	179.142352	136.6920583	36.75758996	179.142352	136.6920583	36.75758996	179.142352	136.6920583	36.75758996	179.142352	136.6920583	36.75758996	179.142352	136.6920583	36.75758996	179.142352	136.6920583	36.75758996	179.142352	136.6920583	36.75758996	179.142352,
253.489263	-40.94371725	161.2281168	253.489263	-40.94371725	161.2281168	253.489263	-40.94371725	161.2281168	253.489263	-40.94371725	161.2281168	253.489263	-40.94371725	161.2281168	253.489263	-40.94371725	161.2281168	253.489263	-40.94371725	161.2281168	253.489263	-40.94371725	161.2281168	253.489263	-40.94371725	161.2281168,
-99.36296716	12.10170751	98.52829363	-99.36296716	12.10170751	98.52829363	-99.36296716	12.10170751	98.52829363	-99.36296716	12.10170751	98.52829363	-99.36296716	12.10170751	98.52829363	-99.36296716	12.10170751	98.52829363	-99.36296716	12.10170751	98.52829363	-99.36296716	12.10170751	98.52829363	-99.36296716	12.10170751	98.52829363,
99.36296716	12.10170751	98.52829363	99.36296716	12.10170751	98.52829363	99.36296716	12.10170751	98.52829363	99.36296716	12.10170751	98.52829363	99.36296716	12.10170751	98.52829363	99.36296716	12.10170751	98.52829363	99.36296716	12.10170751	98.52829363	99.36296716	12.10170751	98.52829363	99.36296716	12.10170751	98.52829363,
-239.9177581	-84.45025331	71.65694082	-239.9177581	-84.45025331	71.65694082	-239.9177581	-84.45025331	71.65694082	-239.9177581	-84.45025331	71.65694082	-239.9177581	-84.45025331	71.65694082	-239.9177581	-84.45025331	71.65694082	-239.9177581	-84.45025331	71.65694082	-239.9177581	-84.45025331	71.65694082	-239.9177581	-84.45025331	71.65694082,
-62.26632556	7.848439479	53.74270561	-62.26632556	7.848439479	53.74270561	-62.26632556	7.848439479	53.74270561	-62.26632556	7.848439479	53.74270561	-62.26632556	7.848439479	53.74270561	-62.26632556	7.848439479	53.74270561	-62.26632556	7.848439479	53.74270561	-62.26632556	7.848439479	53.74270561	-62.26632556	7.848439479	53.74270561,
62.26632556	7.848439479	53.74270561	62.26632556	7.848439479	53.74270561	62.26632556	7.848439479	53.74270561	62.26632556	7.848439479	53.74270561	62.26632556	7.848439479	53.74270561	62.26632556	7.848439479	53.74270561	62.26632556	7.848439479	53.74270561	62.26632556	7.848439479	53.74270561	62.26632556	7.848439479	53.74270561,
239.9177581	-84.45025331	71.65694082	239.9177581	-84.45025331	71.65694082	239.9177581	-84.45025331	71.65694082	239.9177581	-84.45025331	71.65694082	239.9177581	-84.45025331	71.65694082	239.9177581	-84.45025331	71.65694082	239.9177581	-84.45025331	71.65694082	239.9177581	-84.45025331	71.65694082	239.9177581	-84.45025331	71.65694082,

]';

p_w = p_w/1000;

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


