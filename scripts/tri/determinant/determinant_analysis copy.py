#!/usr/bin/python3
from sonardatareader import SonarDataReader
import numpy as np
import transforms3d
from scripts.anp.anp_alg import AnPAlgorithm
from tri.tri import ANRS, GTRS, gradient_descent

from determinant import compute_D

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import sys
import csv
import os
import copy

DEBUG = True

T_z_90 = np.array([[0,-1,0,0],[1,0,0,0],[0,0,1,0],[ 0,0,0,1]])
T_z_min90 = T_z_90.T
R_z_90 = T_z_90[:3, :3]

def quaternion_to_rotation_matrix(quaternion):
    """将四元数转换为旋转矩阵"""
    return transforms3d.quaternions.quat2mat(quaternion)

def pose_to_transform_matrix(pose):
    """将位姿转换为齐次变换矩阵"""
    position = pose['position']
    orientation = pose['orientation']
    # 提取平移向量
    translation = np.array([position['x'], position['y'], position['z']])
    # 提取四元数并转换为旋转矩阵
    quaternion = [orientation['w'], orientation['x'], orientation['y'], orientation['z']]
    rotation_matrix = quaternion_to_rotation_matrix(quaternion)
    
    # 构建齐次变换矩阵
    transform_matrix = np.eye(4)
    transform_matrix[:3, :3] = rotation_matrix
    transform_matrix[:3, 3] = translation
    
    return transform_matrix

def get_match_pairs(si_q_theta_Rho, pts_indice, si_q_theta_Rho_prime, pts_indice_prime):
    
    # 找到共同的索引
    common_indices = np.intersect1d(pts_indice, pts_indice_prime)

    # 获取t0时刻的匹配坐标
    t0_indices = [np.where(pts_indice == idx)[0][0] for idx in common_indices]
    matched_t0 = si_q_theta_Rho[t0_indices]

    # 获取t1时刻的匹配坐标
    t1_indices = [np.where(pts_indice_prime == idx)[0][0] for idx in common_indices]
    matched_t1 = si_q_theta_Rho_prime[t1_indices]
    
    return matched_t0, matched_t1, common_indices

def coordinate_transform_T(T0, T1):
    # T1 = T0 @ T
    T_matrix = np.linalg.inv(T0) @ T1 
    # x-axis oriented switched to y-axis oriented
    T_matrix = T_z_90 @ T_matrix @ T_z_min90
    # get transforamtion matrix
    T_matrix = np.linalg.inv(T_matrix)
    return T_matrix

def coordinate_transform_Pose(Pose):
    return (T_z_90 @ Pose @ T_z_min90)

def coordinate_transform_pt(P):
    return (R_z_90 @ P)

def coordinate_transform(p0, p1, T0, T1):
    p0 = coordinate_transform_pt(p0)
    p1 = coordinate_transform_pt(p1)
    T_matrix = coordinate_transform_T(T0, T1)
    return p0, p1, T_matrix



if __name__ == "__main__":
    reader = SonarDataReader(filepath = "sonar_data_simple.csv")
    reader.read_data()
    data = reader.get_data()
    
    try:
        file_number = max([int(f[6:]) for f in os.listdir('.') if f.startswith('record') and f[6:].isdigit()])
    except:
        file_number = 1
    new_folder = f"record{file_number + 1}"
    os.makedirs(new_folder, exist_ok=True)

    anp_algorithm = AnPAlgorithm()
    
    # initialize
    init_T0 = pose_to_transform_matrix(data[0]['pose'])
    init_T0 = coordinate_transform_Pose(init_T0)
    T0 = init_T0
    init_T1 = pose_to_transform_matrix(data[1]['pose']) # This is what we need to initialize
    init_T1 = coordinate_transform_Pose(init_T1)
    T1 = init_T1
    Tri_T1 = init_T1
    Tri_T1_gt = init_T1
        
    T_matrix = np.linalg.inv(T1) @ T0
    
    start_index = 0
    theta_Rho0 = data[start_index]['si_q_theta_Rho']
    pts_indice0 = data[start_index]['pts_indice']
    theta_Rho1 = data[start_index+1]['si_q_theta_Rho']
    pts_indice1 = data[start_index+1]['pts_indice']
    Tri_theta_Rho1 = theta_Rho1
    Tri_pts_indice1 = pts_indice1
    
    # Dictionary to store estimated points in world coordinate system
    P_dict = {}
    theta_Rho, theta_Rho_prime, common_indices = get_match_pairs(theta_Rho0, pts_indice0, theta_Rho1, pts_indice1)
    for i in range(len(theta_Rho)):
        # determinant = compute_D(T_matrix, theta=theta_Rho[timestep][0], theta_prime=theta_Rho_prime[timestep][0])
        s_P = ANRS(T_matrix, theta_Rho[i], theta_Rho_prime[i])
        w_P = ( T0 @ np.hstack([s_P, 1]) )[:3]
        key = common_indices[i]
        P_dict[key] = w_P
    
    # 初始化空列表用于存储轨迹
    real_poses_x = []
    real_poses_y = []
    real_poses_z = []
    estimated_poses_x = []
    estimated_poses_y = []
    estimated_poses_z = []
    
    # General idea is we have T0 and T1, and we want to get T2
    for timestep, entry in enumerate(data[start_index+2:], start=start_index+2):
        sys.stdout.write(f'\rTimestep: {timestep}')
        sys.stdout.flush()
        # ANP
        ## Get q_si2 and P_w for ANP
        theta_Rho2 = entry['si_q_theta_Rho']
        q_si_x2 = np.sin(theta_Rho2.T[0]) * theta_Rho2.T[1]
        q_si_y2 = np.cos(theta_Rho2.T[0]) * theta_Rho2.T[1]
        q_si2 = np.vstack([q_si_x2, q_si_y2])
        pts_indice2 = entry['pts_indice']

        ##  Find matching pairs of q_si and and w_P in dictionary
        filtered_P_w_values = []
        filtered_q_si_index = []
        for j, idx in enumerate( pts_indice2 ):
            value = P_dict.get(idx)
            if value is not None:
                filtered_P_w_values.append(value[:3])
                filtered_q_si_index.append(j)
        q_si2 = q_si2.T[filtered_q_si_index].T
        P_w = np.array(filtered_P_w_values).T
        t_s_cal, R_sw_cal = anp_algorithm.compute_t_R(q_si2, P_w)
        T2 = np.eye(4)  # 创建一个 4x4 的单位矩阵
        T2[:3, :3] = R_sw_cal  # 将 R 赋值给 T 的左上 3x3 子矩阵
        T2[:3, 3] = t_s_cal.flatten()  # 将 t 赋值给 T 的前 3 行第 4 列
        T2 = np.linalg.inv(T2)
        calculated_T1 = copy.deepcopy(T1)
        calculated_T2 = copy.deepcopy(T2)
        T1 = T2
        
        # END ANP
        #####################################################
        
        # TRI
        T2_gt = coordinate_transform_Pose(pose_to_transform_matrix(entry['pose']))
        Tri_T2_gt = T2_gt
        Tri_T_gt = np.linalg.inv(Tri_T2_gt) @ Tri_T1_gt

        Tri_theta_Rho2 = theta_Rho2
        Tri_pts_indice2 = pts_indice2
        theta_Rho, theta_Rho_prime, common_indices = get_match_pairs(Tri_theta_Rho1, Tri_pts_indice1, Tri_theta_Rho2, Tri_pts_indice2)

        ## Determinant evaluation
        skip_reconstruction = False
        determinant_list = []
        for i in range(len(theta_Rho)):
            determinant = compute_D(Tri_T_gt, theta=theta_Rho[i][0], theta_prime=theta_Rho_prime[i][0])
            determinant_list.append(determinant)
        determinant_evaluation = sum(abs(x) for x in determinant_list) / len(determinant_list)
        
        if determinant_evaluation < 0.02:
            print("Determinant is tooooo small, skip")
            skip_reconstruction = True
        
        
        Tri_T2 = calculated_T2
        T_matrix = np.linalg.inv(Tri_T2) @ Tri_T1
        
        # if not skip_reconstruction:
        if True:
            # Points ground truth
            w_P_gt = entry['w_p']
            w_P_gt_indices = [np.where(pts_indice2 == idx)[0][0] for idx in common_indices]
            w_P_gt = w_P_gt[w_P_gt_indices] 
            w_P_gt = coordinate_transform_pt( w_P_gt.T ).T

            reconstrubtion_error_list = []
            for i in range(len(theta_Rho)):
                s_P_init = GTRS(T_matrix, theta_Rho[i], theta_Rho_prime[i])
                # s_P_init = ANRS(T_matrix, theta_Rho[i], theta_Rho_prime[i])
                s_P, good_reconstruct = gradient_descent(s_P_init, theta_Rho[i], theta_Rho_prime[i], T_matrix)
                if good_reconstruct:
                    w_P = ( Tri_T1 @ np.hstack([s_P, 1]) )[:3]
                    key = common_indices[i]
                    if key not in P_dict:
                        P_dict[key] = w_P
                    # P_dict[key] = w_P
                    
                    difference = np.linalg.norm( w_P - w_P_gt[i] )
                    reconstrubtion_error_list.append(difference)
                
            Tri_theta_Rho1 = Tri_theta_Rho2
            Tri_pts_indice1 = Tri_pts_indice2
            Tri_T1 = Tri_T2
            Tri_T1_gt = Tri_T2_gt
        # TRI END
        ####################################################333

        T2_gt = pose_to_transform_matrix(entry['pose'])
        T2_gt = coordinate_transform_Pose(T2_gt)

        # 提取x和y坐标（即矩阵的第1和第2列的第4个元素）
        real_poses_x.append(T2_gt[0, 3])
        real_poses_y.append(T2_gt[1, 3])
        real_poses_z.append(T2_gt[2, 3])
        estimated_poses_x.append(T2[0, 3])
        estimated_poses_y.append(T2[1, 3])
        estimated_poses_z.append(T2[2, 3])
    
        
        # 绘制三维轨迹
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')

        ax.plot(real_poses_x, real_poses_y, real_poses_z, 'b-', label='Real Traj')
        ax.plot(estimated_poses_x, estimated_poses_y, estimated_poses_z, 'r--', label='Estimated Traj')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title('Trajectory')
        ax.legend()
        ax.grid(True)

        pose_estimation_error = np.linalg.norm(np.array([real_poses_x, real_poses_y, real_poses_z]) - np.array([estimated_poses_x, estimated_poses_y, estimated_poses_z]))
        if not skip_reconstruction and len(reconstrubtion_error_list):
            reconstrubtion_error_evaluation = sum(abs(x) for x in reconstrubtion_error_list) / len(reconstrubtion_error_list)
        else:
            reconstrubtion_error_evaluation = 0
        if not DEBUG:
            file_name = f"record{file_number + 1}/time_{timestep}.png"
            plt.savefig(file_name)  # 你可以指定其他文件名和格式，如 'plot.jpg', 'plot.pdf', 等等  
            plt.close()  # 关闭图表窗口

            with open("debug_file.csv", 'a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([timestep, pose_estimation_error, determinant_evaluation, reconstrubtion_error_evaluation])
               
        else:
            print("\npose_estimation_error=",pose_estimation_error)
            print("determinant_evaluation=",determinant_evaluation)
            print("reconstrubtion_error_evaluation=",reconstrubtion_error_evaluation,"\n")
            plt.show(block=False)
            if timestep % 15 == 0:
                plt.pause(1)  # 暂停5秒
            else:
                plt.pause(1)
            plt.close()  # 关闭图表窗口