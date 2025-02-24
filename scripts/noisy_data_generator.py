#!/usr/bin/python3

import os
import sys
import csv
import numpy as np
from roslib.packages import get_pkg_dir
from utils.sonar_logger import SonarDataReader
from utils.pose2matrix import ros_pose_to_transform_matrix
from utils.transformation_matrix_add_noise import add_noise_to_pose
from scipy.spatial.transform import Rotation

class NoiseDataGenerator:
    def __init__(self, data_path, output_dir):
        self.BESTAnP_dir = get_pkg_dir('BESTAnP')
        self.sonar_data_dir = os.path.join(str(self.BESTAnP_dir), data_path)
        self.output_dir = os.path.join(self.BESTAnP_dir, output_dir)
        os.makedirs(self.output_dir, exist_ok=True)
        
        # 读取原始数据
        self.reader = SonarDataReader(filepath=self.sonar_data_dir)
        self.reader.read_data()
        self.data = self.reader.get_data()

    def add_noise_to_measurements(self, theta_Rho, noise_level):
        """给声呐测量添加噪声"""
        noisy_theta_Rho = []
        for theta, rho in theta_Rho:
            # 给角度和距离添加高斯噪声
            noisy_theta = theta + np.random.normal(0, noise_level['theta'])
            noisy_rho = rho + np.random.normal(0, noise_level['rho'])
            noisy_theta_Rho.append([noisy_theta, noisy_rho])
        return np.array(noisy_theta_Rho)

    def generate_noisy_data(self, seed, noise_levels):
        """生成带噪声的数据"""
        np.random.seed(seed)
        
        # 为每个噪声级别创建一个输出文件
        output_file = os.path.join(self.output_dir, f'noisy_data_seed_{seed}.csv')
        
        with open(output_file, 'w', newline='') as f:
            writer = csv.writer(f)
            # 写入头部
            header = ['timestamp', 
                     'true_pose_x', 'true_pose_y', 'true_pose_z', 'true_pose_qx', 'true_pose_qy', 'true_pose_qz', 'true_pose_qw',
                     'noisy_pose_x', 'noisy_pose_y', 'noisy_pose_z', 'noisy_pose_qx', 'noisy_pose_qy', 'noisy_pose_qz', 'noisy_pose_qw',
                     'pts_indice', 'noisy_si_q_theta_Rho, w_p']
            writer.writerow(header)

            # 处理每个时间戳的数据
            for entry in self.data:
                # 获取真实位姿
                true_pose = entry['pose']
                true_T = ros_pose_to_transform_matrix(true_pose)
                
                # 添加位姿噪声
                noisy_T = add_noise_to_pose(true_T, 
                                          translation_noise_std=noise_levels['pose_position'],
                                          rotation_noise_std=noise_levels['pose_rotation'])
                
                # 给声呐测量添加噪声
                theta_Rho = entry['si_q_theta_Rho']
                noisy_theta_Rho = self.add_noise_to_measurements(theta_Rho, {
                    'theta': noise_levels['sonar_angle'],
                    'rho': noise_levels['sonar_range']
                })

                # 准备写入的数据
                row_data = [
                    entry['timestamp'],
                    # 真实位姿
                    true_pose['position']['x'], true_pose['position']['y'], true_pose['position']['z'],
                    true_pose['orientation']['x'], true_pose['orientation']['y'], true_pose['orientation']['z'], true_pose['orientation']['w'],
                    # 带噪声位姿
                    noisy_T[0,3], noisy_T[1,3], noisy_T[2,3],
                    *Rotation.from_matrix(noisy_T[:3,:3]).as_quat(),  # 将旋转矩阵转换为四元数[x,y,z,w]
                    # 点索引和带噪声的声呐测量
                    entry['pts_indice'].tolist(),
                    noisy_theta_Rho.tolist(),
                    entry['w_p'].tolist()
                ]
                writer.writerow(row_data)

def main():
    # 定义不同的随机种子
    seeds = [i for i in range(100)]
    # seeds = [0]
    
    # 定义噪声级别
    noise_levels = {
        'pose_position': 0.001,  # 位置噪声标准差 (米)
        'pose_rotation': 0.001,  # 旋转噪声标准差 (弧度)
        'sonar_angle': 0.001,   # 声呐角度噪声标准差 (弧度)
        'sonar_range': 0.001     # 声呐距离噪声标准差 (米)
    }
    
    # 处理不同形状的轨迹
    trajectories = ['square', 'circle', 'eight']
    
    for trajectory in trajectories:
        data_path = f"data/{trajectory}/sonar_data.csv"
        output_dir = f"data/{trajectory}/noisy_data"
        
        generator = NoiseDataGenerator(data_path, output_dir)
        
        for seed in seeds:
            print(f"Generating noisy data for {trajectory} trajectory with seed {seed}")
            generator.generate_noisy_data(seed, noise_levels)

if __name__ == "__main__":
    main()