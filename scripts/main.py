#!/usr/bin/python3

import os
import sys
import csv
import yaml
import numpy as np
import matplotlib.pyplot as plt
from roslib.packages import get_pkg_dir
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm

# Append the scripts directory to sys.path
BESTAnP_dir = get_pkg_dir('BESTAnP')
scripts_dir = os.path.abspath(os.path.join(BESTAnP_dir, 'scripts'))
sys.path.append(BESTAnP_dir)
sys.path.append(scripts_dir)

from utils.sonar_data_processor import SonarDataReader
from utils.match_pairs import get_match_pairs
from anp.anp_alg import AnPAlgorithm
from tri.tri import ANRS, reconstrunction_error
from utils.pose2matrix import ros_pose_to_transform_matrix
from utils.coordinate_system_transform import (
    coordinate_transform_Pose,
    coordinate_transform_Pose_back,
    coordinate_transform_pt_back
)
from utils.transformation_matrix_add_noise import add_noise_to_pose
from record.analysis_trajectory import calculate_ATE, calculate_RPE


class AnPSonarSLAM:
    def __init__(self, data_path=None, method=None):
        self.config = self.read_config()
        # np.random.seed(self.config['seed'])
        self.method = method or self.config['ANP_METHOD']
        
        self.sonar_data_dir = os.path.join(str(BESTAnP_dir), data_path or self.config['DATA_PATH'])
        self.record_dir = os.path.join(BESTAnP_dir, "record", self.method)
        os.makedirs(self.record_dir, exist_ok=True)
        
        self.anp_algorithm = AnPAlgorithm(self.method)
        self.P_dict = {}
        
        # Trajectories
        self.real_t = []
        self.estimated_t = []
        self.real_poses = []
        self.estimated_poses = []

        self.first_index = 0
        self.second_index = 4
        self.step_size = 4
        
        self.T1 = None  # Will be initialized in initialize_transformations()
        self.theta_Rho1 = None
        self.pts_indice1 = None
        self.initialize_data()
        self.initialize_transformations()

    def read_config(self):
        """Read configurations from YAML files."""
        config = {}
        # Read odom.yaml
        with open(os.path.join(BESTAnP_dir, 'yaml/odom.yaml'), 'r') as file:
            odom_params = yaml.safe_load(file)
            config.update({
                'RECONSTRUCTION_ERROR_THRESHOLD': odom_params['RECONSTRUCTION_ERROR_THRESHOLD'],
                'DETERMINANT_THRESHOLD': odom_params['DETERMINANT_THRESHOLD'],
                'RECORD': odom_params['RECORD'],
                'rotation_noise_std': float(odom_params['INITIALIZE']['rotation_noise_std']),
                'translation_noise_std': float(odom_params['INITIALIZE']['translation_noise_std']),
                'Rho_noise_std': float(odom_params['SONAR_NOISE']['Rho_noise_std']),
                'theta_noise_std': float(odom_params['SONAR_NOISE']['theta_noise_std']),
            })
        # Read sim_env.yaml
        with open(os.path.join(BESTAnP_dir, 'yaml/sim_env.yaml'), 'r') as file:
            sim_env_params = yaml.safe_load(file)
            config['PHI_MAX'] = int(sim_env_params['sonar_attribute']['fov_vertical']) * np.pi / 180
        return config

    def initialize_data(self):
        """Read sonar data and set up recording if needed."""
        # Read sonar data
        reader = SonarDataReader(filepath=self.sonar_data_dir)
        reader.read_data()
        self.data = reader.get_data()
        # Record setup
        if self.config['RECORD']:
            try:
                file_number = max([
                    int(f[6:]) for f in os.listdir(self.record_dir)
                    if f.startswith('record') and f[6:].isdigit()
                ])
            except ValueError:
                file_number = 0
            self.record_folder = os.path.join(self.record_dir, f"record{file_number + 1}")
            os.makedirs(self.record_folder, exist_ok=True)
        else:
            self.record_folder = None

    def theta_Rho_add_noise(self, theta_Rho):
        """Add noise to theta and Rho measurements."""
        theta = theta_Rho[:, 0]
        Rho = theta_Rho[:, 1]
        theta_noise = np.random.normal(0, self.config['theta_noise_std'], size=theta.shape)
        new_theta = np.arctan(np.tan(theta) + theta_noise)
        Rho_noise = np.random.normal(0, self.config['Rho_noise_std'], size=Rho.shape)
        new_Rho = Rho + Rho_noise
        return np.vstack((new_theta, new_Rho)).T

    def initialize_transformations(self):
        """Initialize transformations with noise."""
        first_index = self.first_index
        second_index = self.second_index
        data = self.data
        # Initialize transformations with noise
        T0 = add_noise_to_pose(
            ros_pose_to_transform_matrix(data[first_index]['pose']),
            rotation_noise_std=self.config['rotation_noise_std'],
            translation_noise_std=self.config['translation_noise_std']
        )
        T1 = add_noise_to_pose(
            ros_pose_to_transform_matrix(data[second_index]['pose']),
            rotation_noise_std=self.config['rotation_noise_std'],
            translation_noise_std=self.config['translation_noise_std']
        )
        self.T0 = T0  # May not be needed later
        self.T1 = T1
        # Add noise to initial measurements
        theta_Rho0 = self.theta_Rho_add_noise(
            data[first_index]['si_q_theta_Rho']
        )
        theta_Rho1 = self.theta_Rho_add_noise(
            data[second_index]['si_q_theta_Rho']
        )
        self.theta_Rho0 = theta_Rho0
        self.theta_Rho1 = theta_Rho1
        self.pts_indice0 = data[first_index]['pts_indice']
        self.pts_indice1 = data[second_index]['pts_indice']

    def initial_point_estimation(self):
        """Initial point estimation using ANRS."""
        T0 = self.T0
        T1 = self.T1
        theta_Rho0 = self.theta_Rho0
        theta_Rho1 = self.theta_Rho1
        pts_indice0 = self.pts_indice0
        pts_indice1 = self.pts_indice1
        data = self.data
        config = self.config
        P_dict = self.P_dict
        
        # Get matching pairs and ground truth points
        theta_Rho, theta_Rho_prime, common_indices = get_match_pairs(
            theta_Rho0, pts_indice0, theta_Rho1, pts_indice1
        )
        # Need to get w_P_gt
        w_P_gt = data[self.first_index]['w_p']
        temp_indices = [np.where(pts_indice0 == idx)[0][0] for idx in common_indices]
        w_P_gt = w_P_gt[temp_indices]  # Matching w_P_gt
        
        # Transformation matrices
        T0_tri = coordinate_transform_Pose(T0)
        T1_tri = coordinate_transform_Pose(T1)
        T_matrix = np.linalg.inv(T1_tri) @ T0_tri

        difference_list, reconstruction_error_list = [], []
        
        # Initial point estimation using ANRS
        for i, idx in enumerate(common_indices):
            s_P, determinant = ANRS(T_matrix, theta_Rho[i], theta_Rho_prime[i])
            w_P = coordinate_transform_pt_back((T0_tri @ np.hstack([s_P, 1]))[:3])
            # Compute reconstruction error
            theta_i = theta_Rho[i][0]
            Rho_i = theta_Rho[i][1]
            theta_prime_i = theta_Rho_prime[i][0]
            Rho_prime_i = theta_Rho_prime[i][1]
            ps = np.array([-Rho_i * np.sin(theta_i), Rho_i * np.cos(theta_i)])
            ps_prime = np.array([-Rho_prime_i * np.sin(theta_prime_i), Rho_prime_i * np.cos(theta_prime_i)])
            recon_error = reconstrunction_error(s_P, ps, ps_prime, T_matrix)
            difference = np.linalg.norm(w_P_gt[i] - w_P)
    
            if recon_error < config['RECONSTRUCTION_ERROR_THRESHOLD'] and abs(determinant) > config['DETERMINANT_THRESHOLD']:
                P_dict[idx] = w_P
                reconstruction_error_list.append(recon_error)
                difference_list.append(difference)
        # Print statistics
        # print(f"Mean difference: {np.mean(difference_list)}, Variance: {np.var(difference_list)}")
        # print(f"Valid points: {len(reconstruction_error_list)}/{len(theta_Rho)}")
        # Update attributes
        self.T1 = T1
        self.theta_Rho1 = theta_Rho1
        self.pts_indice1 = pts_indice1

    def perform_time_step(self, timestep, entry):
        """Perform processing for a single timestep."""
        # Unpack necessary attributes and configs
        config = self.config
        T1 = self.T1
        theta_Rho1 = self.theta_Rho1
        pts_indice1 = self.pts_indice1
        P_dict = self.P_dict

        # Add noise to current measurements
        theta_Rho2 = self.theta_Rho_add_noise(
            entry['si_q_theta_Rho']
        )
        pts_indice2 = entry['pts_indice']

        # Find matching points in P_dict
        q_si2_list, P_w_list = [], []
        for idx, measurement in zip(pts_indice2, theta_Rho2):
            if idx in P_dict:
                theta, Rho = measurement
                q_si = [np.cos(theta) * Rho, np.sin(theta) * Rho]
                q_si2_list.append(q_si)
                P_w_list.append(P_dict[idx])

        q_si2 = np.array(q_si2_list).T  # Shape (2, N)
        P_w = np.array(P_w_list).T  # Shape (3, N)
        # print(f"ANP input size: {q_si2.shape[1]}")
        if q_si2.shape[1] < 3:
            print("Not enough points for ANP estimation.")
            return  # Skip this timestep since we cannot perform estimation

        # Perform ANP Algorithm
        R_SW_true = ros_pose_to_transform_matrix(entry['pose'])[:3, :3]
        t_S_true = ros_pose_to_transform_matrix(entry['pose'])[:3, 3].reshape(-1, 1)
        R_sw_cal, t_s_cal = self.anp_algorithm.compute_R_t(
            P_w, q_si2, phi_max=config['PHI_MAX'], R_true=R_SW_true
        )
        T2 = np.eye(4)
        T2[:3, :3], T2[:3, 3] = R_sw_cal, t_s_cal.flatten()
    
        # Update the point dictionary with new points using ANRS
        # Get matching pairs between frames
        theta_Rho, theta_Rho_prime, common_indices = get_match_pairs(
            theta_Rho1, pts_indice1, theta_Rho2, pts_indice2
        )
        T1_tri = coordinate_transform_Pose(T1)
        T2_tri = coordinate_transform_Pose(T2)
        T_matrix = np.linalg.inv(T2_tri) @ T1_tri

        for i, idx in enumerate(common_indices):
            if idx not in P_dict:
                s_P, determinant = ANRS(T_matrix, theta_Rho[i], theta_Rho_prime[i])
                w_P = coordinate_transform_pt_back((T1_tri @ np.hstack([s_P, 1]))[:3])
                # Compute reconstruction error
                theta_i = theta_Rho[i][0]
                Rho_i = theta_Rho[i][1]
                theta_prime_i = theta_Rho_prime[i][0]
                Rho_prime_i = theta_Rho_prime[i][1]
                ps = np.array([-Rho_i * np.sin(theta_i), Rho_i * np.cos(theta_i)])
                ps_prime = np.array([-Rho_prime_i * np.sin(theta_prime_i), Rho_prime_i * np.cos(theta_prime_i)])
                recon_error = reconstrunction_error(s_P, ps, ps_prime, T_matrix)
                if recon_error < config['RECONSTRUCTION_ERROR_THRESHOLD'] and abs(determinant) > config['DETERMINANT_THRESHOLD']:
                    P_dict[idx] = w_P  # Update the point dictionary

        # Update previous measurements
        self.theta_Rho1 = theta_Rho2
        self.pts_indice1 = pts_indice2
        self.T1 = T2

        # Record real and estimated poses
        T2_gt = ros_pose_to_transform_matrix(entry['pose'])
        self.real_poses.append(T2_gt)
        self.estimated_poses.append(T2)
        self.real_t.append(T2_gt[:3, 3])
        self.estimated_t.append(T2[:3, 3])

        # Visualization
        self.visualize(timestep, T2, T2_gt)

    def visualize(self, timestep, T2, T2_gt):
        """Perform visualization and recording if needed."""
    
        if self.config['RECORD'] and self.record_folder:
            
            real_poses_array = np.array(self.real_t)
            estimated_poses_array = np.array(self.estimated_t)
            
            fig = plt.figure(figsize=(10, 8))
            ax = fig.add_subplot(111, projection='3d')
            ax.plot(
                real_poses_array[:, 0], real_poses_array[:, 1], real_poses_array[:, 2],
                'b-', label='Real Traj'
            )
            ax.plot(
                estimated_poses_array[:, 0], estimated_poses_array[:, 1], estimated_poses_array[:, 2],
                'r--', label=self.method
            )
            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_zlabel('Z')
            ax.set_title('Trajectory')
            ax.legend()
            ax.grid(True)

            file_name = os.path.join(self.record_folder, f"time_{timestep}.png")
            plt.savefig(file_name)
            plt.close()  # Close the figure to free memory

            # Record data to CSV
            debug_file = os.path.join(self.record_folder, "atraj.csv")
            P_dict_values = np.array(list(self.P_dict.values()))
            with open(debug_file, 'a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(np.concatenate([T2.flatten(), T2_gt.flatten(), P_dict_values.flatten()]))
      
    def run(self):
        """Run the SLAM process."""
        self.initial_point_estimation()
        
        for timestep in range(self.second_index + self.step_size, len(self.data), self.step_size):
            entry = self.data[timestep]
            # print(f"\n=== Timestep: {timestep} ===")
            self.perform_time_step(timestep, entry)
    
    def get_result(self):
        return np.array(self.real_poses), np.array(self.estimated_poses)
        
if __name__ == "__main__":
    methods = ['BESTAnP', 'CombineCIO', 'BESTAnPCIO', 'Nonapp', 'App']
    trajectory_shape = ['square', 'circle', 'eight']
    
    test_seed_num = 0
    np.random.seed(test_seed_num)  
    for shape in trajectory_shape:
        print(shape)
        print("{:<10} {:<8}  {:<8}  {:<8}  {:<8}".format("method", "ATE_t", "ATE_R", "RPE_t", "RPE_R"))
        for method in methods:
            path = "data/{shape}/sonar_data.csv".format(shape=shape)
            anp_slam = AnPSonarSLAM(data_path=path, method=method)
            anp_slam.run()
            real_poses, estimated_poses = anp_slam.get_result()
            ATE_t, ATE_R = calculate_ATE(real_poses, estimated_poses)
            RTE_t, RTE_R = calculate_RPE(real_poses, estimated_poses)
            print("{:<10} {:<8.4f}  {:<8.2f}  {:<8.4f}  {:<8.2f}".format(method, ATE_t, ATE_R, RTE_t, RTE_R))

        
