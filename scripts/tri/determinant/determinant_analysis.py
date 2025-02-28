#!/usr/bin/python3

"""
This code is to analyze in which case the determiant of 
triangularation method objective function will decrease to 0 

It will subscribe the "/sim/sonar_data_with_pose" rostopic 
and calculate the determinant between the 1st and the last frame

So you need to run simulator first
"""
# Append the root dir
import sys, roslib, os
project_root = roslib.packages.get_pkg_dir('lias_anp')
root_dir = os.path.abspath(os.path.join(project_root, 'scripts'))
sys.path.append(root_dir)

import numpy as np
from lias_anp.msg import SonarData  # 确保替换为你的包名
import csv


# This is the function implement in this package
from utils.match_pairs import get_match_pairs
from anp.anp_alg import AnPAlgorithm
from utils.pose2matrix import pose_to_transform_matrix
from utils.coordinate_system_transform import coordinate_transform_Pose
from tri.tri import ANRS, GTRS
import rospy

import csv



class Determinant_Analysis:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('tri_sim', anonymous=True)
        
        # 定义订阅者
        self.sonar_data_sub = rospy.Subscriber('/sim/sonar_data_with_pose', SonarData, self.sonar_callback)
        # self.pose_sub = rospy.Subscriber('/sim/sonar_data_with_pose', PoseStamped, self.pose_callback)

        # 定义数据存储
        self.w_p = None
        self.s_p = None
        self.si_q = None
        self.timestep = None
        
        self.pts_indice = None
        self.si_q_theta_Rho = None
        
        self.pts_indice_prime = None
        self.si_q_theta_Rho_prime = None
        
        self.pose = None
        self.pose_T = None
        self.R = None
        self.t = None
        
        # 初始化标志
        self.initialized = False
    
    def initialize_first_frame(self, data):
        """ 初始化第一次回调内容 """
        # self.pose_T0 = pose_to_transform_matrix(data.pose)
        # self.pose_T0 = coordinate_transform_Pose(pose_to_transform_matrix(data.pose))
        self.pose_T0 = np.diag([1,1,1,1])
        
        # self.w_p_T0 = np.array(data.w_p).reshape(-1, 3)
        # self.s_p_T0 = np.array(data.s_p).reshape(-1, 3)
        # self.si_q_T0 = np.array(data.si_q).reshape(-1, 2)
        # self.timestep_T0 = data.timestep
        # self.si_q_theta_Rho_T0 = np.array(data.si_q_theta_Rho).reshape(-1, 2)
        self.si_q_theta_Rho_T0 = np.array([[0,2]])
        self.pts_indice_T0 = np.array(data.indices)
        self.initialized = True
        self.origin = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])

        
    def sonar_callback(self, data):
        if len(data.indices) > 0:
            if not self.initialized:
                self.initialize_first_frame(data)
                
            else:
                pose_T1 = coordinate_transform_Pose(pose_to_transform_matrix(data.pose))
                si_q_theta_Rho_T1 = np.array(data.si_q_theta_Rho).reshape(-1, 2)
                pts_indice_T1 = np.array(data.indices)
                theta_Rho, theta_Rho_prime, common_indices = get_match_pairs(self.si_q_theta_Rho_T0, self.pts_indice_T0, si_q_theta_Rho_T1, pts_indice_T1)

                # self.w_p = np.array(data.w_p).reshape(-1, 3)
                # self.s_p = np.array(data.s_p).reshape(-1, 3)
                # self.si_q = np.array(data.si_q).reshape(-1, 2)
                # self.timestep = data.timestep
                
                determinant_list = []                       
                T_matrix = np.linalg.inv(pose_T1) @ self.pose_T0
                
                present_position = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])
              
                for i in range(len(theta_Rho)):
                    # print(theta_Rho[i][0])
                    # # print(theta_Rho_prime[i][0])
                    # print(T_matrix, theta_Rho[i], theta_Rho_prime[i])
                    s_P_0, determinant0 = ANRS(T_matrix, theta_Rho[i], theta_Rho_prime[i])
                    s_P_1, determinant1 = GTRS(T_matrix, theta_Rho[i], theta_Rho_prime[i])                    
            
                    P0 = np.array([2,0,0])
                    
                    if s_P_0 is not None:
                        print("ANRS", np.sum(s_P_0-P0)**2)
                    if s_P_1 is not None:
                        print("GTRS", np.sum(s_P_1-P0)**2)
                    self.writer.writerow([np.sum(s_P_0-P0)**2, determinant0, np.sum(s_P_1-P0)**2, determinant1])
                print()
          
   
    def main_process(self):        
        # 打开一个文件用于写入数据
        with open(root_dir+'/tri/determiant_error_correlation.csv', mode='a', newline='') as file:
            self.writer = csv.writer(file)
            # 写入表头
            # self.writer.writerow(['s_P_0', 'determinant0', 's_P_1', 'determinant1'])

            rate = rospy.Rate(30)
            while not rospy.is_shutdown():
                rate.sleep()

# 确保你的脚本被执行时调用main_process方法
if __name__ == '__main__':
    da = Determinant_Analysis()
    da.main_process()
