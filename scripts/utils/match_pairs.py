"""
In real case, we need two consecutive frames to do triangularation 
To reconstruct a 3D points, we need its corresponding projection in two sonar image 

For simplicity, we have index for each points, so we can easily get pairs
"""
import numpy as np
from functools import reduce

# def get_match_pairs(theta_Rho, pts_indice, theta_Rho_prime, pts_indice_prime):
    
#     # 找到共同的索引
#     common_indices = np.intersect1d(pts_indice, pts_indice_prime)

#     # 获取t0时刻的匹配坐标
#     matched_theta_Rho_t0 = theta_Rho[[np.where(pts_indice == idx)[0][0] for idx in common_indices]]

#     # 获取t1时刻的匹配坐标
#     matched_theta_Rho_t1 = theta_Rho_prime[[np.where(pts_indice_prime == idx)[0][0] for idx in common_indices]]
    
#     return matched_theta_Rho_t0, matched_theta_Rho_t1, common_indices

# def get_match_pairs(theta_Rhos_across_times, pts_indices_across_times):
    
#     def intersect_multiple_arrays(array_list):
#         array_lists = [list(arr) for arr in array_list]
#         return list(reduce(np.intersect1d, array_lists))
    
#     common_indices = intersect_multiple_arrays(pts_indices_across_times)
#     matched_theta_Rho_across_times = [] 
#     for pts_indicei, theta_Rhoi in zip(pts_indices_across_times, theta_Rhos_across_times):
#         matched_theta_Rho_i = theta_Rhoi[[np.where(pts_indicei == idx)[0][0] for idx in common_indices]]
#         matched_theta_Rho_across_times.append(matched_theta_Rho_i)
#     return matched_theta_Rho_across_times, common_indices


def get_match_pairs(*args):
    if len(args) == 4:
        # 原始的两个时间点的情况
        theta_Rho, pts_indice, theta_Rho_prime, pts_indice_prime = args
        
        # 找到共同的索引
        common_indices = np.intersect1d(pts_indice, pts_indice_prime)

        # 获取t0时刻的匹配坐标
        matched_theta_Rho_t0 = theta_Rho[[np.where(pts_indice == idx)[0][0] for idx in common_indices]]

        # 获取t1时刻的匹配坐标
        matched_theta_Rho_t1 = theta_Rho_prime[[np.where(pts_indice_prime == idx)[0][0] for idx in common_indices]]
        
        return matched_theta_Rho_t0, matched_theta_Rho_t1, common_indices
    
    elif len(args) == 2:
        # 多个时间点的情况
        theta_Rhos_across_times, pts_indices_across_times = args
        
        def intersect_multiple_arrays(array_list):
            array_lists = [list(arr) for arr in array_list]
            return list(reduce(np.intersect1d, array_lists))
        
        common_indices = intersect_multiple_arrays(pts_indices_across_times)
        matched_theta_Rho_across_times = [] 
        for pts_indicei, theta_Rhoi in zip(pts_indices_across_times, theta_Rhos_across_times):
            matched_theta_Rho_i = theta_Rhoi[[np.where(pts_indicei == idx)[0][0] for idx in common_indices]]
            matched_theta_Rho_across_times.append(matched_theta_Rho_i)
        return matched_theta_Rho_across_times, common_indices
    
    else:
        raise ValueError("Invalid number of arguments. Expected either 4 or 2 arguments.")
