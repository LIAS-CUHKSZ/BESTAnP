import numpy as np
from scipy.linalg import svd, inv
from calculate_tz import calculate_tz

from isRotationMatrix import isRotationMatrix,ToRotationMatrix

def Nonapp_Algorithm_2(p_w, p_si_noise, phi_max, R_true, py_path=None):
    """
    非近似算法实现
    
    Parameters:
    p_w: 3D点的世界坐标 (3×n numpy数组)
    p_si_noise: 测量得到的2D投影坐标 (2×n numpy数组)
    phi_max: 最大俯仰角（弧度）
    R_true: 真实旋转矩阵，用于判断R的符号
    py_path: Python脚本目录路径
    
    Returns:
    R_final: 最终旋转矩阵
    t_final: 最终平移向量
    """

    num = len(p_w[0])
    # Create W_Noise_He and H_Noise_He
    W_Noise_He = np.zeros((num, 2))
    H_Noise_He = np.zeros((num, 6))

    for i in range(num):
        W_Noise_He[i, 0] = -p_si_noise[1, i]
        W_Noise_He[i, 1] = p_si_noise[0, i]
        H_Noise_He[i, 0:3] = -p_si_noise[1, i] * p_w[:, i]
        H_Noise_He[i, 3:6] = p_si_noise[0, i] * p_w[:, i]

    # Calculate M_Noise_He
    M_Noise_He = np.dot(W_Noise_He, np.linalg.inv(np.dot(W_Noise_He.T, W_Noise_He)))
    M_Noise_He = np.dot(M_Noise_He, W_Noise_He.T)
    M_Noise_He = np.dot(M_Noise_He, H_Noise_He) - H_Noise_He

    # Perform SVD decomposition
    U_Noise_He, S_Noise_He, V_Noise_He = svd(M_Noise_He)

    # Calculate r_1 and r_2
    r_1 = np.sqrt(2) * V_Noise_He[5, :]
    r_2 = -np.sqrt(2) * V_Noise_He[5, :]

    # print(r_1)

    # Select r based on condition and calculate t_S_Noise_He
    if np.dot(r_1[0], R_true[0, 0]) > 0:
        r = r_1
        t_S_Noise_He = -np.dot(np.linalg.inv(np.dot(W_Noise_He.T, W_Noise_He)),
                               np.dot(W_Noise_He.T, np.dot(H_Noise_He, r_1)))
    else:
        r = r_2
        t_S_Noise_He = -np.dot(np.linalg.inv(np.dot(W_Noise_He.T, W_Noise_He)),
                               np.dot(W_Noise_He.T, np.dot(H_Noise_He, r_2)))

    # Build rotation matrix
    R_Noise_He = np.zeros((3, 3))
    R_Noise_He[0, :] = r[0:3]
    R_Noise_He[1, :] = r[3:6]
    R_Noise_He[2, :] = np.cross(r[0:3], r[3:6])

    # Check if R_Noise_He is a rotation matrix and correct if needed
    if not isRotationMatrix(R_Noise_He):
        R_Noise_He_new = ToRotationMatrix(R_Noise_He)
    else:
        R_Noise_He_new = R_Noise_He

    # print(R_Noise_He_new)

    r_opt = np.concatenate((R_Noise_He_new[0, :], R_Noise_He_new[1, :]))
    t_S_Noise_He_opt = np.array([0.0, 0.0, 0.0])
    t_S_Noise_He_opt[0:2] = -np.dot(np.linalg.inv(np.dot(W_Noise_He.T, W_Noise_He)),
                                    np.dot(W_Noise_He.T, np.dot(H_Noise_He, r_opt)))
    # 计算tz
    t_Nonapp = calculate_tz(R_Noise_He_new, t_S_Noise_He_opt, p_w, p_si_noise, phi_max)

    # 计算最终结果
    t_Nonapp = -R_Noise_He_new.T @ t_Nonapp
    R_Nonapp = R_Noise_He_new.T
    
    return R_Nonapp, t_Nonapp