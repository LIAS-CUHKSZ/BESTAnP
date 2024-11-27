import numpy as np
import math
from scipy.linalg import svd, inv
from scipy.optimize import minimize
from calculate_tz import calculate_tz
from CIO import CIO

from isRotationMatrix import isRotationMatrix,ToRotationMatrix

def Combine_CIO_2(p_w, p_si_noise, phi_max, R_true, py_path=None):
    """
    组合CIO算法实现
    
    Parameters:
    p_w: 3D点的世界坐标 (3×n numpy数组)
    p_si_noise: 测量得到的2D投影坐标 (2×n numpy数组)
    phi_max: 最大俯仰角（弧度）
    R_true: 真实旋转矩阵，用于判断R的符号
    py_path: Python脚本目录路径
    
    Returns:
    R_final_CIO: 最终旋转矩阵
    t_final_CIO: 最终平移向量
    """

    # Nonapp
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
    R_Nonapp = R_Noise_He_new

    # App
    # 以第一个点为世界坐标系原点
    p_w_app = p_w - p_w[:, [0]]
    t_S_Noise_He_app = p_si_noise[:, 0]

    # 构建A矩阵和b向量
    num_points = p_w.shape[1]
    A_app = np.zeros((2 * (num_points - 1), 6))
    b_app = np.zeros(2 * (num_points - 1))

    for i in range(1, num_points):
        A_app[2 * i - 2:2 * i, :3] = np.array([p_w_app[:, i], [0, 0, 0]])
        A_app[2 * i - 2:2 * i, 3:] = np.array([[0, 0, 0], p_w_app[:, i]])
        b_app[2 * i - 2:2 * i] = p_si_noise[:, i] - t_S_Noise_He_app

    # 求解线性方程组
    r_app = np.linalg.inv(A_app.T @ A_app) @ A_app.T @ b_app

    # 归一化
    r_app[:3] = r_app[:3] / np.linalg.norm(r_app[:3])
    r_app[3:] = r_app[3:] / np.linalg.norm(r_app[3:])

    # 检查矩阵秩
    rank_A = np.linalg.matrix_rank(A_app)
    min_dim_A = min(A_app.shape)

    if rank_A != min_dim_A:
        print('矩阵A_app不是满秩的')

        U_Noise_He_app, S_Noise_He_app, V_Noise_He_app = np.linalg.svd(A_app)
        v_1_1 = V_Noise_He_app.T[4, :3]
        v_1_2 = V_Noise_He_app.T[4, 3:6]
        v_2_1 = V_Noise_He_app.T[5, :3]
        v_2_2 = V_Noise_He_app.T[5, 3:6]

        # 构造矩阵F
        F = np.zeros((3, 3))
        F[0, 0] = v_1_1 @ v_1_1
        F[0, 1] = v_2_1 @ v_2_1
        F[0, 2] = 2 * v_1_1 @ v_2_1
        F[1, 0] = v_1_2 @ v_1_2
        F[1, 1] = v_2_2 @ v_2_2
        F[1, 2] = 2 * v_1_2 @ v_2_2
        F[2, 0] = v_1_1 @ v_1_2
        F[2, 1] = v_2_1 @ v_2_2
        F[2, 2] = v_1_1 @ v_2_2 + v_1_2 @ v_2_1

        c = np.array([1, 1, 0])

        # 定义目标函数
        def objective(x):
            return np.linalg.norm(F @ np.array([x[0] ** 2, x[1] ** 2, x[0] * x[1]]) - c) ** 2

        # 优化求解
        x0 = np.array([0.5, 0.5])
        result = minimize(objective, x0, method='BFGS')
        alpha_1, alpha_2 = result.x

        r_app = r_app + alpha_1 * V_Noise_He_app.T[:, 4] + alpha_2 * V_Noise_He_app.T[:, 5]

    # 构造旋转矩阵
    R_Noise_He_app = np.zeros((3, 3))
    R_Noise_He_app[0, :] = r_app[:3]
    R_Noise_He_app[1, :] = r_app[3:6]
    R_Noise_He_app[2, :] = np.cross(r_app[:3], r_app[3:6])

    # 检查并修正旋转矩阵
    if not isRotationMatrix(R_Noise_He_app):
        R_Noise_He_app = ToRotationMatrix(R_Noise_He_app)

    t_S_Noise_He_app = np.append(t_S_Noise_He_app, 0.0)

    # 计算tz
    t_app = calculate_tz(R_Noise_He_app, t_S_Noise_He_app, p_w_app, p_si_noise, phi_max)
    R_app = R_Noise_He_app

    t_app_w = -R_app.T @ t_app
    t_app_w = p_w[:, 0] + t_app_w
    t_app_new = -R_app @ t_app_w

    # 计算投影误差
    Temp_R_Nonapp = R_Nonapp.T
    Temp_R_app = R_app.T

    Temp_t_Nonapp = -R_Nonapp.T @ t_Nonapp
    Temp_t_app = -R_app.T @ t_app_new

    # 计算投影误差
    def compute_projection_error(R, t, p_w, p_si_noise):
        p_s_est = R @ (p_w - t.reshape(-1, 1))
        d_est = np.linalg.norm(p_s_est, axis=0)
        tan_theta = p_s_est[1] / p_s_est[0]
        theta_est = np.arctan(tan_theta)
        p_si_est = np.vstack([d_est * np.cos(theta_est), d_est * np.sin(theta_est)])
        return np.mean(np.linalg.norm(p_si_est - p_si_noise, axis=0))

    error_Nonapp = compute_projection_error(Temp_R_Nonapp, Temp_t_Nonapp, p_w, p_si_noise)
    error_app = compute_projection_error(Temp_R_app, Temp_t_app, p_w, p_si_noise)

    if error_Nonapp > error_app:
        R = R_app
        t = t_app_new
        # print("1")
    else:
        R = R_Nonapp
        t = t_Nonapp
        # print("2")

    # CIO优化
    R_CIO, t_CIO = CIO(R, t, p_w, p_si_noise, phi_max)

    R_final_CIO = R_CIO.T
    t_final_CIO = -R_CIO.T @ t_CIO
    return R_final_CIO, t_final_CIO