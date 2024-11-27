import numpy as np
from scipy.linalg import svd, norm
from scipy.optimize import minimize
from calculate_tz import calculate_tz  # 假设这是一个已有的Python模块

from isRotationMatrix import isRotationMatrix,ToRotationMatrix

def App_Algorithm_2(p_w, p_si_noise, phi_max, py_path=None):
    """
    App算法实现
    
    Parameters:
    p_w: 3D点的世界坐标 (3×n numpy数组)
    p_si_noise: 测量得到的2D投影坐标 (2×n numpy数组)
    phi_max: 最大俯仰角（弧度）
    py_path: Python脚本目录路径（在这个Python实现中不需要）
    
    Returns:
    R_final: 最终旋转矩阵
    t_final: 最终平移向量
    """

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
    t_S_Noise_He_app = calculate_tz(R_Noise_He_app, t_S_Noise_He_app, p_w_app, p_si_noise, phi_max)

    # 计算最终结果
    t_S_Noise_He_app_w = -R_Noise_He_app.T @ t_S_Noise_He_app
    t_app = p_w[:, 0] + t_S_Noise_He_app_w
    R_app = R_Noise_He_app.T
    
    return R_app, t_app