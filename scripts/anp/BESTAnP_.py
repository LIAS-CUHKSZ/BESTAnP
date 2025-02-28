import numpy as np
from scipy.linalg import expm, eig, inv

from isRotationMatrix import isRotationMatrix,ToRotationMatrix


def BESTAnP(p_w, p_si_noise):
    """
    BESTAnP
    
    Parameters:
    p_w: 3D点的世界坐标 (3×n numpy数组)
    p_si_noise: 测量得到的2D投影坐标 (2×n numpy数组)
    
    Returns:
    R_est_noise_GN: 估计的旋转矩阵
    t_est_noise_GN: 估计的平移向量
    """

    # 处理p_si_noise获得d_noise和tan_theta_noise
    tan_theta_noise = p_si_noise[1, :] / p_si_noise[0, :]
    norm_p_si = np.linalg.norm(p_si_noise[:2, :], axis=0)
    cos_theta_noise = p_si_noise[0, :] / norm_p_si
    sin_theta_noise = p_si_noise[1, :] / norm_p_si
    theta_noise = np.arctan(tan_theta_noise)

    d_noise = p_si_noise[0, :] / cos_theta_noise

    # 获得点数
    num_points = p_w.shape[1]

    # 估计平移矢量t
    A_noise = np.hstack((-2 * p_w.T, np.ones((num_points, 1))))
    b_noise = (d_noise ** 2 - np.sum(p_w ** 2, axis=0))[:, np.newaxis]

    x_noise = np.linalg.solve(A_noise.T @ A_noise, A_noise.T @ b_noise)
    t_est_noise = x_noise[:3].flatten()

    stdVar_noise_d_est = np.sqrt(abs(x_noise[3] - np.linalg.norm(t_est_noise) ** 2))

    # 估计R
    p_w_centered = p_w - t_est_noise.reshape(3, 1)
    B_noise = np.hstack([np.diag(tan_theta_noise) @ p_w_centered.T, -p_w_centered.T])

    Q = B_noise.T @ B_noise / num_points
    S = np.zeros((6, 6))
    S[:3, :3] = p_w_centered @ p_w_centered.T / num_points

    # 计算最大特征值对应的sigma^2
    eigvals = eig(Q, S)[0]
    stdVar_noise_theta_est = np.sqrt(1 / max(eigvals.real))

    C = stdVar_noise_theta_est ** 2 * S
    Q_BE = Q - C

    eigvals, eigvecs = np.linalg.eig(Q_BE)
    eigvec = eigvecs[:, np.argmin(eigvals)]

    R_est_noise_1 = np.vstack([
        np.sqrt(2) * eigvec[:3],
        np.sqrt(2) * eigvec[3:6]
    ])

    R_est_noise_2 = -R_est_noise_1

    p_s_est_noise_1 = R_est_noise_1 @ (p_w[:, 0] - t_est_noise)

    R_est_noise = R_est_noise_1 if p_s_est_noise_1[0] * cos_theta_noise[0] > 0 else R_est_noise_2

    R_est_noise = np.vstack([
        R_est_noise,
        np.cross(R_est_noise[0, :], R_est_noise[1, :])
    ])

    if not isRotationMatrix(R_est_noise):
        R_est_noise = ToRotationMatrix(R_est_noise)

    R_est_noise = R_est_noise.T

    # 高斯牛顿优化
    e_1 = np.array([1, 0, 0])
    e_2 = np.array([0, 1, 0])
    I = np.eye(3)

    Residuals_R = np.zeros(2 * num_points)
    J_R = np.zeros((2 * num_points, 6))

    for i in range(num_points):
        Residuals_R[2 * i] = (d_noise[i] - np.linalg.norm(p_w[:, i] - t_est_noise))
        Residuals_R[2 * i + 1] = (tan_theta_noise[i] -
                                  (e_2 @ R_est_noise.T @ (p_w[:, i] - t_est_noise)) /
                                  (e_1 @ R_est_noise.T @ (p_w[:, i] - t_est_noise)))

        J_phi = np.array([[0, 0, 0],
                          [0, 0, 1],
                          [0, -1, 0],
                          [0, 0, -1],
                          [0, 0, 0],
                          [1, 0, 0],
                          [0, 1, 0],
                          [-1, 0, 0],
                          [0, 0, 0]])

        vec_R = R_est_noise.T.flatten()
        ukronR = np.kron((p_w[:, i] - t_est_noise).T @ R_est_noise, I)
        g = e_2 @ np.kron((p_w[:, i] - t_est_noise).T, np.eye(3)) @ vec_R
        h = e_1 @ np.kron((p_w[:, i] - t_est_noise).T, np.eye(3)) @ vec_R

        norm_diff = np.linalg.norm(p_w[:, i] - t_est_noise)
        J_R[2 * i, 3:6] = (p_w[:, i] - t_est_noise).T / norm_diff
        J_R[2 * i + 1, :3] = -(h * e_2 - g * e_1) @ ukronR @ J_phi / h ** 2
        J_R[2 * i + 1, 3:6] = (h * e_2 - g * e_1) @ R_est_noise.T / h ** 2

    temp_result = np.concatenate([[0, 0, 0], t_est_noise]) - inv(J_R.T @ J_R) @ J_R.T @ Residuals_R

    t_est_noise_GN = temp_result[3:6]
    s_new = temp_result[:3]
    s_matrix = np.array([[0, -s_new[2], s_new[1]],
                         [s_new[2], 0, -s_new[0]],
                         [-s_new[1], s_new[0], 0]])

    R_est_noise_GN = R_est_noise @ expm(s_matrix)
    
    return R_est_noise_GN, t_est_noise_GN