import numpy as np
from numpy.linalg import inv

def Calculate_CRLB(p_w, R_true, t_true, stdVar_noise_d, stdVar_noise_theta):
    """
    Calculate CRLB (Cramer-Rao Lower Bound)
    
    Args:
        p_w: 3D points in world coordinates (3xN numpy array)
        R_true: True rotation matrix (3x3 numpy array)
        t_true: True translation vector (3x1 numpy array)
        stdVar_noise_d: Standard deviation of distance noise
        stdVar_noise_theta: Standard deviation of angle noise
    
    Returns:
        R_CRLB: CRLB for rotation
        t_CRLB: CRLB for translation
    """
    num = p_w.shape[1]
    
    e_1 = np.array([[1], [0], [0]])
    e_2 = np.array([[0], [1], [0]])
    I = np.eye(3)
    F = np.zeros((6, 6))
    
    phi_function = np.array([
        [0, 0, 0],
        [0, 0, 1],
        [0, -1, 0],
        [0, 0, -1],
        [0, 0, 0],
        [1, 0, 0],
        [0, 1, 0],
        [-1, 0, 0],
        [0, 0, 0]
    ])
    
    for i in range(num):
        u = p_w[:, i:i+1] - t_true
        
        f_d_s = np.array([0, 0, 0])
        f_d_t = (u.T / np.linalg.norm(u))[0]
        
        R_true_T = R_true.T
        vec_R_T = R_true_T.flatten()
        
        g = e_2.T @ np.kron((p_w[:, i:i+1] - t_true).T, np.eye(3)) @ vec_R_T
        h = e_1.T @ np.kron((p_w[:, i:i+1] - t_true).T, np.eye(3)) @ vec_R_T
        
        ukronR = np.kron(((p_w[:, i:i+1] - t_true).T @ R_true), I)
        f_theta_s = ((g * e_1.T - h * e_2.T) @ ukronR @ phi_function) / (h * h)
        f_theta_t = ((h * e_2.T - g * e_1.T) @ R_true_T) / (h * h)
        
        A = (1 / stdVar_noise_theta**2) * (f_theta_s.T @ f_theta_s)
        B = (1 / stdVar_noise_theta**2) * (f_theta_s.T @ f_theta_t)
        C = (1 / stdVar_noise_theta**2) * (f_theta_t.T @ f_theta_s)
        D = (1 / stdVar_noise_d**2) * (f_d_t @ f_d_t.T) + (1 / stdVar_noise_theta**2) * (f_theta_t.T @ f_theta_t)
        
        F += np.block([[A, B], [C, D]])
    
    CRLB = inv(F)
    
    R_CRLB = np.trace(CRLB[0:3, 0:3])
    t_CRLB = np.trace(CRLB[3:6, 3:6])
    
    return R_CRLB, t_CRLB
