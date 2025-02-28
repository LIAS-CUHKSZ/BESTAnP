#!/usr/bin/python3

import numpy as np
import cvxpy as cp
from scipy.linalg import eig

import sys, roslib, os
project_root = roslib.packages.get_pkg_dir('lias_anp')
root_dir = os.path.abspath(os.path.join(project_root, 'scripts'))
sys.path.append(root_dir)

# DETERMINANT_THRESHOLD = 0.008
DETERMINANT_THRESHOLD = 0.0

T_z_90 = np.array([[0,-1,0,0],[1,0,0,0],[0,0,1,0],[ 0,0,0,1]])
T_z_min90 = T_z_90.T
R_z_90 = T_z_90[:3, :3]

def coordinate_transform_T(Pose0, Pose1):
    # T1 = T0 @ T
    T_matrix = np.linalg.inv(Pose0) @ Pose1 
    # x-axis oriented switched to y-axis oriented
    T_matrix = T_z_90 @ T_matrix @ T_z_min90
    # get transforamtion matrix
    T_matrix = np.linalg.inv(T_matrix)
    return T_matrix

def coordinate_transform_Pose(Pose):
    return (T_z_90 @ Pose)

def coordinate_transform_pt(P):
    return (R_z_90 @ P)

def coordinate_transform(P0, P1, Pose0, Pose1):
    P0 = coordinate_transform_pt(P0)
    P1 = coordinate_transform_pt(P1)
    T_matrix = coordinate_transform_T(Pose0, Pose1)
    return P0, P1, T_matrix

def ANRS(T_matrix, theta_Rho, theta_Rho_prime):
    """_summary_

    Args:
        T_matrix (_type_): _description_
        theta_Rho (_type_): _description_
        theta_Rho_prime (_type_): _description_

    Returns:
        P_tilde_prime, 
        determinant: DETERMINANT_THRESHOLD is recommended to set to 0.005 to filter out estimated point with high error
        _type_: _description_
    """
    # 将线性方程组写成矩阵形式 A @ P = B
    R_matrix = T_matrix[:3, :3]
    r1 = R_matrix[0, :]
    r2 = R_matrix[1, :]
    t = T_matrix[:3, 3]


    theta = -theta_Rho[0]
    theta_prime = -theta_Rho_prime[0]
    R = theta_Rho[1]  # example value for R
    R_prime = theta_Rho_prime[1] # example value for R'
    
    a1 = np.array([-1, np.tan(theta), 0])
    b1 = 0 
    a2 = np.tan(theta_prime) * r2 - r1
    b2 = t[0] - np.tan(theta_prime) * t[1]
    a3 = t.T @ R_matrix
    b3 = (R_prime**2 - R**2 - np.linalg.norm(t)**2) / 2

    A = np.vstack([a1, a2, a3])
    b = np.array([b1, b2, b3])

    determinant = np.linalg.det(A)
    # if abs(determinant) > DETERMINANT_THRESHOLD:
    # ANRS
    P_o = np.linalg.inv(A) @ b
    norm_P_o = np.linalg.norm(P_o)
    P_hat_o = P_o / norm_P_o

    A_tilde_prime = np.vstack([A, P_hat_o.T])
    b_tilde_prime = np.append(b, R)
    
    P_tilde_prime = np.linalg.inv(A_tilde_prime.T @ A_tilde_prime) @ A_tilde_prime.T @ b_tilde_prime      
    

    # 计算最小二乘解 x = (A^T A)^(-1) A^T b
    # least_square = np.linalg.inv(A.T @ A) @ (A.T @ b)
    
    return P_tilde_prime, determinant 
    # return P_tilde_prime, least_square 

def GTRS_old(T_matrix, theta_Rho, theta_Rho_prime):
 
    # Extract components from T_matrix
    R_matrix = T_matrix[:3, :3]
    t = T_matrix[:3, 3]

    # Extract theta, theta_prime, R, R_prime from inputs
    theta = theta_Rho[0]
    theta_prime = theta_Rho_prime[0]
    R = theta_Rho[1]
    R_prime = theta_Rho_prime[1]

    # Define linear constraints in matrix form A @ P = B
    r1 = R_matrix[0, :]
    r2 = R_matrix[1, :]

    a1 = np.array([-1, np.tan(theta), 0])
    a2 = np.tan(theta_prime) * r2 - r1
    a3 = t.T @ R_matrix
    determinant = np.linalg.det(np.vstack([a1, a2, a3]))
    if abs(determinant) > 0.01:
        
        a1 = np.hstack([a1, 0])
        a2 = np.hstack([a2, 0])
        a3 = np.hstack([a3, 0])
        b1 = 0 
        b2 = t[0] - np.tan(theta_prime) * t[1]
        b3 = (R_prime**2 - R**2 - np.linalg.norm(t)**2) / 2
        a4 = np.array([0, 0, 0, 1])
        b4 = R**2
            
        # 将线性方程组写成矩阵形式 A @ P = B
        A = np.vstack([a1, a2, a3, a4])
        b = np.array([b1, b2, b3, b4])

        ATA_be = A.T @ A
        ATb_be = A.T @ b
        D = np.diag([1,1,1,0])
        g = np.zeros(4)
        g[3] = -0.5

        # eig_lambda = eig(D, ATA_be, right=False)
        eig_lambda = np.real(eig(D, ATA_be, right=False))

        lambda_min = -1 / np.max(eig_lambda)

        
        ## 找二分法的初始左端点
        lambda_l = int(lambda_min) 
        y_l = np.linalg.solve(ATA_be + lambda_l * D, ATb_be - lambda_l * g)
        if y_l.T @ D @ y_l + 2 * g.T @ y_l < 0:    
            while True:
                # print("Determinant ATA_be", np.linalg.det(ATA_be + lambda_l * D))
                y_l = np.linalg.solve(ATA_be + lambda_l * D, ATb_be - lambda_l * g)
                if y_l.T @ D @ y_l + 2 * g.T @ y_l > 0:
                    break
                lambda_l -= 1
        # already satisfy now we want to refine
        while y_l.T @ D @ y_l + 2 * g.T @ y_l > 0:
            lambda_l += 0.1
            y_l = np.linalg.solve(ATA_be + lambda_l * D, ATb_be - lambda_l * g)
        # 细调使其接近0
        while y_l.T @ D @ y_l + 2 * g.T @ y_l <= 0:
            lambda_l -= 0.01
            y_l = np.linalg.solve(ATA_be + lambda_l * D, ATb_be - lambda_l * g)

        # 找二分法的初始右端点
        lambda_u = lambda_l + 0.1
        
        y_temp = np.linalg.solve(ATA_be + lambda_u * D, ATb_be - lambda_u * g)
        left_check =y_temp.T @ D @ y_temp + 2 * g.T @ y_temp
        y_temp = np.linalg.solve(ATA_be + lambda_l * D, ATb_be - lambda_l * g)
        right_check =y_temp.T @ D @ y_temp + 2 * g.T @ y_temp
        print("GTRS BINARY SEARCH CHECK:", left_check, right_check)
        # 二分法找最优lambda
        while (lambda_u - lambda_l) > 1e-14:
            lambda_temp = (lambda_u + lambda_l) / 2
            y_temp = np.linalg.solve(ATA_be + lambda_temp * D, ATb_be - lambda_temp * g)
            if (y_temp.T @ D @ y_temp + 2 * g.T @ y_temp) > 0:
                lambda_l = lambda_temp
            else:
                lambda_u = lambda_temp
        y = np.linalg.solve(ATA_be + lambda_u * D, ATb_be - lambda_u * g)
        pos = y[:3]
    
    else:
        pos = None
    # distance_constraint = y[3] - np.linalg.norm(pos)**2
    # print('Distance constraint value:', distance_constraint)
    
    return pos, determinant


# CVX
def GTRS_cvx(T_matrix, theta_Rho, theta_Rho_prime):
    print("LALA")
    R_matrix = T_matrix[:3, :3]
    t = T_matrix[:3, 3]

    # Extract theta, theta_prime, R, R_prime from inputs
    theta = theta_Rho[0]
    theta_prime = theta_Rho_prime[0]
    R = theta_Rho[1]
    R_prime = theta_Rho_prime[1]

    # Define linear constraints in matrix form A @ P = B
    r1 = R_matrix[0, :]
    r2 = R_matrix[1, :]

    a1 = np.array([-1, np.tan(theta), 0])
    a2 = np.tan(theta_prime) * r2 - r1
    a3 = t.T @ R_matrix
    determinant = np.linalg.det(np.vstack([a1, a2, a3]))
    if abs(determinant) > 0.01:
        
        a1 = np.hstack([a1, 0])
        a2 = np.hstack([a2, 0])
        a3 = np.hstack([a3, 0])
        b1 = 0 
        b2 = t[0] - np.tan(theta_prime) * t[1]
        b3 = (R_prime**2 - R**2 - np.linalg.norm(t)**2) / 2
        a4 = np.array([0, 0, 0, 1])
        b4 = R**2
            
        # 将线性方程组写成矩阵形式 A @ P = B
        A = np.vstack([a1, a2, a3, a4])
        b = np.array([b1, b2, b3, b4]).reshape(-1, 1)

        ATA = A.T @ A
        ATb = A.T @ b
        
        D = np.diag([1,1,1,0])
        g = np.array([0, 0, 0, -0.5]).reshape(-1, 1)

        # Define variables
        t = cp.Variable(1)
        v = cp.Variable(1)

        # Define the objective
        objective = cp.Minimize(t)
        
        # Define the constraint
        constraint = cp.bmat([
            [ATA + v * D, ATb - v * g],  # First row: (4x4), (4x1)
            [(ATb - v * g).T, cp.reshape(t, (1, 1))]  # Second row: (1x4), (1x1)
        ]) >> 0

        # Solve the problem
        problem = cp.Problem(objective, [constraint])
        problem.solve(solver=cp.SCS, verbose=False)

        # Compute y and x
        y = np.linalg.inv(ATA + v.value * D) @ (ATb - v.value * g)
        x = y[:3]

        # Print the results
    else:
        x = None
    
    return x, determinant

def GTRS(T_matrix, theta_Rho, theta_Rho_prime):

    """     return x, determinant    """
        
    R_matrix = T_matrix[:3, :3]
    t = T_matrix[:3, 3]

    # Extract theta, theta_prime, R, R_prime from inputs
    theta = theta_Rho[0]
    theta_prime = theta_Rho_prime[0]
    R = theta_Rho[1]
    R_prime = theta_Rho_prime[1]

    # Define linear constraints in matrix form A @ P = B
    r1 = R_matrix[0, :]
    r2 = R_matrix[1, :]

    a1 = np.array([-1, np.tan(theta), 0])
    a2 = np.tan(theta_prime) * r2 - r1
    a3 = t.T @ R_matrix
    determinant = np.linalg.det(np.vstack([a1, a2, a3]))
    if abs(determinant) > DETERMINANT_THRESHOLD:
        
        a1 = np.hstack([a1, 0])
        a2 = np.hstack([a2, 0])
        a3 = np.hstack([a3, 0])
        b1 = 0 
        b2 = t[0] - np.tan(theta_prime) * t[1]
        b3 = (R_prime**2 - R**2 - np.linalg.norm(t)**2) / 2
        a4 = np.array([0, 0, 0, 1])
        b4 = R**2
            
        # 将线性方程组写成矩阵形式 A @ P = B
        A = np.vstack([a1, a2, a3, a4])
        b = np.array([b1, b2, b3, b4]).reshape(-1, 1)

        ATA = A.T @ A
        ATb = A.T @ b
        
        D = np.diag([1,1,1,0])
        f = np.array([0, 0, 0, -0.5]).reshape(-1, 1)
       
        lambda_var = cp.Variable()
        objective = cp.Minimize(lambda_var)
        constraint = [A.T @ A + lambda_var * D >> 0]
        problem = cp.Problem(objective, constraint)
        problem.solve(solver=cp.SCS, verbose=False)
        lambda_val = lambda_var.value
        lambda_l = lambda_val + 2
        lambda_u = lambda_val + 2

        # Define functions for y_l and y_u computation
        def compute_y(A, D, lambda_value, b, f):
            return np.linalg.inv(A.T @ A + lambda_value * D) @ (A.T @ b - lambda_value * f)

        # Compute y_l and adjust lambda_l
        y_l = compute_y(A, D, lambda_l, b, f)
        while (y_l.T @ D @ y_l + 2 * f.T @ y_l).item() < 0:
            lambda_l = (lambda_val + lambda_l) / 2
            y_l = compute_y(A, D, lambda_l, b, f)

        # Compute y_u and adjust lambda_u
        y_u = compute_y(A, D, lambda_u, b, f)
        while (y_u.T @ D @ y_u + 2 * f.T @ y_u).item() > 0:
            lambda_u = lambda_u + (lambda_u - lambda_val) * 2
            y_u = compute_y(A, D, lambda_u, b, f)

        # Perform bisection method to refine lambda
        while (lambda_u - lambda_l) > 1e-14:
            lambda_temp = (lambda_u + lambda_l) / 2
            y_temp = compute_y(A, D, lambda_temp, b, f)
            if (y_temp.T @ D @ y_temp + 2 * f.T @ y_temp).item() > 0:
                lambda_l = lambda_temp
            else:
                lambda_u = lambda_temp

        # Final computation of y and x
        y = compute_y(A, D, lambda_u, b, f)
        x = y[:3]

        # Print the final results
        # print("Final lambda value:", lambda_u)
        # print("Final y:", y)
        # print("Final x:", x)
    else:
        x = None
    
    return x.reshape(-1), determinant

def reconstrunction_error(P, ps, ps_prime, T_matrix):
    
    def project_to_2d(P):
        X, Y, Z = P
        theta = np.arctan(X/Y)
        d = np.sqrt(X**2 + Y**2 + Z**2)
        x_s = d * np.sin(theta)
        y_s = d * np.cos(theta)
        return np.array([x_s, y_s])
    
    R = T_matrix[:3, :3]
    t = T_matrix[:3, 3]
    # 投影点
    ps_hat = project_to_2d(P)
    P_prime = R @ P + t
    ps_prime_hat = project_to_2d(P_prime)
    
    # 计算误差
    error_ps = ps - ps_hat
    error_ps_prime = ps_prime - ps_prime_hat
    
    # 计算加权误差
    error = np.sum(error_ps**2) + np.sum(error_ps_prime**2)
    return error

# 定义梯度下降法进行优化
def gradient_descent(P_init, theta_Rho, theta_Rho_prime, T_matrix, learning_rate=0.01, max_iter=1000, tol=1e-3):
    
    def project_to_2d(P):
        X, Y, Z = P
        theta = np.arctan2(X, Y)
        d = np.sqrt(X**2 + Y**2 + Z**2)
        x_s = d * np.sin(theta)
        y_s = d * np.cos(theta)
        return np.array([x_s, y_s])

    # 定义误差函数
    def error_function(P, ps, ps_prime, T_matrix):
        R = T_matrix[:3, :3]
        t = T_matrix[:3, 3]
        # 投影点
        ps_hat = project_to_2d(P)
        P_prime = R @ P + t
        ps_prime_hat = project_to_2d(P_prime)
        
        # 计算误差
        error_ps = ps - ps_hat
        error_ps_prime = ps_prime - ps_prime_hat
        
        # 计算加权误差
        error = error_ps.T @ np.diag(error_ps) @ error_ps + error_ps_prime.T @ np.diag(error_ps_prime) @ error_ps_prime
        return error
    
    theta = theta_Rho[0]
    R = theta_Rho[1]
    theta_prime = theta_Rho_prime[0]
    R_prime = theta_Rho_prime[1]
    ps = np.array([R * np.sin(theta), R * np.cos(theta)])
    ps_prime = np.array([R_prime * np.sin(theta_prime), R_prime * np.cos(theta_prime)])
    
    
    P = P_init
    previous_error = float('inf')
    for i in range(max_iter):
        # 计算当前误差
        error = error_function(P, ps, ps_prime, T_matrix)
        # 计算梯度
        grad = np.zeros_like(P)
        for j in range(len(P)):
            P_temp = P.copy()
            P_temp[j] += tol
            grad[j] = (error_function(P_temp, ps, ps_prime, T_matrix) - error) / tol
        
        # if error increase, we need to take the half of step learning rate
        if previous_error < error:
            learning_rate /= 2
        step = learning_rate * grad
        P_new = P + step
        
        P = P_new
        previous_error = error
        # 检查收敛
        if np.linalg.norm(P_new - P) < tol:
            break
        
        # print(f"Iteration {i+1}, Error: {error}")
    if previous_error > tol:
        return P, False
    return P, True 

T_z_90 = np.array([[0,-1,0,0],[1,0,0,0],[0,0,1,0],[ 0,0,0,1]])
T_z_min90 = T_z_90.T
R_z_90 = T_z_90[:3, :3]
R_z_min90 = T_z_min90[:3, :3]

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

def coordinate_transform_Pose_back(Pose):
    return (T_z_min90 @ Pose @ T_z_90)

def coordinate_transform_pt_back(P):
    return (R_z_min90 @ P)


def mat_difference(T1, T2):
    """计算并打印两个旋转矩阵之间的差别"""
    R1, R2 = T1[:3, :3], T2[:3, :3]
    t1, t2 = T1[:3, 3], T2[:3, 3]
    print("Distance: ", np.linalg.norm(t1-t2))
    # 计算相对旋转矩阵
    R_diff = R1 @ R2.T
    
    # Frobenius范数
    frob_norm = np.linalg.norm(R1 - R2, 'fro')
    
    # 计算旋转角度，添加裁剪避免数值误差
    cos_theta = (np.trace(R_diff) - 1) / 2
    cos_theta = np.clip(cos_theta, -1.0, 1.0)  # 裁剪到[-1, 1]范围
    theta = np.arccos(cos_theta)
    angle_deg = np.degrees(theta)
    
    print(f"Frobenius范数: {frob_norm:.4f}")
    print(f"旋转角度: {angle_deg:.2f}度")
    print("相对旋转矩阵:\n", R_diff)
    print()
    
if __name__ == "__main__":

    T1 = np.array([[ 1.11022302e-16,  4.26659808e-01, -9.04412189e-01,
         2.66508834e+00],
       [-9.95556223e-01, -8.51676195e-02, -4.01781407e-02,
         1.88617536e+00],
       [-9.41690310e-02,  9.00393183e-01,  4.24763827e-01,
         5.33254417e-01],
       [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
         1.00000000e+00]])
    R1 = T1[:3, :3]
    t1 = T1[:3, 3]
    
    T2 = np.array([[ 3.33066907e-16,  3.90151684e-01, -9.20750598e-01,
         2.68767402e+00],
       [-9.95594358e-01, -8.63341734e-02, -3.65825700e-02,
         1.87805869e+00],
       [-9.37649931e-02,  9.16694100e-01,  3.88432816e-01,
         5.34383701e-01],
       [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
         1.00000000e+00]])
    R2 = T2[:3, :3]
    t2 = T2[:3, 3]
    
    P = np.array([ 3.06298065, -3.10321116, -1.26090062])
    P_S1 = np.linalg.inv(R1) @ (P.T - t1)
    print(P_S1)
    P_S2 = np.linalg.inv(R2) @ (P.T - t2)
    
    X, Y, Z = P_S1[0], P_S1[1], P_S1[2]
    Rho = np.sqrt(X**2 + Y**2 + Z**2)
    theta = np.arctan(Y/X)
    theta_Rho = [theta, Rho]
    
    X, Y, Z = P_S2[0], P_S2[1], P_S2[2]
    Rho = np.sqrt(X**2 + Y**2 + Z**2)
    theta = np.arctan(Y/X)
    theta_Rho_prime = [theta, Rho]

    T1_tri = coordinate_transform_Pose(T1)
    T2_tri = coordinate_transform_Pose(T2)
    T_matrix = np.linalg.inv(T2_tri) @ T1_tri
    # array([[ 9.99200038e-01,  3.73671513e-04, -3.99893019e-02, 1.05477951e-02],
    #    [-3.67040888e-04,  9.99999918e-01,  1.73152908e-04, -7.97502355e-03],
    #    [ 3.99893636e-02, -1.58336655e-04,  9.99200092e-01, 2.00601987e-02],
    #    [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,1.00000000e+00]])
   
    
    # Our calculated output
    # T_matrix_ = np.array([
    # [ 0.99947266,  0.01165186, -0.03030888, -0.0139405 ],
    # [-0.00746575,  0.99085433,  0.13472919,  0.01267297],
    # [ 0.03160153, -0.13443186,  0.99041881,  0.06271355],
    # [ 0.        ,  0.        ,  0.        ,  1.        ]])
    T_matrix_ = np.array([
    [ 0.99947266,  0.01165186, -0.03030888, -0.0139405 ],
    [-0.00746575,  0.99085433,  0.13472919,  0.01267297],
    [ 0.03160153, -0.13443186,  0.99041881,  0.06271355],
    [ 0.        ,  0.        ,  0.        ,  1.        ]])
    mat_difference(T_matrix, T_matrix_)
    
    s_P, determinant = ANRS(T_matrix_, theta_Rho, theta_Rho_prime)
    # s_P, determinant = GTRS(T_matrix, theta_Rho, theta_Rho_prime)
    print("SP take a look")
    print(s_P, '\n', coordinate_transform_pt(P_S1))
    print()
    # s_P[0] = -s_P[0]
    # s_P[1] = -s_P[1]
    w_P = ( T1_tri @ np.hstack([s_P, 1]) )[:3]
    w_P = coordinate_transform_pt_back(w_P)
    difference = np.linalg.norm( P - w_P )
    # print(w_P, '\n', P)
    print("difference:", difference)
    print(s_P[0]>0)
    
    # P_ = coordinate_transform_pt(P)
    # s_P_ = ( np.linalg.inv(T1_tri) @ np.hstack([P_, 1]) )[:3]
    
    theta, R = -theta_Rho[0], theta_Rho[1]
    theta_prime, R_prime = -theta_Rho_prime[0], theta_Rho_prime[1]
    ps, ps_prime = np.array([R * np.sin(theta), R * np.cos(theta)]), np.array([R_prime * np.sin(theta_prime), R_prime * np.cos(theta_prime)])
    recon_error = reconstrunction_error(s_P, ps, ps_prime, T_matrix)
    print("recon_error: ", recon_error)

