import numpy as np

def compute_D(T_matrix, theta, theta_prime):
    """
    Compute the determinant D(A0; R, t) for given parameters.
    """
    R = T_matrix[:3, :3]
    t = T_matrix[:3, 3]
    
    def skew_symmetric_matrix(t):
        """
        Create a skew-symmetric matrix for a vector t.
        """
        return np.array([
            [0, -t[2], t[1]],
            [t[2], 0, -t[0]],
            [-t[1], t[0], 0]
        ])
    ux = np.array([1, 0, 0])
    uy = np.array([0, 1, 0])
    
    r1 = R[0, :]
    r2 = R[1, :]
        
    t_cross = skew_symmetric_matrix(t)
    
    determinant = - (r1 - np.tan(theta) * r2).T @ t_cross @ (ux - np.tan(theta_prime) * uy)
    
    return determinant 

if __name__ == "__main__":
    T_matrix = np.array([[ 0.77757272,  0.62879302,  0.,          0.        ],
                        [-0.62879302,  0.77757272,  0.,          0.        ],
                        [ 0.,          0.,          1.,          0.2129228 ],
                        [ 0.,          0.,          0.,          1.        ]] )
    theta_Rho = np.array([-1.03251374,  2.01635957])
    theta_Rho_prime = np.array([-0.36251375,  2.05391335])

    determinant = compute_D(T_matrix, theta=theta_Rho[0], theta_prime=theta_Rho_prime[0])
    print(determinant)
    
    
    # 将线性方程组写成矩阵形式 A @ P = b
    R_matrix = T_matrix[:3, :3]
    r1 = R_matrix[0, :]
    r2 = R_matrix[1, :]
    t = T_matrix[:3, 3]


    theta = theta_Rho[0]
    theta_prime = theta_Rho_prime[0]
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
    print("A, b:")
    print(A,b)
    determinant = np.linalg.det(A)
    print("SS:", np.linalg.inv(A) @ b)
    print(A)
    print("Determinant of A:", determinant)
    
    P = np.array([2,0,0])
    print(A @ P)