import numpy as np
from scipy.optimize import minimize
from scipy.optimize import NonlinearConstraint
from scipy.spatial.transform import Rotation as R

def CIO(R_Noise_He_new,t_S_Noise_He_opt,p_w,p_si_noise,phi_max):
    # 将传入的参数转换为 NumPy 数组，确保类型正确
    R_Noise_He_new = np.array(R_Noise_He_new)
    t_S_Noise_He_opt = np.array(t_S_Noise_He_opt)
    p_w = np.array(p_w)
    p_si_noise = np.array(p_si_noise)

    ###########################################################
    # CIO
    # 定义误差函数
    def func_qt_norm(qt, x3d, x2d):
        q = qt[:4]  # 四元数 q
        t = np.array(qt[4:7])  # 平移向量 t
        t = t[:, np.newaxis]
        R_temp = R.from_quat(q)  # 四元数转换为旋转矩阵
        R_matrix = np.array(R_temp.as_matrix())
    
        
        xc = np.dot(R_matrix, x3d) + t
        xc = xc.astype('float')
        r = np.sqrt(xc[0]**2 + xc[1]**2 + xc[2]**2)
        theta = np.arctan(xc[1]/xc[0])
        xs = r*np.cos(theta)
        ys = r*np.sin(theta)
        pc = np.array([xs,ys])
        residual = x2d-pc
        residual = residual.reshape(-1)
        return np.linalg.norm(residual)
    
    
    
    # 定义约束条件：角度限制
    def con_qt(qt):
        q = qt[:4]
        t = np.array(qt[4:7])
        t = t[:, np.newaxis]
        r = R.from_quat(q)
        R_matrix = np.array(r.as_matrix())
    
        
        xc = np.dot(R_matrix, p_w) + t
        xc = xc.astype('float')
        r = np.sqrt(xc[0]**2 + xc[1]**2)
        phi = np.arctan(xc[2]/r)# 计算 phi 角度
        return phi
    
    q_init = R.from_matrix(R_Noise_He_new).as_quat()
    t_init = t_S_Noise_He_opt
    
    # 优化的初始猜测：使用已知的四元数和位移向量 
    qt_init = np.hstack((q_init, t_init)) #拼接四元数和t
    
    # 设置非线性约束条件
    nlc_CIO = NonlinearConstraint(con_qt, -phi_max, phi_max)
    # 执行优化
    result_qt_CIO = minimize(func_qt_norm, qt_init, args=(p_w, p_si_noise), constraints=nlc_CIO, method="SLSQP")
    
    # 获取优化后的四元数和位移
    q_CIO = result_qt_CIO.x[:4]  # 优化后的四元数
    # 将四元数转换为旋转矩阵
    R_CIO = np.array(R.from_quat(q_CIO).as_matrix())
    t_CIO = np.array(result_qt_CIO.x[4:7])  # 优化后的位移向量

    t_S_Noise_He_opt = t_S_Noise_He_opt.reshape(3, 1)
    return R_CIO,t_CIO



