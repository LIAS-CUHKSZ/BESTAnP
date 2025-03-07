import numpy as np
from scipy.optimize import minimize
from scipy.optimize import NonlinearConstraint

def calculate_tz(R_Noise_He_new,t_S_Noise_He_opt,p_w,p_si_noise,phi_max):
    # 将传入的参数转换为 NumPy 数组，确保类型正确
    R_Noise_He_new = np.array(R_Noise_He_new)
    t_S_Noise_He_opt = np.array(t_S_Noise_He_opt)
    p_w = np.array(p_w)
    p_si_noise = np.array(p_si_noise)

    def func_z_norm(tz, R, tx, ty, x3d, x2d):
        t = np.array([[tx,ty,tz]]).transpose()
        xc = np.dot(R,x3d)+t
        xc = xc.astype('float')
        r=np.sqrt(xc[0]**2+xc[1]**2+xc[2]**2)
        theta = np.arctan(xc[1]/xc[0])
        xs = r*np.cos(theta)
        ys = r*np.sin(theta)
        pc = np.array([xs,ys])
        residual = x2d-pc
        residual = residual.reshape(-1)
        
        return np.linalg.norm(residual)

    def con(tz):
        t = np.array([[t_S_Noise_He_opt[0],t_S_Noise_He_opt[1],tz]]).transpose()
        # xc = np.dot(R_c2n,x3d)+t
        xc = np.dot(R_Noise_He_new,p_w)+t
        xc = xc.astype('float')
        r=np.sqrt(xc[0]**2+xc[1]**2)
        phi = np.arctan(xc[2]/r)

        return phi
    


    nlc = NonlinearConstraint(con, -phi_max, phi_max)  # phi_max单位是弧度
    result_tz_c2 = minimize(func_z_norm, 0.0, args=(R_Noise_He_new, t_S_Noise_He_opt[0], t_S_Noise_He_opt[1], p_w, p_si_noise),constraints=nlc,method="SLSQP")
    
    t_S_Noise_He_opt[2] = result_tz_c2.x
    return  t_S_Noise_He_opt


