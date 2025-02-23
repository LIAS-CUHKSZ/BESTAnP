import numpy as np
import matlab.engine
from pathlib import Path

# 获取脚本的路径
script_path = Path(__file__).resolve()
# 获取脚本所在的目录
script_dir = script_path.parent
script_dir = str(script_dir) + "/"

# Append the root dir
import sys, roslib, os
BESTAnP_dir = roslib.packages.get_pkg_dir('BESTAnP')
scripts_dir = os.path.abspath(os.path.join(BESTAnP_dir, 'scripts/anp'))
sys.path.append(scripts_dir)

from App_Algorithm_2 import App_Algorithm_2
from Nonapp_Algorithm_2 import Nonapp_Algorithm_2
from Combine_CIO_2 import Combine_CIO_2
from BESTAnP_CIO import BESTAnP_CIO
from ToCAnP import ToCAnP
from Calculate_CRLB import Calculate_CRLB

class AnPAlgorithm:
    def __init__(self, method = None):
        # t_s, R_sw
        self.R_sw = None
        self.t_s = None
        self.method = method

    @staticmethod
    def orthogonalize(r1_Noise, r2_Noise):
        angle_Noise_rad = np.arccos(np.dot(r1_Noise, r2_Noise) / (np.linalg.norm(r1_Noise) * np.linalg.norm(r2_Noise)))
        angle_tran = (np.pi / 2 - angle_Noise_rad) / 2
        k = np.cross(r1_Noise, r2_Noise)
        k = k / np.linalg.norm(k)
        r1_Noise_new = (r1_Noise * np.cos(-angle_tran) + 
                        np.cross(k, r1_Noise) * np.sin(-angle_tran) + 
                        k * np.dot(k, r1_Noise) * (1 - np.cos(-angle_tran)))
        r2_Noise_new = (r2_Noise * np.cos(angle_tran) + 
                        np.cross(k, r2_Noise) * np.sin(angle_tran) + 
                        k * np.dot(k, r2_Noise) * (1 - np.cos(angle_tran)))
        return r1_Noise_new, r2_Noise_new

    @staticmethod
    def rot2aa(R):
        """
        Converts a 3x3 rotation matrix to axis-angle representation.

        Args:
            R (numpy.ndarray): A 3x3 rotation matrix representing a rotation in 3D space.

        Returns:
            tuple: A tuple containing:
                - k (numpy.ndarray): A 3D unit vector representing the axis of rotation. If no rotation is present (theta == 0), k is [0, 0, 0].
                - theta (float): The rotation angle in radians, representing the amount of rotation around the axis k.
        """
        # 计算旋转角度 theta
        theta = np.arccos(np.clip((np.trace(R) - 1) / 2, -1.0, 1.0))
        if theta == 0: # 如果 theta 为零，意味着没有旋转，此时旋转轴向量 k 无意义
            k = np.array([0, 0, 0])
        else: # 如果 theta 不为零，使用以下公式计算旋转轴 k
            k = np.array([(R[2, 1] - R[1, 2]),
                          (R[0, 2] - R[2, 0]),
                          (R[1, 0] - R[0, 1])]) / (2 * np.sin(theta))
        return k, theta
    
# APP(p_w, p_si_noise, phi_max, R_true)
    def compute_R_t(self, P_W, P_SI, phi_max=None, R_true=None):
        
        if self.method == "ToCAnP":
            R_sw, t_s = ToCAnP(P_W, P_SI)
        elif self.method == "App":
            R_sw, t_s = App_Algorithm_2(P_W, P_SI, phi_max)
        elif self.method == "Nonapp":
            R_sw, t_s = Nonapp_Algorithm_2(P_W, P_SI, phi_max, R_true)
        elif self.method == "CombineCIO":
            R_sw, t_s = Combine_CIO_2(P_W, P_SI, phi_max, R_true)
        elif self.method == "BESTAnPCIO":
            R_sw, t_s = BESTAnP_CIO(P_W, P_SI, phi_max)
        self.t_s, self.R_sw = t_s, R_sw
        
        return self.R_sw, self.t_s

    def estimate_accuracy(self, R_sw_gt):
        """
        Evaluate the accuracy of the estimated rotation matrix by comparing it to the ground truth.

        This function computes the axis-angle difference between the estimated rotation matrix and 
        the ground truth rotation matrix. It returns the axis of rotation and the angular deviation (the closer to 0, the better).

        Args:
            R_sw_gt (numpy.ndarray): The ground truth rotation matrix, which is a 3x3 numpy array 
                                    representing the true rotation.

        Returns:
            tuple: A tuple containing:
                - axis (numpy.ndarray): A 3D unit vector representing the axis of rotation difference.
                - theta (float): The angular deviation between the estimated and ground truth rotation 
                                matrices, in radians.
        """
        axis, theta = self.rot2aa(R_sw_gt.T @ self.R_sw)
        return axis, theta

