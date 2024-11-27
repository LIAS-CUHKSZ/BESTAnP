import numpy as np
from scipy.spatial.transform import Rotation

def add_noise_to_pose(T, rotation_noise_std=0.001, translation_noise_std=0.01):
    """
    给4x4变换矩阵添加噪声
    
    参数:
    T: 4x4 numpy 数组，原始变换矩阵
    rotation_noise_std: 旋转噪声的标准差（弧度）
    translation_noise_std: 平移噪声的标准差（与T中平移单位相同）
    
    返回:
    noisy_T: 添加了噪声的4x4变换矩阵
    """
    # 提取旋转矩阵和平移向量
    R = T[:3, :3]
    t = T[:3, 3]
    
    # 为旋转添加噪声
    rotation_noise = Rotation.from_rotvec(
        np.random.normal(0, rotation_noise_std, 3)
    ).as_matrix()
    noisy_R = np.dot(R, rotation_noise)
    
    # 为平移添加噪声
    noisy_t = t + np.random.normal(0, translation_noise_std, 3)
    
    # 构造新的带噪声的变换矩阵
    noisy_T = np.eye(4)
    noisy_T[:3, :3] = noisy_R
    noisy_T[:3, 3] = noisy_t
    
    return noisy_T

# 使用示例
if __name__ == "__main__":
    # 创建一个示例变换矩阵
    original_T = np.array([
        [0.9689, -0.2474, 0.0131, 1.5000],
        [0.2473, 0.9689, 0.0131, 2.0000],
        [-0.0153, -0.0102, 0.9998, 0.5000],
        [0.0000, 0.0000, 0.0000, 1.0000]
    ])

    # 添加噪声
    noisy_T = add_noise_to_pose(original_T)

    print("Original transformation matrix:")
    print(original_T)
    print("\nNoisy transformation matrix:")
    print(noisy_T-original_T)