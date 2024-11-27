import numpy as np
import tf


def quaternion_to_rotation_matrix(quaternion):
    """将四元数转换为旋转矩阵"""
    return tf.transformations.quaternion_matrix(quaternion)[:3, :3]

def pose_to_transform_matrix(pose):
    """将位姿转换为齐次变换矩阵"""
    position = pose.position
    orientation = pose.orientation
    
    # 提取平移向量
    translation = np.array([position.x, position.y, position.z])
    
    # 提取四元数并转换为旋转矩阵
    quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
    rotation_matrix = quaternion_to_rotation_matrix(quaternion)
    
    # 构建齐次变换矩阵
    transform_matrix = np.eye(4)
    transform_matrix[:3, :3] = rotation_matrix
    transform_matrix[:3, 3] = translation
    
    return transform_matrix


if __name__ == '__main__':
    # 示例使用
    pose_N1 = {
        'position': {'x': 1.0, 'y': 2.0, 'z': 3.0},
        'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
    }
    T_N1 = pose_to_transform_matrix(pose_N1)
    print("Transform Matrix T_N1:")
    print(T_N1)
