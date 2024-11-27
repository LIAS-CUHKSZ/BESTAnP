"""
Transfrom the pose (3 element for postion and 4 for orientation)
To the T matrix
Returns:
    np.array: 4 by 4 matrix
"""

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


def quaternion_to_rotation_matrix(quaternion):
    """将四元数转换为旋转矩阵"""
    return tf.transformations.quaternion_matrix(quaternion)[:3, :3]

def ros_pose_to_transform_matrix(pose):
    """将位姿转换为齐次变换矩阵"""
    position = pose['position']
    orientation = pose['orientation']
    
    # 提取平移向量
    translation = np.array([position['x'], position['y'], position['z']])
    
    # 提取四元数并转换为旋转矩阵
    quaternion = [orientation['x'], orientation['y'], orientation['z'], orientation['w']]
    rotation_matrix = quaternion_to_rotation_matrix(quaternion)
    
    # 构建齐次变换矩阵
    transform_matrix = np.eye(4)
    transform_matrix[:3, :3] = rotation_matrix
    transform_matrix[:3, 3] = translation
    
    return transform_matrix


def rotation_matrix_to_quaternion(rotation_matrix):
    """将旋转矩阵转换为四元数"""
    
    # 使用tf.transformations中的函数将旋转矩阵转换为齐次矩阵
    homogenous_matrix = np.eye(4)
    homogenous_matrix[:3, :3] = rotation_matrix
    
    # 提取四元数
    quaternion = tf.transformations.quaternion_from_matrix(homogenous_matrix)
    
    return quaternion

def transform_matrix_to_ros_pose(transform_matrix):
    """将齐次变换矩阵转换为ROS位姿"""
    
    # 提取平移向量
    translation = transform_matrix[:3, 3]
    position = {
        'x': translation[0],
        'y': translation[1],
        'z': translation[2]
    }
    
    # 提取旋转矩阵
    rotation_matrix = transform_matrix[:3, :3]
    
    # 将旋转矩阵转换为四元数
    quaternion = rotation_matrix_to_quaternion(rotation_matrix)
    orientation = {
        'x': quaternion[0],
        'y': quaternion[1],
        'z': quaternion[2],
        'w': quaternion[3]
    }
    
    # 构建ROS位姿字典
    pose = {
        'position': position,
        'orientation': orientation
    }
    
    return pose
