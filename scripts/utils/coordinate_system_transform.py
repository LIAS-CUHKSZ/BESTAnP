#!/usr/bin/python3
"""_summary_
The coordinate system of our simulator treat x-axis as the sonar orientation
But the Triangluaration treat y-axis as the sonar orientation

Therefore we need to transform coordinate system
"""
import numpy as np

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