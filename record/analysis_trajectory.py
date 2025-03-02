import csv
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from collections import Counter


class TrajectoryPlotter:
    def __init__(self):
        self.fig = plt.figure(figsize=(10, 8))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.trajectories = []

    def add_trajectory(self, poses, color_t, color_l, label):
        """
        Add the 3D trajectory of the given poses to the plot.
        
        Parameters:
        poses (list): A list of 4x4 transformation matrices representing the poses.
        color (str)
        label (str): The label for the trajectory.
        """
        self.trajectories.append((poses, color_t, color_l, label))

    def plot_all(self):
        """
        Plot all the added trajectories and display the plot.
        """
        # Plot all the trajectories
        for poses, color_t, color_l, label in self.trajectories:
            # colors = [cm.get_cmap(color + 's')( i / len(poses)) for i in range(len(poses))]
            colors = [cm.get_cmap(color_t + 's') ( (i + 0.7*len(poses)) / (2*len(poses)) ) for i in range(len(poses))]
            # Extract the x, y, z coordinates from the transformation matrices
            poses_x = [pose[0, 3] for pose in poses]
            poses_y = [pose[1, 3] for pose in poses]
            poses_z = [pose[2, 3] for pose in poses]
            
            n_points = len(poses_x)
            
            # Plot the trajectory with color gradient
            for i in range(n_points - 1):
                self.ax.plot([poses_x[i], poses_x[i+1]], [poses_y[i], poses_y[i+1]], [poses_z[i], poses_z[i+1]], color=colors[i])
            self.ax.plot([], [], color_l, label=label)

        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        self.ax.set_title('Trajectory')
        
        # Add legend
        self.ax.legend()
        # self.ax.set_zlim(0.25,0.65)
        
        self.ax.grid(True)
        plt.show()

def read_csv_file(file_path):
    """
    Read the CSV file and return the real and estimated poses.
    """
    real_poses = []
    estimated_poses = []
    coordinates_list = []
    with open(file_path, 'r') as file:
        reader = csv.reader(file)
        for row in reader:
            estimated_pose = np.array([float(x) for x in row[:16]]).reshape(4, 4)
            real_pose = np.array([float(x) for x in row[16:32]]).reshape(4, 4)
            
            # 读取剩余的元素作为坐标点
            coordinates = [float(x) for x in row[32:]]
            # 将坐标点重塑为 (n, 3) 的形状，其中 n 是坐标点的数量
            coordinates = np.array(coordinates).reshape(-1, 3)
            
            real_poses.append(real_pose)
            estimated_poses.append(estimated_pose)
            coordinates_list.append(coordinates)
    
    return real_poses, estimated_poses, coordinates_list


def calculate_ATE(real_poses, estimated_poses):
    """
    Calculate the Absolute Trajectory Error (ATE) between the real and estimated poses.

    Parameters:
    real_poses (list): A list of 4x4 transformation matrices representing the real poses.
    estimated_poses (list): A list of 4x4 transformation matrices representing the estimated poses.

    Returns:
    tuple: (translation_rmse, rotation_rmse)
        translation_rmse (float): The Absolute Trajectory Error (ATE) in translation.
        rotation_rmse (float): The Absolute Trajectory Error (ATE) in rotation.
    """
    assert len(real_poses) == len(estimated_poses), "Real and estimated poses must have the same length."

    translation_errors = []
    rotation_errors = []
    for i in range(len(real_poses)):
        real_pose = real_poses[i]
        estimated_pose = estimated_poses[i]

        # Calculate the transformation error between the real and estimated poses
        pose_error = np.linalg.inv(real_pose) @ estimated_pose

        # Extract the translation and rotation components
        translation_component = pose_error[:3, 3]
        rotation_component = pose_error[:3, :3]

        # Calculate the translation error
        translation_error = np.linalg.norm(translation_component)
        translation_errors.append(translation_error)

        # Calculate the rotation error as the sum of squared differences between the elements
        # rotation_error = np.sum(np.square(rotation_component - np.eye(3)))
        rotation_error = np.arccos(np.clip((np.trace(rotation_component) - 1) / 2, -1.0, 1.0))
        rotation_errors.append(np.rad2deg(rotation_error))

    translation_rmse = np.sqrt(np.mean(np.square(translation_errors)))
    rotation_rmse = np.sqrt(np.mean(np.square(rotation_errors)))
    # print(translation_errors)
    return np.round(translation_rmse, 4), np.round(rotation_rmse, 2) 
    # return translation_rmse, rotation_rmse

def calculate_RPE(real_poses, estimated_poses):
    """
    Calculate the Relative Trajectory Error (RTE) between the real and estimated poses.

    Parameters:
    real_poses (list): A list of 4x4 transformation matrices representing the real poses.
    estimated_poses (list): A list of 4x4 transformation matrices representing the estimated poses.

    Returns:
    tuple: (translation_error, rotation_error)
        translation_error (float): The Relative Trajectory Error (RTE) in translation.
        rotation_error (float): The Relative Trajectory Error (RTE) in rotation.
    """
    assert len(real_poses) == len(estimated_poses), "Real and estimated poses must have the same length."

    translation_errors = []
    rotation_errors = []
    
    for i in range(1, len(real_poses)):
        real_relative_pose = np.linalg.inv(real_poses[i-1]) @ real_poses[i]
        estimated_relative_pose = np.linalg.inv(estimated_poses[i-1]) @ estimated_poses[i]

        # Calculate the transformation error between the real and estimated relative poses
        relative_pose_error = np.linalg.inv(real_relative_pose) @ estimated_relative_pose

        # Extract the translation and rotation components
        translation_component = relative_pose_error[:3, 3]
        rotation_component = relative_pose_error[:3, :3]

        # Calculate the translation error
        translation_error = np.linalg.norm(translation_component)
        translation_errors.append(translation_error)

        # Calculate the rotation error as the sum of squared differences between the elements
        # rotation_error = np.sum(np.square(rotation_component - np.eye(3)))
        rotation_error = np.arccos(np.clip((np.trace(rotation_component) - 1) / 2, -1.0, 1.0))
        rotation_errors.append(np.rad2deg(rotation_error))

    translation_rmse = np.sqrt(np.mean(np.square(translation_errors)))
    rotation_rmse = np.sqrt(np.mean(np.square(rotation_errors)))

    return np.round(translation_rmse, 4), np.round(rotation_rmse, 2) 


if __name__ == "__main__":

    plotter = TrajectoryPlotter()
    index= 3  # 1 square 2 circle 3  eight
    ToCAnP_path = "BESTAnP/record{index}/atraj.csv".format(index=str(index))
    CIO_path = "CombineCIO/record{index}/atraj.csv".format(index=str(index))
    BESTAnPCIO_path = "BESTAnPCIO/record{index}/atraj.csv".format(index=str(index))
    Nonapp_path = "Nonapp/record{index}/atraj.csv".format(index=str(index))
    App_path = "App/record{index}/atraj.csv".format(index=str(index))
    real_poses1, estimated_poses_ToCAnP, coordinates_list = read_csv_file(ToCAnP_path)
    real_poses2, estimated_poses_CIO, coordinates_list = read_csv_file(CIO_path)
    real_poses3, estimated_poses_Nonapp, coordinates_list = read_csv_file(Nonapp_path)
    real_poses4, estimated_poses_App, coordinates_list = read_csv_file(App_path)
    real_poses5, estimated_poses_BESTAnPCIO, coordinates_list = read_csv_file(BESTAnPCIO_path)

    # Add the real trajectory
    print(len(real_poses1))
    start_index = 0
    plotter.add_trajectory(real_poses1[start_index:], 'Grey', 'grey', 'Ground truth')


    # # Add the estimated trajectory
    plotter.add_trajectory(estimated_poses_ToCAnP[start_index:], 'Red', 'Red', 'BESTAnP')
    plotter.add_trajectory(estimated_poses_CIO[start_index:], 'Green', 'Green', 'Combined+CIO')
    plotter.add_trajectory(estimated_poses_BESTAnPCIO[start_index:], 'Blue', 'Blue', 'BESTAnP+CIO')
    # plotter.add_trajectory(estimated_poses_Nonapp[start_index:], 'Orange', 'Orange', 'Nonap')
    # plotter.add_trajectory(estimated_poses_App[start_index:], 'Purple', 'Purple', 'App')
    # plotter.add_trajectory(estimated_poses_Nonapp, 'Green', 'Nonapp')

    # # Plot all the added trajectories
    plotter.plot_all()
