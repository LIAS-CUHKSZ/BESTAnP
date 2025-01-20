#!/usr/bin/python3

import sys, roslib, os
project_root = roslib.packages.get_pkg_dir('BESTAnP')
root_dir = os.path.abspath(os.path.join(project_root, 'scripts'))
sys.path.append(root_dir)

import cv2
import numpy as np
# from tri import sonar_triangulation
# from anp import AnPAlgorithm
import random
random.seed(2)
import rospy
from cv_bridge import CvBridge
from nav_msgs.msg import Path
import copy

from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Twist
from visualization_msgs.msg import Marker
from BESTAnP.msg import SonarData  # 确保替换为你的包名
from std_msgs.msg import Header
import tf

from scipy.spatial.transform import Rotation as R
from utils.pose2matrix import pose_to_transform_matrix, ros_pose_to_transform_matrix

import yaml
import os
import rospkg

from utils.pose2matrix import pose_to_transform_matrix
from utils.coordinate_system_transform import coordinate_transform_Pose


class Anp_sim:
    def __init__(self, yaml_file_path):
        
        with open(yaml_file_path, 'r') as file:
            params = yaml.safe_load(file)
            
        points_params = params['points_parameters']
        sonar_attr = params['sonar_attribute']
        sonar_img = params['sonar_image']
        sonar_ctrl_mode = params['sonar_control_mode']
        
        self.points_rviz = []
        self.xmin = points_params['xmin']
        self.xmax = points_params['xmax']
        self.ymin = points_params['ymin']
        self.ymax = points_params['ymax']
        self.zmin = points_params['zmin']
        self.zmax = points_params['zmax']
        
        sonar_noise_params = params['sonar_noise']
        self.Rho_noise = sonar_noise_params['Rho_noise']
        self.theta_noise = sonar_noise_params['theta_noise']
        
        for _ in range(points_params['pts_num']):
            x = random.uniform(self.xmin, self.xmax)
            y = random.uniform(self.ymin, self.ymax)
            z = random.uniform(self.zmin, self.zmax)
            self.points_rviz.append(Point(x, y, z))
        # self.points_rviz.append(Point(2, 0, 0))
        
        self.points = np.array([[point.x, point.y, point.z] for point in self.points_rviz])

        self.sonar_image = None
        self.pose = {'position': { 'x': 0.0, 'y': 0.0, 'z': 0.0,}, 'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0,} }
        self.pose_T = ros_pose_to_transform_matrix(self.pose)
        present_pose = copy.deepcopy(self.pose)
        self.trajectory = [present_pose]
        
        self.estimated_pose = None
        
        # visualize
        self.img_match = None
        self.bridge = CvBridge()

        self.sonar_image_pub = rospy.Publisher('/sim/sonar_image', Image, queue_size=10)
        self.sonar_data_pub = rospy.Publisher('/sim/sonar_data_with_pose', SonarData, queue_size=10)

        self.sonar_marker_pub = rospy.Publisher('/rviz/sonar_view', Marker, queue_size=10)
        self.marker_pub = rospy.Publisher('/rviz/visualization_pts', Marker, queue_size=10)
        self.marker_pub2 = rospy.Publisher('/rviz/visualization_fov_pts', Marker, queue_size=10)
        self.traj_gt_pub = rospy.Publisher("/rviz/trajectory_gt", Path, queue_size=10)
        
        if sonar_ctrl_mode == 0:
            self.cmd_vel_sub = rospy.Subscriber('/joy/cmd_vel', Twist, self.remoter_callback)
        elif  sonar_ctrl_mode == 1:
            self.sonar_pose_sub = rospy.Subscriber('/set_sonar_pose', PoseStamped, self.set_pose_callback)


        # Sonar
        # Define the sonar's field of view as a fan shape with top and bottom faces
        self.fov_horizontal = np.deg2rad(sonar_attr['fov_horizontal'])  # 90 degrees horizontal field of view
        self.fov_vertical = np.deg2rad(sonar_attr['fov_vertical'])  # 60 degrees vertical field of view
        self.range_max = sonar_attr['range_max']  # 5 meters range
        # Sonar image
        self.img_width, self.img_height = sonar_img['img_width'], sonar_img['img_height']
        self.s_p = None
        self.w_p = None
        self.si_q = None
        self.si_q_theta_Rho = None
        
        self.__timestep = 0
        
        self.__rviz_init()
    
    def remoter_callback(self, msg):
        dt = 1.0 / 100.0  # Assuming the loop runs at 10 Hz

        # Create the translation vector from the linear velocities
        translation = np.array([msg.linear.x * dt, msg.linear.y * dt, msg.linear.z * dt])

        # Create the rotation matrix from the angular velocities
        delta_rotation_matrix = tf.transformations.euler_matrix(
            msg.angular.x * dt,
            msg.angular.y * dt,
            msg.angular.z * dt
        )

        # Combine translation and rotation into a transformation matrix
        delta_T_robot = tf.transformations.compose_matrix(translate=translation) @ delta_rotation_matrix
        T_world = self.pose_T @ delta_T_robot
        
        # 限制平移部分在指定边界内
        T_world_t = T_world[:3, 3]
        # T_world_t[0] = np.clip(T_world_t[0], -2, 2)
        # T_world_t[1] = np.clip(T_world_t[1], -2, 2)
        # T_world_t[2] = np.clip(T_world_t[2], self.zmin, self.zmax)
        # T_world_t[0] = np.clip(T_world_t[0], self.xmin, self.xmax)
        # T_world_t[1] = np.clip(T_world_t[1], self.ymin, self.ymax)
        # T_world_t[2] = np.clip(T_world_t[2], self.zmin, self.zmax)
        # 将调整后的平移部分放回 T_world 矩阵中
        T_world[:3, 3] = T_world_t
        
        # Update the pose transformation matrix
        self.pose_T = T_world

        # Extract the updated position and orientation from the transformation matrix
        updated_translation = tf.transformations.translation_from_matrix(self.pose_T)
        updated_orientation = tf.transformations.quaternion_from_matrix(self.pose_T)

        # Update the pose dictionary
        self.pose['position']['x'] = updated_translation[0]
        self.pose['position']['y'] = updated_translation[1]
        self.pose['position']['z'] = updated_translation[2]
        self.pose['orientation']['x'] = updated_orientation[0]
        self.pose['orientation']['y'] = updated_orientation[1]
        self.pose['orientation']['z'] = updated_orientation[2]
        self.pose['orientation']['w'] = updated_orientation[3]
        
        present_pose = copy.deepcopy(self.pose)
        self.trajectory.append(present_pose)
        # print("remoter_callback", len(self.trajectory))
        
    def set_pose_callback(self, msg):

        # Create the translation vector from the linear velocities
        self.pose_T = pose_to_transform_matrix(msg.pose)

        # Extract the updated position and orientation from the transformation matrix
        updated_translation = tf.transformations.translation_from_matrix(self.pose_T)
        updated_orientation = tf.transformations.quaternion_from_matrix(self.pose_T)

        # Update the pose dictionary
        self.pose['position']['x'] = updated_translation[0]
        self.pose['position']['y'] = updated_translation[1]
        self.pose['position']['z'] = updated_translation[2]
        self.pose['orientation']['x'] = updated_orientation[0]
        self.pose['orientation']['y'] = updated_orientation[1]
        self.pose['orientation']['z'] = updated_orientation[2]
        self.pose['orientation']['w'] = updated_orientation[3]
        
        present_pose = copy.deepcopy(self.pose)
        self.trajectory.append(present_pose)
  
    def __points_in_fov(self):
        """
        Determine which points are within the field of view (FOV) of a sonar.

        Returns:
        np.ndarray: A boolean array of length N indicating which points are within the FOV.
        """
        sonar_position = np.array([self.pose['position']['x'], self.pose['position']['y'], self.pose['position']['z']])
        sonar_orientation = np.array([self.pose['orientation']['x'], self.pose['orientation']['y'], self.pose['orientation']['z'], self.pose['orientation']['w']])
        
        # Convert points to the sonar's coordinate frame
        points_relative = self.points - sonar_position
        r = R.from_quat(sonar_orientation)
        points_in_sonar_frame = r.inv().apply(points_relative)

        # Calculate angles and distances in the sonar's coordinate frame
        x, y, z = points_in_sonar_frame[:, 0], points_in_sonar_frame[:, 1], points_in_sonar_frame[:, 2]
        distances = np.sqrt(x**2 + y**2 + z**2)
        horizontal_angles = np.arctan2(y, x)
        vertical_angles = np.arctan2(z, np.sqrt(x**2 + y**2))

        # Check if self.points are within the FOV and range
        within_horizontal_fov = np.abs(horizontal_angles) <= (self.fov_horizontal / 2)
        within_vertical_fov = np.abs(vertical_angles) <= (self.fov_vertical / 2)
        within_range = distances <= self.range_max
        
        pts_in_fov_index = within_horizontal_fov & within_vertical_fov & within_range
        
        return pts_in_fov_index, self.pose
    
    def publish_sonar_image_and_data(self):
        # Get points that are within the FOV
        pts_in_fov_index, pose = self.__points_in_fov() # get index of pts in Fov stored in self.__pts_in_fov
        points_in_fov = self.points[pts_in_fov_index]        
        self.pts_in_fov_index = pts_in_fov_index
        # Convert points to the sonar's coordinate frame
        sonar_position = np.array([self.pose['position']['x'], self.pose['position']['y'], self.pose['position']['z']])
        sonar_orientation = np.array([self.pose['orientation']['x'], self.pose['orientation']['y'], self.pose['orientation']['z'], self.pose['orientation']['w']])
        points_relative = points_in_fov - sonar_position
        r = R.from_quat(sonar_orientation)
        points_in_sonar_frame = r.inv().apply(points_relative)

        # Initialize a blank image
        image = np.ones((self.img_height, self.img_width,3), dtype=np.uint8) * 255
        center = (int(self.img_width/2), 0)
        radius = int(self.img_height)
        start_angle = -int(np.rad2deg(self.fov_horizontal)/2)  # 对称轴为中轴，扇形从-60度到60度
        end_angle = int(np.rad2deg(self.fov_horizontal)/2)
        cv2.ellipse(image, center, (radius, radius), 90, start_angle, end_angle, (0, 0, 0), -1)

        # Convert points to polar coordinates and map to image coordinates
        ## Here is the ground truth
        X, Y, Z = points_in_sonar_frame[:, 0], points_in_sonar_frame[:, 1], points_in_sonar_frame[:, 2]
        
        Rho = np.sqrt(X**2 + Y**2 + Z**2)
        theta = np.arctan(Y/X)

        ps_x = Rho * np.cos(theta)
        ps_y = Rho * np.sin(theta)
        
        si_q_xy = []
        si_q_theta_Rho = []
        si_q_xy_img_frame = []
        for y,x, theta_i, Rho_i in zip(ps_x, ps_y, theta, Rho):
            # Normalize to image coordinates
            x_img = int((x / self.range_max) * (self.img_width) + (self.img_width/2))
            y_img = int((y / self.range_max) * (self.img_height) )
            # cv2.circle(image, (x_img, y_img), 3, (255, 255, 255), -1)
            si_q_xy.append(np.array([x, y]))
            si_q_theta_Rho.append(np.array([theta_i, Rho_i]))
            si_q_xy_img_frame.append(np.array([self.img_width/2-x_img, y_img]))
        
        ####################################
        ### Now we can add noise
        ####################################
        theta_noise = np.arctan(Y/X + np.random.normal(0, self.theta_noise, size=Y.shape)) # arctan(tan(theta)+noise)
        Rho_noise = np.sqrt(X**2 + Y**2 + Z**2) + np.random.normal(0, self.Rho_noise, size=Y.shape) 
        ps_x_noise = Rho_noise * np.cos(theta_noise)
        ps_y_noise = Rho_noise * np.sin(theta_noise)
        si_q_xy_noise = []
        si_q_theta_Rho_noise = []
        si_q_xy_img_frame_noise = []
        for y, x, theta_i, Rho_i in zip(ps_x_noise, ps_y_noise, theta_noise, Rho_noise):
            x_img = int((x / self.range_max) * (self.img_width) + (self.img_width/2))
            y_img = int((y / self.range_max) * (self.img_height) )
            cv2.circle(image, (x_img, y_img), 3, (255, 255, 255), -1)
            # print((x_img, y_img))
            si_q_xy_noise.append(np.array([x, y]))
            si_q_theta_Rho_noise.append(np.array([theta_i, Rho_i]))
            si_q_xy_img_frame_noise.append(np.array([self.img_width/2-x_img, y_img]))
                
        # Convert to ROS Image message and publish
        # cv2.imshow("test", image)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
        
        ros_image = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
        self.sonar_image_pub.publish(ros_image)
        
        self.w_p = points_in_fov
        self.s_p = points_in_sonar_frame
        
        # Publish SonarData with pose
        sonar_data_msg = SonarData()
        sonar_data_msg.header = Header()
        sonar_data_msg.header.stamp = rospy.Time.now()
        sonar_data_msg.indices = np.where(pts_in_fov_index)[0].tolist()
        sonar_data_msg.w_p = self.w_p.flatten()
        sonar_data_msg.s_p = self.s_p.flatten()
        
        sonar_data_msg.si_q_xy = np.array(si_q_xy).flatten()
        sonar_data_msg.si_q_theta_Rho = np.array(si_q_theta_Rho).flatten()
        sonar_data_msg.si_q_xy_img_frame = np.array(si_q_xy_img_frame).flatten()
        
        sonar_data_msg.si_q_xy_noise = np.array(si_q_xy_noise).flatten()
        sonar_data_msg.si_q_theta_Rho_noise = np.array(si_q_theta_Rho_noise).flatten()
        sonar_data_msg.si_q_xy_img_frame_noise = np.array(si_q_xy_img_frame_noise).flatten()
    
        
        sonar_data_msg.timestep = self.__timestep
        sonar_data_msg.pose.position.x = pose['position']['x']
        sonar_data_msg.pose.position.y = pose['position']['y']
        sonar_data_msg.pose.position.z = pose['position']['z']
        sonar_data_msg.pose.orientation.x = pose['orientation']['x']
        sonar_data_msg.pose.orientation.y = pose['orientation']['y']
        sonar_data_msg.pose.orientation.z = pose['orientation']['z']
        sonar_data_msg.pose.orientation.w = pose['orientation']['w']
        # indices = np.where(pts_in_fov_index)[0]
        # print(indices)

        self.sonar_data_pub.publish(sonar_data_msg)

    def main_process(self, step=1):        
        rospy.init_node('anp_sim')
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            self.publish_sonar_image_and_data()
            self.__visualize()
            self.__timestep += 1
            rate.sleep()
            
    def __visualize(self):
        self.__publish_points()
        self.__publish_fov_pts()
        self.__publish_sonar_view()
        self.__publish_traj_gt()
        return
    
    def __rviz_init(self):
        self.marker = Marker()
        self.marker.header.frame_id = "map"
        self.marker.ns = "sonar"
        self.marker.id = 0
        self.marker.type = Marker.TRIANGLE_LIST
        self.marker.action = Marker.ADD
        self.marker.scale.x = 1.0
        self.marker.scale.y = 1.0
        self.marker.scale.z = 1.0
        self.marker.color.a = 0.3  # Transparency
        self.marker.color.r = 0.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0

        # Define the vertices of the fan shape
        origin = Point(0, 0, 0)
        angle_step = np.deg2rad(5)  # Angle step for the fan shape

        # Vertices for the bottom and top fan shapes
        bottom_points = []
        top_points = []

        angle_min = -self.fov_horizontal / 2
        angle_max = self.fov_horizontal / 2
        angle_increment = (angle_max - angle_min) / (self.fov_horizontal / angle_step)

        for i in range(int(self.fov_horizontal / angle_step) + 1):
            angle = angle_min + i * angle_increment
            x = self.range_max * np.cos(angle)
            y = self.range_max * np.sin(angle)
            z_bottom = -self.range_max * np.tan(self.fov_vertical / 2)
            z_top = self.range_max * np.tan(self.fov_vertical / 2)
            bottom_points.append(Point(x, y, z_bottom))
            top_points.append(Point(x, y, z_top))

        # Create triangles for the bottom and top fan shapes and connect them
        for i in range(len(bottom_points) - 1):
            # Bottom fan triangles
            self.marker.points.append(origin)
            self.marker.points.append(bottom_points[i])
            self.marker.points.append(bottom_points[i + 1])

            # Top fan triangles
            self.marker.points.append(origin)
            self.marker.points.append(top_points[i])
            self.marker.points.append(top_points[i + 1])

            # Side triangles
            # First triangle
            self.marker.points.append(bottom_points[i])
            self.marker.points.append(top_points[i])
            self.marker.points.append(bottom_points[i + 1])

            # Second triangle
            self.marker.points.append(bottom_points[i + 1])
            self.marker.points.append(top_points[i])
            self.marker.points.append(top_points[i + 1])

    def __publish_sonar_view(self):
        """
        publish sonar in rviz       
        """
        self.marker.header.stamp = rospy.Time.now()
        self.marker.pose.position = Point(
            self.pose['position']['x'],
            self.pose['position']['y'],
            self.pose['position']['z']
        )
        self.marker.pose.orientation = Quaternion(
            self.pose['orientation']['x'],
            self.pose['orientation']['y'],
            self.pose['orientation']['z'],
            self.pose['orientation']['w']
        )
        self.sonar_marker_pub.publish(self.marker)
    
    def __publish_points(self):
        """
        publish points in rviz       
        """
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "points"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.02
        marker.scale.y = 0.02

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        for point in self.points_rviz:
            marker.points.append(point)

        self.marker_pub.publish(marker)
    
    def __publish_fov_pts(self):
        """
        publish points in rviz       
        """
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "points"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.02
        marker.scale.y = 0.02

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        
        for point in self.points[self.pts_in_fov_index]  :
            marker.points.append(Point(point[0], point[1], point[2]+0.01))

        self.marker_pub2.publish(marker)
    
    def __publish_traj_gt(self):
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "map"

        current_time = rospy.Time.now()
        for i, pose in enumerate(self.trajectory):
            pose_stamped = PoseStamped()
            # 使用递增的时间戳
            pose_stamped.header.stamp = current_time + rospy.Duration(i * 0.1)  # 每个姿态相隔0.1秒
            pose_stamped.header.frame_id = "map"
            pose_stamped.pose.position.x = pose['position']['x']
            pose_stamped.pose.position.y = pose['position']['y']
            pose_stamped.pose.position.z = pose['position']['z']
            pose_stamped.pose.orientation.x = pose['orientation']['x']
            pose_stamped.pose.orientation.y = pose['orientation']['y']
            pose_stamped.pose.orientation.z = pose['orientation']['z']
            pose_stamped.pose.orientation.w = pose['orientation']['w']

            path_msg.poses.append(pose_stamped)
        self.traj_gt_pub.publish(path_msg)

    
    

if __name__ == '__main__':
    rospack = rospkg.RosPack()
    package_path = rospack.get_path('BESTAnP')
    yaml_file_path = os.path.join(package_path, 'yaml/sim_env.yaml')
    # yaml_file_path = os.path.join(package_path, 'yaml/sim_env_simple.yaml')
    
    # manual_ctr = True
    # if len(sys.argv) != 2 or sys.argv[1] not in ['-m', '-a']:
    #     print("Usage: rosrun BESTAnP simulator.py -m/-a (manual control by default)")
    # elif sys.argv[1] != '-m':
    #     manual_ctr = False
    estimator = Anp_sim(yaml_file_path)
    estimator.main_process()
