#!/usr/bin/python3
import rospy
import numpy as np
from BESTAnP.msg import SonarData  # 确保替换为你的包名
import csv

class SonarDataWriter:
    def __init__(self, data_dir="sonar_data.csv", noisy_data_dir="sonar_data_noisy.csv"):
        
        # 初始化ROS节点
        rospy.init_node('sonar_data_write', anonymous=True)
        self.data_dir = data_dir
        self.noisy_data_dir = noisy_data_dir
        
        # 定义订阅者
        self.sonar_data_sub = rospy.Subscriber('/sim/sonar_data_with_pose', SonarData, self.sonar_callback)

        
        # We need to set the frequency
        # frequency = 5/7
        frequency = 3
        self.last_callback_time = rospy.Time.now()
        self.callback_interval = rospy.Duration(1/frequency)  # Throttle to 5 Hz

        self.callback_times = 0
    
    def sonar_callback(self, data):
        current_time = rospy.Time.now()
        if current_time - self.last_callback_time >= self.callback_interval:
            self.last_callback_time = current_time
            pts_indice = np.array(data.indices)
        
            pose = data.pose
            w_p = np.array(data.w_p).reshape(-1, 3)
            s_p = np.array(data.s_p).reshape(-1, 3)
            
            si_q_xy = np.array(data.si_q_xy).reshape(-1, 2)
            si_q_theta_Rho = np.array(data.si_q_theta_Rho).reshape(-1, 2)
            si_q_xy_img_frame = np.array(data.si_q_xy_img_frame).reshape(-1, 2)
            
            si_q_xy_noise = np.array(data.si_q_xy_noise).reshape(-1, 2)
            si_q_theta_Rho_noise = np.array(data.si_q_theta_Rho_noise).reshape(-1, 2)
            si_q_xy_img_frame_noise = np.array(data.si_q_xy_img_frame_noise).reshape(-1, 2)
            
            timestep = data.timestep

            # 写入文件
            # 将数据写入CSV文件
            with open(self.data_dir, 'a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([pose.position.x, pose.position.y, pose.position.z,
                                    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w,
                                    w_p.tolist(), s_p.tolist(), si_q_xy.tolist(), si_q_theta_Rho.tolist(), si_q_xy_img_frame.tolist(),
                                    timestep, pts_indice.tolist()])
   
            with open(self.noisy_data_dir, 'a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([pose.position.x, pose.position.y, pose.position.z,
                                    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w,
                                    w_p.tolist(), s_p.tolist(), si_q_xy_noise.tolist(), si_q_theta_Rho_noise.tolist(), si_q_xy_img_frame_noise.tolist(),
                                    timestep, pts_indice.tolist()])
   
            self.callback_times += 1
            print(self.callback_times)
        
    def write(self):        
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            rate.sleep()
            # if self.callback_times > 120:
            #     break

class SonarDataReader:
    def __init__(self, filepath):
        self.filepath = filepath
        self.data = []

    def read_data(self):
        with open(self.filepath, 'r') as file:
            reader = csv.reader(file)
            # headers = next(reader)  # 跳过表头

            for row in reader:
                pose_x = float(row[0])
                pose_y = float(row[1])
                pose_z = float(row[2])
                pose_orient_x = float(row[3])
                pose_orient_y = float(row[4])
                pose_orient_z = float(row[5])
                pose_orient_w = float(row[6])

                w_p = np.array(eval(row[7]))
                s_p = np.array(eval(row[8]))
                si_q_xy = np.array(eval(row[9]))
                si_q_theta_Rho = np.array(eval(row[10]))
                si_q_xy_img_frame = np.array(eval(row[11]))
                timestep = int(row[12])
                pts_indice = np.array(eval(row[13]))

                self.data.append({
                    'pose': {
                        'position': {'x': pose_x, 'y': pose_y, 'z': pose_z},
                        'orientation': {'x': pose_orient_x, 'y': pose_orient_y, 'z': pose_orient_z, 'w': pose_orient_w}
                    },
                    'w_p': w_p,
                    's_p': s_p,
                    'si_q_xy': si_q_xy,
                    'si_q_theta_Rho': si_q_theta_Rho,
                    'si_q_xy_img_frame': si_q_xy_img_frame,
                    'timestamp': timestep,
                    'pts_indice': pts_indice
                })

    def get_data(self):
        return self.data

class SonarNoisyDataReader:
    def __init__(self, filepath):
        self.filepath = filepath
        self.data = []

    def read_data(self):
        with open(self.filepath, 'r') as file:
            reader = csv.reader(file)
            next(reader)  # 跳过表头

            for row in reader:
                # 解析真实位姿
                timestamp = float(row[0])
                true_pose_x = float(row[1])
                true_pose_y = float(row[2])
                true_pose_z = float(row[3])
                true_pose_orient_x = float(row[4])
                true_pose_orient_y = float(row[5])
                true_pose_orient_z = float(row[6])
                true_pose_orient_w = float(row[7])

                # 解析带噪声位姿
                noisy_pose_x = float(row[8])
                noisy_pose_y = float(row[9])
                noisy_pose_z = float(row[10])
                noisy_pose_orient_x = float(row[11])
                noisy_pose_orient_y = float(row[12])
                noisy_pose_orient_z = float(row[13])
                noisy_pose_orient_w = float(row[14])

                # 解析点索引和带噪声的声呐测量
                pts_indice = np.array(eval(row[15]))
                noisy_si_q_theta_Rho = np.array(eval(row[16]))
                w_p = np.array(eval(row[17]))

                # 构造与原格式一致的数据结构
                self.data.append({
                    'true_pose': {
                        'position': {'x': true_pose_x, 'y': true_pose_y, 'z': true_pose_z},
                        'orientation': {'x': true_pose_orient_x, 'y': true_pose_orient_y, 
                                      'z': true_pose_orient_z, 'w': true_pose_orient_w}
                    },
                    'noisy_pose': {
                        'position': {'x': noisy_pose_x, 'y': noisy_pose_y, 'z': noisy_pose_z},
                        'orientation': {'x': noisy_pose_orient_x, 'y': noisy_pose_orient_y, 
                                      'z': noisy_pose_orient_z, 'w': noisy_pose_orient_w}
                    },
                    'si_q_theta_Rho': noisy_si_q_theta_Rho,
                    'timestamp': timestamp,
                    'pts_indice': pts_indice,
                    'w_p': w_p
                })

    def get_data(self):
        return self.data

if __name__ == '__main__':
    import os
    from roslib.packages import get_pkg_dir
    BESTAnP_dir = get_pkg_dir('BESTAnP')
    data_dir = BESTAnP_dir + "/data/sonar_data.csv"
    noisy_data_dir = BESTAnP_dir + "/data/sonar_data_noisy.csv"
    # print(data_dir)
    # sonar_data_write = SonarDataWriter(data_dir, noisy_data_dir)
    # sonar_data_write.write()
    
    # # Write
    # filepath = "/home/clp/catkin_ws/src/BESTAnP/scripts/sim/noisy_sonar_data.csv"
    # filepath = "/home/clp/catkin_ws/src/BESTAnP/data/square/noisy_data/noisy_data_seed_145.csv"
    # reader = SonarDataReader(filepath)
    # reader.read_data()
    # data = reader.get_data()

    # # 测试打印读取的数据
    # for entry in data:
    #     print("Pose Position: ", entry['pose']['position'])
    #     print("Pose Orientation: ", entry['pose']['orientation'])
    #     # print("w_p: ", entry['w_p'])
    #     # print("s_p: ", entry['s_p'])
    #     # print("si_q: ", entry['si_q'])
    #     # print("si_q_theta_d: ", entry['si_q_theta_d'])
    #     # print("Timestep: ", entry['timestep'])
    #     # print("Pts Indice: ", entry['pts_indice'])
    #     # print("\n")
    #     break
    
    filepath = "/home/clp/catkin_ws/src/BESTAnP/data/square/noisy_data/noisy_data_seed_0_full.csv"
    reader = SonarNoisyDataReader(filepath)
    reader.read_data()
    data = reader.get_data()

    # 测试打印读取的数据
    for entry in data:
        print("Pose Position: ", entry['true_pose']['position'])
        print("Pose Orientation: ", entry['true_pose']['orientation'])
        print("Pose Position: ", entry['noisy_pose']['position'])
        print("Pose Orientation: ", entry['noisy_pose']['orientation'])
        print("si_q_theta_Rho: ", entry['si_q_theta_Rho'])
        print("Timestamp: ", entry['timestamp'])
        print("Pts Indice: ", entry['pts_indice'])
        print("\n")
        break




  