#!/usr/bin/python3
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler

class TrajectoryGenerator:
    def __init__(self, rate=1.0):
        """
        初始化轨迹生成器参数
        :param rate: 轨迹速度
        :param delta_t: 时间步长
        """
        self.a = 3.0  # X轴振幅
        self.b = 2.0  # Y轴振幅
        self.c = 0.5  # Z轴振幅
        self.rate = rate
        self.delta_t = 1
        self.t = 0

        # 初始化ROS节点和发布器
        rospy.init_node('sonar_pose_publisher', anonymous=True)
        self.pub = rospy.Publisher('/set_sonar_pose', PoseStamped, queue_size=10)
        self.rate_hz = rospy.Rate(40)  # 10 Hz

    def parametric_function(self, t):
        """
        参数化函数，生成8字形轨迹
        :param t: 时间参数
        :return: (x, y, z) 坐标
        """
        theta = self.rate * t
        x = self.a * np.sin(theta)
        y = self.b * np.sin(theta) * np.cos(theta)
        z = self.c * np.sin(theta)
        # z = 0
        return x, y, z

    def generate_pose(self, t):
        """
        生成8字形轨迹, 并计算 roll, pitch, yaw
        :param t: 时间
        :return: (x, y, z, roll, pitch, yaw)
        """
        # 当前时刻的位置
        x, y, z = self.parametric_function(t)
        
        # 计算下一时刻的位置，用于计算切线方向
        x_next, y_next, z_next = self.parametric_function(t + self.delta_t)
        
        # # 计算切线方向
        # dx = x_next - x
        # dy = y_next - y
        # dz = z_next - z
        
        # # 归一化切线向量
        # magnitude = np.sqrt(dx**2 + dy**2 + dz**2)
        # tangent_x = dx / magnitude
        # tangent_y = dy / magnitude
        # tangent_z = dz / magnitude

        # # 计算 roll, pitch, yaw
        # roll = 0  # 假设 roll 为 0
        # pitch = -np.arctan2(tangent_z, np.sqrt(tangent_x**2 + tangent_y**2))  # 计算俯仰角 pitch
        # yaw = np.arctan2(tangent_y, tangent_x)  # 计算航向角 yaw
        roll = 0.2 * np.sin(0.02 * t)
        pitch = 0.1 * np.cos(0.02 * t)
        point = [0, 4]
        yaw = np.arctan2( (y-point[1]), x-point[0])
        yaw = np.where(yaw < 0, yaw + np.pi, yaw)

        deg = yaw * 180/np.pi
        # print(deg)
        # print(x,y)
        return x, y, z, roll, pitch, yaw

    def publish_pose(self):
        while not rospy.is_shutdown():
            # 生成当前位置和姿态
            x, y, z, roll, pitch, yaw = self.generate_pose(self.t)
            self.t += self.delta_t  # 时间递增

            # 创建并发布姿态消息
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "world"

            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z

            quaternion = quaternion_from_euler(roll, pitch, yaw)
            pose.pose.orientation.x = quaternion[0]
            pose.pose.orientation.y = quaternion[1]
            pose.pose.orientation.z = quaternion[2]
            pose.pose.orientation.w = quaternion[3]

            self.pub.publish(pose)
            self.rate_hz.sleep()

if __name__ == '__main__':
    try:
        # 初始化轨迹生成器，参数可以在这里修改
        traj_gen = TrajectoryGenerator(rate=0.003)
        traj_gen.publish_pose()
    except rospy.ROSInterruptException:
        pass