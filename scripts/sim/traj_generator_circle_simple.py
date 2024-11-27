#!/usr/bin/python3
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler

class TrajectoryGenerator:
    def __init__(self, fast_speed=0.4, slow_speed=0.2, delta_t=0.01):
        """
        初始化轨迹生成器参数
        :param fast_speed: 直线段速度
        :param slow_speed: 转弯段速度
        :param delta_t: 时间步长
        """
        self.fast_speed = fast_speed
        self.slow_speed = slow_speed
        self.delta_t = delta_t
        self.t = 0.0

        self.side_length = 8  # 正方形边长
        self.corner_radius = 4  # 角落圆弧半径
        
        self.straight_length = self.side_length - 2 * self.corner_radius
        self.quarter_circle = np.pi * self.corner_radius / 2
        self.total_length = 4 * (self.straight_length + self.quarter_circle)
        self.straight_time = self.straight_length / self.fast_speed
        self.corner_time = self.quarter_circle / self.slow_speed
        self.period = 4 * (self.straight_time + self.corner_time)  # 完成一圈所需的时间
        
        # 初始化ROS节点和发布器
        rospy.init_node('sonar_pose_publisher', anonymous=True)
        self.pub = rospy.Publisher('/set_sonar_pose', PoseStamped, queue_size=10)
        self.rate_hz = rospy.Rate(100)  # 10 Hz

    def generate_trajectory(self, t):
        """
        生成正方形轨迹，角落为1/4圆弧，并计算 roll, pitch, yaw
        :param t: 时间
        :return: (x, y, z, roll, pitch, yaw)
        """
        
        t = t % self.period  # 确保t在一个周期内

        # 确定当前在哪个段
        segment_time = self.straight_time + self.corner_time
        segment = int(t // segment_time)
        segment_t = t % segment_time

        if segment_t < self.straight_time:
            # 直线段
            progress = self.fast_speed * segment_t
            if segment == 0:
                x = progress + self.corner_radius
                y = self.corner_radius
                yaw = 0
            elif segment == 1:
                x = self.side_length
                y = - progress
                # y = -(progress - self.straight_length - self.quarter_circle) - self.side_length
                yaw = -np.pi/2
            elif segment == 2:
                x = self.straight_length + self.corner_radius - progress
                # x = self.corner_radius - progress +3 * self.straight_length + 2 * self.quarter_circle
                y = -self.straight_length - self.corner_radius
                yaw = -np.pi
            else:  # segment == 3
                x = 0
                y = -self.straight_length + progress
                # y = -(4*self.straight_length - progress + 3*self.quarter_circle)
                yaw = -3*np.pi/2
        else:
            # 转弯段
            angle = self.slow_speed * (segment_t - self.straight_time) / self.corner_radius
            if segment == 0:
                x = self.side_length - self.corner_radius + self.corner_radius * np.sin(angle)
                y = self.corner_radius * np.cos(angle)
                yaw = -angle
            elif segment == 1:
                x = self.side_length - self.corner_radius + self.corner_radius * np.cos(angle)
                y = -self.straight_length - self.corner_radius * np.sin(angle)
                yaw = -(np.pi/2 + angle)
            elif segment == 2:
                x = self.corner_radius - self.corner_radius * np.sin(angle)
                y = -self.straight_length - self.corner_radius * np.cos(angle)
                yaw = -(np.pi + angle)
            else:  # segment == 3
                x = self.corner_radius * (1-np.cos(angle))
                y = self.corner_radius * np.sin(angle)
                yaw = -(1.5*np.pi + angle)

        z = 0.5 + 0.1 * np.sin(2 * np.pi * t / self.period)
        roll = 0.1 * np.sin(20 * np.pi * t / self.period)
        pitch = 0.1 * np.cos(20 * np.pi * t / self.period)
        # posit = np.array([x,y])
        point = np.array([self.side_length/2, 0])
        # print(np.linalg.norm(posit-point))
        # print(y)
        yaw = np.pi+np.arctan2( (y-point[1]), (x-point[0]))
        # yaw = np.where(yaw < 0, yaw + np.pi, yaw)
        # print('{:.2g}, {:.2g}'.format(x, y))

        return x, y, z, roll, pitch, yaw

    def publish_pose(self):
        while not rospy.is_shutdown():
            # 生成当前位置和姿态
            x, y, z, roll, pitch, yaw = self.generate_trajectory(self.t)
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
        traj_gen = TrajectoryGenerator(fast_speed=1, slow_speed=0.5, delta_t=0.01)
        traj_gen.publish_pose()
    except rospy.ROSInterruptException:
        pass