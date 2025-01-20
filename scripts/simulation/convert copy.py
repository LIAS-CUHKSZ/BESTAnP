#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image, Imu
from nav_msgs.msg import Odometry
from uuv_sensor_ros_plugins_msgs.msg import DVL
from geometry_msgs.msg import TwistWithCovarianceStamped, PoseStamped
import time

class FrequencyMonitor:
    def __init__(self):
        rospy.init_node('frequency_monitor', anonymous=True)
        self.last_time = None
        self.frequencies = []
        # self.subscriber = rospy.Subscriber('/blueview_p900_ray/sonar_image', Image, self.callback)
        # self.subscriber = rospy.Subscriber('/rexrov/rexrov/cameraright/camera_image', Image, self.callback)
        
        # self.subscriber = rospy.Subscriber('/rexrov/imu', Imu, self.callback)
        self.subscriber = rospy.Subscriber('/rexrov/pose_gt', Odometry, self.callback)
        # self.subscriber = rospy.Subscriber('/rexrov/dvl_twist', TwistWithCovarianceStamped, self.callback)
        # self.subscriber = rospy.Subscriber('/charuco_pose', PoseStamped, self.callback)
        
        # 创建日志文件,使用时间戳命名
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        self.log_file = open(f'frequency_log_{timestamp}.txt', 'w')
        self.log_file.write("Time,Frequency(Hz)\n")

    def callback(self, msg: Odometry):
        current_time = msg.header.stamp.to_sec()
        
        if self.last_time is not None:
            # 计算时间差并转换为频率
            time_diff = current_time - self.last_time
            freq = 1.0 / time_diff
            rospy.loginfo(f"Current frequency: {freq:.2f} Hz, time_diff: {time_diff:.4f} s")
            # rospy.loginfo(f"Data: {2f} Hz, time_diff: {time_diff:.4f} s")
        self.last_time = current_time
        
        pose = msg.pose.pose
                
        # 打印或处理pose信息
        print("Position: x={:.2f}, y={:.2f}, z={:.2f}".format(
            pose.position.x, 
            pose.position.y, 
            pose.position.z
        ))
        print("Orientation: x={:.2f}, y={:.2f}, z={:.2f}, w={:.2f}".format(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        ))

    def shutdown(self):
        self.log_file.close()
        rospy.loginfo("Shutting down frequency monitor...")

if __name__ == '__main__':
    try:
        monitor = FrequencyMonitor()
        rospy.on_shutdown(monitor.shutdown)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass