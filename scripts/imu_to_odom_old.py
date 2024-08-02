#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Twist, Point
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math

class ImuOdomPublisher:
    def __init__(self):
        rospy.init_node('imu_odom_publisher', anonymous=True)
        
        self.imu_sub = rospy.Subscriber('/imu', Imu, self.imu_callback)
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)

        self.current_pose = Odometry()
        self.velocity = Twist()
        self.last_time = rospy.Time.now()
        self.orientation = Quaternion()

        rospy.spin()

    def imu_callback(self, data):
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        self.last_time = current_time

        # IMU 데이터를 이용하여 속도 계산
        ax = data.linear_acceleration.x
        ay = data.linear_acceleration.y
        az = data.linear_acceleration.z - 9.81  # 중력 보정
        
        # 속도 업데이트
        self.velocity.linear.x += ax * dt
        self.velocity.linear.y += ay * dt
        self.velocity.linear.z += az * dt

        # 자세 업데이트
        self.orientation = data.orientation
        quaternion = [self.orientation.x, self.orientation.y, self.orientation.z, self.orientation.w]
        roll, pitch, yaw = euler_from_quaternion(quaternion)

        # 위치 업데이트
        self.current_pose.pose.pose.position.x += self.velocity.linear.x * dt
        self.current_pose.pose.pose.position.y += self.velocity.linear.y * dt
        self.current_pose.pose.pose.position.z += self.velocity.linear.z * dt
        self.current_pose.pose.pose.orientation = Quaternion(*quaternion_from_euler(roll, pitch, yaw))

        # 오도메트리 메시지 설정
        self.current_pose.header.stamp = current_time
        self.current_pose.header.frame_id = "odom"
        self.current_pose.child_frame_id = "base_link"
        self.current_pose.twist.twist = self.velocity

        # 퍼블리시
        self.odom_pub.publish(self.current_pose)

if __name__ == '__main__':
    try:
        ImuOdomPublisher()
    except rospy.ROSInterruptException:
        pass
