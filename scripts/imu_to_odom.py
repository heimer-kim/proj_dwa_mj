#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Twist, Point
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
from morai_msgs.msg import EgoVehicleStatus
from message_filters import Subscriber, ApproximateTimeSynchronizer
class ImuOdomPublisher:
    def __init__(self):
        rospy.init_node('imu_odom_publisher', anonymous=True)
        

        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)


        self.imu_sub = Subscriber('/imu', Imu)
        self.Ego_status_sub = Subscriber('/Ego_topic', EgoVehicleStatus)
        self.ts = ApproximateTimeSynchronizer(
            [self.imu_sub, self.Ego_status_sub], 
            queue_size=10, 
            slop=0.1,  # 더 엄격한 시간 허용 범위 설정
            allow_headerless=True
        )
        self.ts.registerCallback(self.callback)
        self.current_pose = Odometry()
        self.velocity = Twist()
        self.last_time = rospy.Time.now()
        self.orientation = Quaternion()

        rospy.spin()

    def callback(self, imu_data, ego_data):
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        self.last_time = current_time


        
        # 속도 업데이트
        self.velocity.linear.x = ego_data.velocity.x
        print(ego_data.velocity.x)

        self.current_pose.twist.twist.angular = imu_data.angular_velocity
        # 자세 업데이트
        self.orientation = imu_data.orientation
        quaternion = [self.orientation.x, self.orientation.y, self.orientation.z, self.orientation.w]
        roll, pitch, yaw = euler_from_quaternion(quaternion)


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
