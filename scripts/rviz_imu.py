#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
import random

def publish_imu_data():
    pub = rospy.Publisher('/imu/data', Imu, queue_size=10)
    rospy.init_node('imu_publisher')
    rate = rospy.Rate(10) # 10 Hz

    imu = Imu()
    imu.header.frame_id = 'imu_link'

    while not rospy.is_shutdown():
        imu.header.stamp = rospy.Time.now()
        imu.orientation_covariance[0] = -1
        imu.angular_velocity.x = random.uniform(-0.1, 0.1)
        imu.angular_velocity.y = random.uniform(-0.1, 0.1)
        imu.angular_velocity.z = random.uniform(-0.1, 0.1)
        imu.linear_acceleration.x = random.uniform(-1.0, 1.0)
        imu.linear_acceleration.y = random.uniform(-1.0, 1.0)
        imu.linear_acceleration.z = random.uniform(-1.0, 1.0)
        pub.publish(imu)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_imu_data()
    except rospy.ROSInterruptException:
        pass
