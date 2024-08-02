#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
import numpy as np

def publish_lidar_data():
    pub = rospy.Publisher('/scan', LaserScan, queue_size=10)
    rospy.init_node('lidar_publisher')
    rate = rospy.Rate(10) # 10 Hz

    scan = LaserScan()
    scan.header.frame_id = 'lidar_link'
    scan.angle_min = -np.pi
    scan.angle_max = np.pi
    scan.angle_increment = np.pi / 180
    scan.range_min = 0.0
    scan.range_max = 10.0
    num_readings = int((scan.angle_max - scan.angle_min) / scan.angle_increment)
    scan.ranges = [1.0] * num_readings

    while not rospy.is_shutdown():
        scan.header.stamp = rospy.Time.now()
        pub.publish(scan)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_lidar_data()
    except rospy.ROSInterruptException:
        pass
