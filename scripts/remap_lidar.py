#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import LaserScan

class LidarRemapper:
    def __init__(self):
        rospy.init_node('lidar_remapper', anonymous=True)
        
        # 구독 설정: /lidar2D 토픽 구독
        rospy.Subscriber('/lidar2D', LaserScan, self.lidar_callback)
        
        # 발행 설정: /scan 토픽 발행
        self.scan_pub = rospy.Publisher('/scan', LaserScan, queue_size=10)

    def lidar_callback(self, data):
        # 수신한 데이터를 그대로 /scan 토픽으로 발행
        self.scan_pub.publish(data)

if __name__ == '__main__':
    try:
        LidarRemapper()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
