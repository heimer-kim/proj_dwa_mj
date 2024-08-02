#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

# 목표 지점 설정
goal_latitude = -0.004651025267134353
goal_longitude = -0.004616987472891559
goal_altitude = 0.21024732291698456

class GpsNavigator:
    def __init__(self):
        rospy.init_node('gps_navigator', anonymous=True)
        
        self.current_pose = None
        self.goal_pose = PoseStamped()
        
        # 목표 지점 설정
        self.goal_pose.pose.position.x = goal_latitude
        self.goal_pose.pose.position.y = goal_longitude
        self.goal_pose.pose.position.z = goal_altitude

        # GPS 데이터 구독
        rospy.Subscriber('/gps', NavSatFix, self.gps_callback)
        
        # 경로 퍼블리셔
        self.path_pub = rospy.Publisher('/planned_path', Path, queue_size=10)
        
        rospy.spin()

    def gps_callback(self, data):
        self.current_pose = data
        
        # 현재 위치와 목표 위치를 이용해 전역 경로 생성
        self.plan_path()

    def plan_path(self):
        if self.current_pose is None:
            return

        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = rospy.Time.now()
        
        # 시작점 추가
        start_pose = PoseStamped()
        start_pose.pose.position.x = self.current_pose.latitude
        start_pose.pose.position.y = self.current_pose.longitude
        start_pose.pose.position.z = self.current_pose.altitude
        path.poses.append(start_pose)

        # 목표점 추가
        goal_pose = PoseStamped()
        goal_pose.pose.position.x = goal_latitude
        goal_pose.pose.position.y = goal_longitude
        goal_pose.pose.position.z = goal_altitude
        path.poses.append(goal_pose)
        
        # 경로 퍼블리싱
        self.path_pub.publish(path)
        
if __name__ == '__main__':
    try:
        GpsNavigator()
    except rospy.ROSInterruptException:
        pass
