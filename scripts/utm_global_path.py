#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import math
import threading

class GpsToGlobalPath:
    def __init__(self, input_file, threshold=0.5):
        rospy.init_node('gps_to_global_path', anonymous=True)
        
        self.input_file = input_file
        self.path_pub = rospy.Publisher('/path', Path, queue_size=10)
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.global_path = Path()
        self.global_path.header = Header(frame_id="map")
        self.threshold = threshold
        self.current_goal_index = 0
        self.goals = []
        
        rospy.loginfo("Initializing GpsToGlobalPath")
        self.create_global_path()

        # Subscribe to the robot's current position (odometry)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

        rospy.loginfo("Starting to publish path and goals")
        # Start publishing path and goal
        path_thread = threading.Thread(target=self.publish_path)
        goal_thread = threading.Thread(target=self.publish_goal_periodically)
        path_thread.start()
        goal_thread.start()

    def create_global_path(self):
        try:
            with open(self.input_file, 'r') as file:
                lines = file.readlines()
                i = 0
                while i < len(lines):
                    if lines[i].startswith("Timestamp:"):
                        timestamp = lines[i].strip().split(": ")[1]
                        utm_easting = float(lines[i+1].strip().split(": ")[1])
                        utm_northing = float(lines[i+2].strip().split(": ")[1])
                        utm_zone_number = int(lines[i+3].strip().split(": ")[1])
                        utm_zone_letter = lines[i+4].strip().split(": ")[1]
                        altitude = float(lines[i+5].strip().split(": ")[1])

                        # UTM 좌표를 로컬 포즈로 변환하여 Path 메시지에 추가
                        pose = PoseStamped()
                        pose.header = Header(stamp=rospy.Time.now(), frame_id="map")
                        pose.pose.position.x = utm_easting
                        pose.pose.position.y = utm_northing
                        pose.pose.position.z = altitude
                        pose.pose.orientation.w = 1.0  # 기본적인 방향 설정

                        self.global_path.poses.append(pose)

                        # 목표 지점을 리스트에 추가
                        self.goals.append(pose)

                        i += 6  # 다음 데이터 블록으로 이동
                    else:
                        i += 1  # 빈 줄이나 다른 형식의 줄을 무시하고 다음 줄로 이동

            rospy.loginfo("Global path created with {} points.".format(len(self.global_path.poses)))
        except Exception as e:
            rospy.logerr("Failed to create global path: {}".format(e))

    def publish_path(self):
        rate = rospy.Rate(1)  # 1Hz로 발행
        while not rospy.is_shutdown():
            self.global_path.header.stamp = rospy.Time.now()
            self.path_pub.publish(self.global_path)
            rospy.loginfo("Path published")
            rate.sleep()

    def odom_callback(self, msg):
        if self.current_goal_index < len(self.goals):
            goal = self.goals[self.current_goal_index]
            distance = self.calculate_distance(msg.pose.pose.position, goal.pose.position)

            if distance < self.threshold:
                rospy.loginfo(f"Reached goal {self.current_goal_index + 1}")
                self.current_goal_index += 1
                if self.current_goal_index < len(self.goals):
                    self.publish_goal()
                else:
                    rospy.loginfo("All goals reached")

    def calculate_distance(self, pos1, pos2):
        return math.sqrt((pos1.x - pos2.x)**2 + (pos1.y - pos2.y)**2)

    def publish_goal(self):
        if self.current_goal_index < len(self.goals):
            goal = self.goals[self.current_goal_index]
            goal.header.stamp = rospy.Time.now()
            self.goal_pub.publish(goal)
            rospy.loginfo(f"Publishing goal {self.current_goal_index + 1}")

    def publish_goal_periodically(self):
        rate = rospy.Rate(1)  # 1Hz로 발행
        while not rospy.is_shutdown():
            if self.current_goal_index < len(self.goals):
                goal = self.goals[self.current_goal_index]
                goal.header.stamp = rospy.Time.now()
                self.goal_pub.publish(goal)
                rospy.loginfo(f"Publishing goal {self.current_goal_index + 1} periodically")
            rate.sleep()

if __name__ == '__main__':
    try:
        input_file = "/home/ros/utm_data/utm1.txt"  # UTM 데이터 파일 경로를 지정하세요.
        GpsToGlobalPath(input_file)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
