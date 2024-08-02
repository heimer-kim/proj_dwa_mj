#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

def parse_utm_string(utm_string):
    parts = utm_string.split(',')
    easting = float(parts[0].split(': ')[1])
    northing = float(parts[1].split(': ')[1])
    zone_number = int(parts[2].split(': ')[1])
    zone_letter = parts[3].split(': ')[1].strip()
    altitude = float(parts[4].split(': ')[1])
    
    return easting, northing, altitude

def utm_to_pose(easting, northing, altitude):
    pose = PoseStamped()
    pose.header = Header()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = 'map'
    pose.pose.position.x = easting
    pose.pose.position.y = northing
    pose.pose.position.z = altitude
    pose.pose.orientation.w = 1.0
    return pose

def gps_utm_callback(data):
    easting, northing, altitude = parse_utm_string(data.data)
    goal_pose = utm_to_pose(easting, northing, altitude)
    goal_pub.publish(goal_pose)
    rospy.loginfo("Published goal pose: x=%f, y=%f, z=%f" % (easting, northing, altitude))

def publish_custom_goal(easting, northing, altitude):
    goal_pose = utm_to_pose(easting, northing, altitude)
    goal_pub.publish(goal_pose)
    rospy.loginfo("Published custom goal pose: x=%f, y=%f, z=%f" % (easting, northing, altitude))

rospy.init_node('goal_publisher', anonymous=True)
goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
rospy.Subscriber('/gps_utm', String, gps_utm_callback)

# 예시로 사용자 지정 목표 지점 발행 (이 좌표는 사용자가 직접 설정)
custom_easting =  833462.9881265063
custom_northing = 9999521.28788737
custom_altitude = 0.5

# 사용자 지정 목표 지점을 발행하는 부분
rospy.sleep(2)  # 노드 초기화 후 잠시 대기
publish_custom_goal(custom_easting, custom_northing, custom_altitude)

rospy.spin()
