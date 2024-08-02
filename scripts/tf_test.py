#!/usr/bin/python3

import rospy
import tf
import math
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped


class Robot(object):
   def __init__(self):
       position = PoseStamped()

       position.header.frame_id = "home"

       position.pose.position.x = 1
       position.pose.position.y = 1
       position.pose.position.z = 1

       # 새로운 회전 상태 설정 (예: roll=0, pitch=0, yaw=90도)
       quat = quaternion_from_euler(0, 0, math.radians(0))
       position.pose.orientation.x = quat[0]
       position.pose.orientation.y = quat[1]
       position.pose.orientation.z = quat[2]
       position.pose.orientation.w = quat[3]

       self.position = position
       self.pub = rospy.Publisher("robot_pose", PoseStamped, queue_size=1)

   def publishPose(self):
       self.position.header.stamp = rospy.Time.now()

       self.pub.publish(self.position)


if __name__ == '__main__':
   rospy.init_node("tf_pub")

   robot = Robot()

   r = rospy.Rate(1)
   while not rospy.is_shutdown():

       robot.publishPose()

       r.sleep()