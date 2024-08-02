#!/usr/bin/env python3

import rospy
import tf
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler
import time
import math

class RobotController:
    def __init__(self):
        rospy.init_node('robot_controller', anonymous=True)

        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)

        self.odom_broadcaster = tf.TransformBroadcaster()

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.vx = 0.0
        self.vy = 0.0
        self.vth = 0.0

        self.last_time = time.time()

        self.rate = rospy.Rate(10.0)

    def cmd_vel_callback(self, msg):
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.vth = msg.angular.z
        rospy.loginfo("Received a /cmd_vel message!")
        rospy.loginfo("Linear Components: [%f, %f, %f]" % (msg.linear.x, msg.linear.y, msg.linear.z))
        rospy.loginfo("Angular Components: [%f, %f, %f]" % (msg.angular.x, msg.angular.y, msg.angular.z))

    def update_odometry(self):
        current_time = time.time()
        dt = current_time - self.last_time

        delta_x = self.vx * dt
        delta_y = self.vy * dt
        delta_th = self.vth * dt

        self.x += delta_x * math.cos(self.th) - delta_y * math.sin(self.th)
        self.y += delta_x * math.sin(self.th) + delta_y * math.cos(self.th)
        self.th += delta_th

        quat = quaternion_from_euler(0, 0, self.th)

        # Publish odometry message
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "odom"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = quat[0]
        odom.pose.pose.orientation.y = quat[1]
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3]

        odom.child_frame_id = "base_link"
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.vth

        self.odom_pub.publish(odom)

        # Publish TF transform
        self.odom_broadcaster.sendTransform(
            (self.x, self.y, 0),
            quat,
            rospy.Time.now(),
            "base_link",
            "odom"
        )

        self.last_time = current_time

    def spin(self):
        while not rospy.is_shutdown():
            self.update_odometry()
            self.rate.sleep()

if __name__ == '__main__':
    try:
        robot_controller = RobotController()
        robot_controller.spin()
    except rospy.ROSInterruptException:
        pass
