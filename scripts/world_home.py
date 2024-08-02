#!/usr/bin/python3

import rospy
import tf
import math
import tf_test
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped

rospy.init_node('world_home_pub')


tf_broadcaster = tf.TransformBroadcaster()
robot = tf_test.Robot()

r = rospy.Rate(1)
while not rospy.is_shutdown():

        tf_broadcaster.sendTransform(
            translation=[2, 2, 0],
            rotation=tf.transformations.quaternion_from_euler(0, 0, 0),
            time=rospy.Time.now(),
            child="home",
            parent="world"
        )

        tf_broadcaster.sendTransform(
            translation=[-2, -2, 0],
            rotation=tf.transformations.quaternion_from_euler(0, 0, 0),
            time=rospy.Time.now(),
            child="robot_cennter",
            parent="world"
        )

        tf_broadcaster.sendTransform(
            translation=[0, 0, 3],
            rotation=tf.transformations.quaternion_from_euler(0, 0, 0),
            time=rospy.Time.now(),
            child="camera",
            parent="robot_center"
        )

        robot.publishPose()

        r.sleep()

        
