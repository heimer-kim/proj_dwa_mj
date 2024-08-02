#!/usr/bin/env python3

import rospy
import tf
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class GlobalPathPlanner:
    def __init__(self):
        self.tf_listener = tf.TransformListener()
        self.path_pub = rospy.Publisher('/global_path', Path, queue_size=10)
        rospy.Timer(rospy.Duration(1.0), self.update_path)
        rospy.loginfo("GlobalPathPlanner initialized")

    def update_path(self, event):
        try:
            now = rospy.Time.now()
            # Request the latest available transform
            self.tf_listener.waitForTransform("/world", "/base_link", rospy.Time(0), rospy.Duration(1.0))
            (trans, rot) = self.tf_listener.lookupTransform("/world", "/base_link", rospy.Time(0))

            rospy.loginfo("Transform received: trans={}, rot={}".format(trans, rot))

            # Example: create a simple straight path from the current position
            path = Path()
            path.header.stamp = now
            path.header.frame_id = "world"

            start_pose = PoseStamped()
            start_pose.header.frame_id = "world"
            start_pose.pose.position.x = trans[0]
            start_pose.pose.position.y = trans[1]
            start_pose.pose.position.z = trans[2]
            start_pose.pose.orientation.x = rot[0]
            start_pose.pose.orientation.y = rot[1]
            start_pose.pose.orientation.z = rot[2]
            start_pose.pose.orientation.w = rot[3]
            path.poses.append(start_pose)

            # Example target position (10, 10, 0)
            target_pose = PoseStamped()
            target_pose.header.frame_id = "world"
            target_pose.pose.position.x = 10.0
            target_pose.pose.position.y = 10.0
            target_pose.pose.position.z = 0.0
            target_pose.pose.orientation.w = 1.0
            path.poses.append(target_pose)

            self.path_pub.publish(path)
            rospy.loginfo("Published path: {}".format(path))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn("TF Exception: {}".format(e))

if __name__ == '__main__':
    rospy.init_node('global_path_planner')
    planner = GlobalPathPlanner()
    rospy.spin()
