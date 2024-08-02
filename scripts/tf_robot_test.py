#!/usr/bin/env python3

import rospy
import tf
from tf.transformations import quaternion_from_euler

def broadcast_transforms():
    br = tf.TransformBroadcaster()


    # # Transform from map to base_link (필수)map과 odom의 위채를 상대좌표로 고정시키면 이동할때 변화되는값이 반영이 안되서 이건 쓰면 안됨
    # br.sendTransform((0.0, 0.0, 0.0),  # z축으로 0.05 이동
    #                  quaternion_from_euler(0, 0, 0),
    #                  rospy.Time.now(),
    #                  "odom",
    #                  "map")
    
    # Transform from odom to base_link (필수)
    br.sendTransform((0.0, 0.0, 0.0),  # z축으로 0.5 이동
                     quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(),
                     "base_link",
                     "odom")

    # Transform from base_link to camera
    br.sendTransform((0.25, 0.0, 0.2), # translation
                     quaternion_from_euler(0, 0, 0), # rotation
                     rospy.Time.now(), # time
                     "camera",  # child
                     "base_link") # parent

    # Transform from base_link to left_wheel
    br.sendTransform((0.0, 0.25, -0.05),
                     quaternion_from_euler(1.5708, 0, 0),
                     rospy.Time.now(),
                     "left_wheel",
                     "base_link")

    # Transform from base_link to right_wheel
    br.sendTransform((0.0, -0.25, -0.05),
                     quaternion_from_euler(1.5708, 0, 0),
                     rospy.Time.now(),
                     "right_wheel",
                     "base_link")
     
    # Transform from base_link to lidar_link
    br.sendTransform((0.0, 0.0, 0.3),
                     quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(),
                     "lidar_link",
                     "base_link")

    # Transform from base_link to imu_link
    br.sendTransform((0.0, 0.0, 0.1),
                     quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(),
                     "imu_link",
                     "base_link")
    
    
if __name__ == '__main__':
    rospy.init_node('robot_tf_broadcaster')
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        broadcast_transforms()
        rate.sleep()
