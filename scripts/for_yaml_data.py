#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu
import math

class IMUProcessor: #robot_param.yaml에 필요한 변수를 imu를 통해 구하는 코드
    def __init__(self):
        self.max_acceleration = 0.0
        self.max_deceleration = 0.0
        self.max_yawrate = 0.0
        self.max_d_yawrate = 0.0
        self.prev_time = rospy.Time.now()
        self.prev_angular_velocity_z = 0.0

        rospy.Subscriber('/imu', Imu, self.imu_callback)

    def imu_callback(self, data):
        current_time = rospy.Time.now()
        dt = (current_time - self.prev_time).to_sec()
        self.prev_time = current_time

        # Linear acceleration
        linear_acceleration = data.linear_acceleration.x
        self.max_acceleration = max(self.max_acceleration, linear_acceleration)
        self.max_deceleration = min(self.max_deceleration, linear_acceleration)

        # Angular velocity
        angular_velocity_z = data.angular_velocity.z
        self.max_yawrate = max(self.max_yawrate, abs(angular_velocity_z))

        # Angular acceleration (yaw rate change)
        angular_acceleration_z = (angular_velocity_z - self.prev_angular_velocity_z) / dt
        self.max_d_yawrate = max(self.max_d_yawrate, abs(angular_acceleration_z))
        self.prev_angular_velocity_z = angular_velocity_z

    def print_parameters(self):
        print("MAX_ACCELERATION: {:.2f} m/s^2".format(self.max_acceleration))
        print("MAX_DECELERATION: {:.2f} m/s^2".format(abs(self.max_deceleration)))
        print("MAX_YAWRATE: {:.2f} rad/s".format(self.max_yawrate))
        print("MAX_D_YAWRATE: {:.2f} rad/s^2".format(self.max_d_yawrate))

if __name__ == '__main__':
    rospy.init_node('imu_processor', anonymous=True)
    imu_processor = IMUProcessor()

    rospy.spin()
    imu_processor.print_parameters()
