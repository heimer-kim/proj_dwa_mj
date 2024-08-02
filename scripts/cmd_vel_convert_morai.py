#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class CmdVelToSpeedPosition:
    def __init__(self):
        rospy.init_node('cmd_vel_to_speed_position', anonymous=True)
        
        # Publisher 설정
        self.speed_pub = rospy.Publisher('/commands/motor/speed', Float64, queue_size=1)
        self.position_pub = rospy.Publisher('/commands/servo/position', Float64, queue_size=1)
        
        # Subscriber 설정
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
    
    def deg_to_rad(self, deg):
        rad_diff = 0.5304
        rad = deg * (3.14 / 180)
        return rad + rad_diff

    def cmd_vel_callback(self, data):
        # /cmd_vel 메시지에서 linear.x와 angular.z 값을 추출
        linear_x = data.linear.x
        angular_z = data.angular.z

        # speed와 position 값 계산 (예시: 적절히 조정 필요)
        speed = linear_x*7000  # 직진 속도를 motor speed로 사용
        position = angular_z  # 회전 속도를 servo position으로 사용

        # 메시지 생성
        speed_msg = Float64()
        speed_msg.data = speed

        position_msg = Float64()
        position_msg.data = position

        # 발행
        self.speed_pub.publish(speed_msg)
        self.position_pub.publish(position_msg)

if __name__ == '__main__':
    try:
        node = CmdVelToSpeedPosition()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
