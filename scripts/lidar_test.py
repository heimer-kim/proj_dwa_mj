#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan

def degTorad(deg):
	rad_diff = 0.5304
	rad = deg * (3.14/180)
	return rad + rad_diff

def callback(data):
    '''
    Publisher speed,
    '''
    speed = rospy.Publisher('/commands/motor/speed', Float64, queue_size=1)
    position = rospy.Publisher('/commands/servo/position', Float64, queue_size=1)

    range_data = list(data.ranges)[::-1]
    
    if min(range_data[0:30]) < 5 or min(range_data[330:359]) < 5:
        if min(range_data[0:30]) > min(range_data[330:359]):
            speed.publish(7000)
            position.publish(degTorad(11))
        else:
            speed.publish(7000)
            position.publish(degTorad(-11))
    else:
        speed.publish(10000)
        position.publish(degTorad(0))

if __name__ == '__main__':
    rospy.init_node("ObjectAvoid")
    sub = rospy.Subscriber("/lidar2D", LaserScan, callback)
    rospy.spin()