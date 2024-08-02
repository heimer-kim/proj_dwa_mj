#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import NavSatFix, NavSatStatus
from morai_msgs.msg import GPSMessage


#morai gps 토픽타입을 nav 로 타입변환
def gps_callback(data):
    navsatfix = NavSatFix()
    navsatfix.header = data.header
    navsatfix.status.status = NavSatStatus.STATUS_FIX
    navsatfix.status.service = NavSatStatus.SERVICE_GPS
    navsatfix.latitude = data.latitude
    navsatfix.longitude = data.longitude
    navsatfix.altitude = data.altitude

    navsatfix_pub.publish(navsatfix)

if __name__ == '__main__':
    rospy.init_node('gps_converter')
    navsatfix_pub = rospy.Publisher('/gps_real', NavSatFix, queue_size=10)  # NavSatFix 메시지를 /gps 토픽에 발행
    rospy.Subscriber('/gps', GPSMessage, gps_callback)  # GPSMessage 메시지를 /gps 토픽에서 구독
    rospy.spin()
