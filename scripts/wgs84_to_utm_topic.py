#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
import utm

class GpsConverter:
    def __init__(self):
        rospy.init_node('gps_converter', anonymous=True)
        
        # 구독 및 발행 설정
        rospy.Subscriber('/gps_real', NavSatFix, self.gps_callback)
        self.utm_pub = rospy.Publisher('/gps_utm', String, queue_size=10)

    def gps_callback(self, data):
        latitude = data.latitude
        longitude = data.longitude
        altitude = data.altitude

        # WGS84 좌표를 UTM 좌표로 변환
        utm_coords = utm.from_latlon(latitude, longitude)
        utm_easting = utm_coords[0]
        utm_northing = utm_coords[1]
        utm_zone_number = utm_coords[2]
        utm_zone_letter = utm_coords[3]

        # UTM 좌표를 문자열로 포맷팅
        utm_data = (f"UTM Easting: {utm_easting}, UTM Northing: {utm_northing}, "
                    f"UTM Zone Number: {utm_zone_number}, UTM Zone Letter: {utm_zone_letter}, "
                    f"Altitude: {altitude}")

        #rospy.loginfo(f"Converted to UTM: {utm_data}")

        # 변환된 UTM 좌표를 /gps_utm 토픽으로 발행
        self.utm_pub.publish(utm_data)

if __name__ == '__main__':
    try:
        GpsConverter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
