#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import NavSatFix
from datetime import datetime
import os
import threading

class GpsDataSaver: #gps데이터 저장된 .txt가 가장처음께 가장 위 
    def __init__(self):
        rospy.init_node('gps_data_saver', anonymous=True)
        
        self.current_gps_data = None
        self.saved_gps_data = []
        self.save_path = "/home/ros/waypoint_data"
        
        # 저장 경로 설정
        if not os.path.exists(self.save_path):
            os.makedirs(self.save_path)
        
        # GPS 데이터 구독
        rospy.Subscriber('/gps_real', NavSatFix, self.gps_callback)

        # 키보드 입력을 별도 스레드에서 처리
        self.input_thread = threading.Thread(target=self.process_input)
        self.input_thread.start()

        # rospy.spin()을 별도 스레드에서 실행
        self.spin_thread = threading.Thread(target=rospy.spin)
        self.spin_thread.start()

    def gps_callback(self, data):
        self.current_gps_data = data
        #rospy.loginfo(f"Received GPS data: Latitude={data.latitude}, Longitude={data.longitude}, Altitude={data.altitude}")

    def process_input(self):
        while not rospy.is_shutdown():
            user_input = input("Press 's' to save GPS data, 'q' to save all data and exit: ")
            if user_input == 's':
                rospy.loginfo("Key 's' pressed, saving GPS data...")
                self.save_gps_data()
            elif user_input == 'q':
                rospy.loginfo("Key 'q' pressed, saving all GPS data to file and exiting...")
                self.save_all_data_and_exit()
                break

    def save_gps_data(self):
        if self.current_gps_data is None:
            rospy.logwarn("No GPS data available to save.")
            return
        
        self.saved_gps_data.append({
            'latitude': self.current_gps_data.latitude,
            'longitude': self.current_gps_data.longitude,
            'altitude': self.current_gps_data.altitude,
            'timestamp': datetime.now().strftime("%Y%m%d_%H%M%S")
        })
        
        rospy.loginfo("GPS data saved in memory")

    def save_all_data_and_exit(self):
        filename = os.path.join(self.save_path, datetime.now().strftime("%Y%m%d_%H%M%S_all_gps_data.txt"))
        
        with open(filename, 'w') as file:
            for data in self.saved_gps_data:
                file.write(f"Timestamp: {data['timestamp']}\n")
                file.write(f"Latitude: {data['latitude']}\n")
                file.write(f"Longitude: {data['longitude']}\n")
                file.write(f"Altitude: {data['altitude']}\n")
                file.write("\n")

        rospy.loginfo(f"All GPS data saved to {filename}")
        rospy.signal_shutdown("User requested shutdown")

if __name__ == '__main__':
    try:
        GpsDataSaver()
    except rospy.ROSInterruptException:
        pass
