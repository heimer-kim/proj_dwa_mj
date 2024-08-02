#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import utm
import os
from datetime import datetime

class GpsToUtmConverter:
    def __init__(self, input_file, output_path):
        self.input_file = input_file
        self.output_path = output_path
        
        # 저장 경로 설정
        if not os.path.exists(self.output_path):
            os.makedirs(self.output_path)
        
        # UTM 데이터 저장
        self.convert_and_save()

    def convert_and_save(self):
        output_file = os.path.join(self.output_path, f"UTM_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt")

        with open(self.input_file, 'r') as infile, open(output_file, 'w') as outfile:
            lines = infile.readlines()
            for i in range(0, len(lines), 5):
                if i+3 < len(lines):
                    timestamp = lines[i].strip().split(": ")[1]
                    latitude = float(lines[i+1].strip().split(": ")[1])
                    longitude = float(lines[i+2].strip().split(": ")[1])
                    altitude = float(lines[i+3].strip().split(": ")[1])

                    utm_coords = utm.from_latlon(latitude, longitude)
                    utm_easting = utm_coords[0]
                    utm_northing = utm_coords[1]
                    utm_zone_number = utm_coords[2]
                    utm_zone_letter = utm_coords[3]

                    outfile.write(f"Timestamp: {timestamp}\n")
                    outfile.write(f"UTM Easting: {utm_easting}\n")
                    outfile.write(f"UTM Northing: {utm_northing}\n")
                    outfile.write(f"UTM Zone Number: {utm_zone_number}\n")
                    outfile.write(f"UTM Zone Letter: {utm_zone_letter}\n")
                    outfile.write(f"Altitude: {altitude}\n")
                    outfile.write("\n")

        print(f"UTM data saved to {output_file}")

if __name__ == '__main__':
    input_file = "/home/ros/waypoint_data/test1.txt"  # 입력 파일 경로를 지정하세요.
    output_path = "/home/ros/utm_data"  # 출력 디렉토리를 지정하세요.
    GpsToUtmConverter(input_file, output_path)
