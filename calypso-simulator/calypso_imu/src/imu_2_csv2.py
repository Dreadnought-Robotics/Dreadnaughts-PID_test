#! /usr/bin/python3
import pandas as pd
import csv
from calypso_msgs.msg import buoy
from datetime import datetime
import time
import rospy

def write_into_csv1(name,row):
  with open('/home/dafodilrat/auv/src/calypso_imu/csv_files/'+name, 'w', newline='') as file:
     writer = csv.writer(file)
     writer.writerow(row)

def write_into_csv(name,row):
  with open('/home/dafodilrat/auv/src/calypso_imu/csv_files/'+name, 'a', newline='') as file:
     writer = csv.writer(file)
     writer.writerow(row)

def listener(imu_msg):
  row = [time.time() - start_time, imu_msg.gyro_x, imu_msg.gyro_y, imu_msg.gyro_z, imu_msg.accel_x, imu_msg.accel_y, imu_msg.accel_z, imu_msg.mag_x, imu_msg.mag_y, imu_msg.mag_z]
  write_into_csv(csv_name, row)
  rospy.loginfo(imu_msg)



if __name__ == '__main__':
  now = datetime.now()
  date_time = now.strftime("%m_%d_%Y__%H_%M:%S")
  csv_name = date_time+"_imu_test_agm.csv"
  start_time = time.time()
  list = ["Packet number", "Gyroscope X (deg/s)", "Gyroscope Y (deg/s)", "Gyroscope Z (deg/s)", "Accelerometer X (g)", "Accelerometer Y (g)", "Accelerometer Z (g)", "Magnetometer X (G)", "Magnetometer Y (G)", "Magnetometer Z (G)"]
  write_into_csv1(csv_name,list)


  rospy.init_node('imu_msg_2_csv', anonymous=False)
  while not rospy.is_shutdown():
    gypseas=rospy.Subscriber("/calypso/imu_raw", buoy, listener)