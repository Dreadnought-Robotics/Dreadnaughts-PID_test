#!usr/bin/python3

import rospy
from sensor_msgs.msg import Imu
import time

def getposition(acc, previous_vel, previous_distance):
    vel = previous_vel + acc*(time_lapsed)
    distance = vel*(time_lapsed) + previous_distance
    previous_vel = vel
    previous_distance = distance
    return distance

def talker2(self, Imu):
    self.acc_x = Imu.linear_acceleration.x
    self.acc_y = Imu.linear_acceleration.y
    self.acc_z = Imu.linear_acceleration.z
    print("acc")
    print(self.acc_z)



if __name__=='__main__':
    try:
        time_lapsed = 0
        accdata=rospy.Subscriber("/calypso_sim/imu/data",Imu, talker2)
        i = 0
        while i<3:
            start_time = time.time()
            height = getposition
            print(height)
            end_time = time.time()
            time_lapsed = end_time - start_time
            i = i+1
    except rospy.ROSInterruptException:
        pass