#!usr/bin/python3

import rospy
from calypso_msgs.msg import gypseas
from calypso_msgs.msg import buoy
from sensor_msgs.msg import Imu
import time
from scipy.interpolate import interp1d

class hover:
    def __init__(self):

        rospy.init_node("hovering")
        self.rate = rospy.Rate(10)
        self.subscriber = rospy.Subscriber("/rosetta/imu/data", buoy, self.getimu)
        self.publisher  = rospy.Publisher("/rosetta/gypseas", gypseas,queue_size=1000)
        self.accdata=rospy.Subscriber("/calypso_sim/imu/data",Imu, self.talker2)
        self.height_to_move = 3 #set this
        self.time_lapsed = 0
        self.altitude = 0

        self.kp = 10
        self.ki = 0 
        self.kd = 0
        self.previous_throttle = 0
        self.integrator_throttle = 0
        self.previous_vel_z = 0
        self.acc_z = 0
        
        self.x = 0
        self.y = 0
        self.z = 0
        self.w = 0
        self.m = interp1d([0, 30],[1579,1800])

    def start(self):
        while not rospy.is_shutdown():
            start_time = time.time()
            moved = self.getposition(self.acc_z, self.previous_vel_z, self.altitude)
            self.altitude = self.altitude + moved
            print("altitude")
            print(100*self.altitude)

            self.throttle_to_map = self.getPID(self.kp, self.ki, self.kd, 100*self.altitude, self.height_to_move, self.integrator_throttle, self.previous_throttle)
            self.throttle = self.m(self.throttle_to_map)
            
            self.g = gypseas()

            self.g.t1 = int(self.throttle)
            self.g.t2 = int(self.throttle)
            self.g.t3 = int(self.throttle)
            self.g.t4 = int(self.throttle)  
            self.publisher.publish(self.g)
            self.rate.sleep()
            end_time = time.time()
            self.time_lapsed = end_time - start_time


    def getimu(self, buoy):
        self.x = buoy.x
        self.y = buoy.y
        self.z = buoy.z
        self.w = buoy.w

    def talker2(self, Imu):
        self.acc_x = Imu.linear_acceleration.x
        self.acc_y = Imu.linear_acceleration.y
        self.acc_z = Imu.linear_acceleration.z
        print("acc")
        print(self.acc_z)

    def getposition(self, acc, previous_vel, previous_distance):
        vel = previous_vel + acc*(self.time_lapsed)
        distance = vel*(self.time_lapsed) + 0.5*(acc * self.time_lapsed * self.time_lapsed)
        previous_vel = vel
        previous_distance = distance
        return distance
    
    def getPID(self, kd, ki, kp, actual, desired, pid_i, previous_error):
  
        error = desired - actual
        pid_p = kp*error
        pid_i = pid_i + error
        pid_d = kd*(error - previous_error)
        if pid_i>max(30-pid_p-pid_d, 0):
            pid_i = max(30-pid_p-pid_d,0)
        elif pid_i<min(-30-pid_i-pid_d, 0):
            pid_i = min(-30-pid_p-pid_d,0)
        pid_i_final = ki*pid_i
        PID = pid_p + pid_i_final + pid_d

        if(PID > 30):
            PID=30
        if(PID < -30):
            PID=-30
        previous_error = error
        return PID


if __name__=='__main__':
    try:
        x = hover()
        x.start()
    except rospy.ROSInterruptException:
        pass
