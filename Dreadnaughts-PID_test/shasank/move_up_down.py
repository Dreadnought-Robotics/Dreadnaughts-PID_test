#!usr/bin/python3

import rospy
from calypso_msgs.msg import gypseas
from calypso_msgs.msg import buoy
from sensor_msgs.msg import Imu
import time
from scipy.interpolate import interp1d
from scipy.integrate import trapezoid
import math

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

        self.kp = 30
        self.ki = 50
        self.kd = 1
        self.previous_throttle = 0
        self.integrator_throttle = 0
        self.vel = 0
        self.previous_acc = 0
        self.prev_acc_z = 0
        self.acc_z = 0
        self.vel_z = 0
        
        self.x = 0
        self.y = 0
        self.z = 0
        self.w = 0
        self.m = interp1d([0, 30],[1574,2000])
        self.n = interp1d([-30, 0], [1200,1468])
        self.start_time = time.time()

        self.time = []
        self.acc = []
        self.vel = []
        

    def start(self):
        while not rospy.is_shutdown():

            end_time = time.time()
            self.time_lapsed = end_time - self.start_time
            self.time.append(self.time_lapsed)
            self.acc.append(self.acc_z)
            self.vel_z = self.integrate(self.acc, self.time)
            self.vel.append(self.vel_z)
            self.altitude = self.integrate(self.vel, self.time)
            print("altitude")
            print(self.altitude)

            self.throttle_to_map = self.getPID(self.kd, self.ki, self.kp, self.altitude, self.height_to_move, self.integrator_throttle, self.previous_throttle, self.acc_z)
            if self.throttle_to_map>=0:
                self.throttle = self.m(self.throttle_to_map)
            else:
                self.throttle = self.n(self.throttle_to_map)
            print("map")
            print(self.throttle_to_map)
            self.g = gypseas()
            print("acc")
            print(self.acc_z)

            self.g.t1 = int(self.throttle)
            self.g.t2 = int(self.throttle)
            self.g.t3 = int(self.throttle)
            self.g.t4 = int(self.throttle)  
            self.publisher.publish(self.g)
            self.rate.sleep()



    def getimu(self, buoy):
        self.x = buoy.x
        self.y = buoy.y
        self.z = buoy.z
        self.w = buoy.w

    def talker2(self, Imu):
        self.acc_x = Imu.linear_acceleration.x
        self.acc_y = Imu.linear_acceleration.y
        self.acc_z = Imu.linear_acceleration.z



    def integrate(self, y, x):
        return trapezoid(y, x)
        
    
    def getPID(self, kd, ki, kp, actual, desired, pid_i, previous_error, feedforward):
  
        error = desired - actual
        pid_p = kp*error
        pid_i = pid_i + error
        pid_d = kd*(error - previous_error)
        if pid_i>max(30-pid_p-pid_d, 0):
            pid_i = max(30-pid_p-pid_d,0)
        elif pid_i<min(-30-pid_i-pid_d, 0):
            pid_i = min(-30-pid_p-pid_d,0)
        pid_i_final = ki*pid_i
        PID = pid_p + (pid_i_final + pid_d)/self.time_lapsed + feedforward

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
