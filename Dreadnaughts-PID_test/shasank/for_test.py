#! /usr/bin/python3

import rospy
from calypso_msgs.msg import dolphins
from calypso_msgs.msg import buoy
from sensor_msgs.msg import Imu
import time
import math
from scipy.interpolate import interp1d
from scipy.integrate import trapezoid

class for_bac:
    def __init__(self):

        rospy.init_node("for_back")
        self.rate = rospy.Rate(10)
        self.subscriber = rospy.Subscriber("/rosetta/imu/data", buoy, self.getimu)
        self.publisher  = rospy.Publisher("/rosetta/dolphins", dolphins,queue_size=1000)
        self.accdata=rospy.Subscriber("/calypso_sim/imu/data",Imu, self.talker2)
        self.x_coordinate = 1
        self.y_coordinate = 1
        self.displacement = 0
        self.time_lapsed = 0
        self.req_yaw = math.degrees(math.atan2(self.y_coordinate, self.x_coordinate))

        self.kp = 15
        self.ki = 0 
        self.kd = 30
        self.previous_throttle = 0
        self.integrator_throttle = 0

        self.x = 0
        self.y = 0
        self.z = 0
        self.w = 0
        self.m = interp1d([-30, 30],[1158,2000])
        self.n = interp1d([-30, 0], [1200,1468])

        self.throttle1 = 1540
        self.throttle2 = 1540
        self.throttle3 = 1540 
        self.throttle4 = 1540

        self.acc_z = 0
        self.acc_x = 0
        self.ang_y = 0 

    def start(self):
        while not rospy.is_shutdown():
            self.error_dist = math.sqrt((self.y_coordinate * self.y_coordinate) + (self.x_coordinate * self.x_coordinate))
            self.start_time = time.time()
            self.time = []
            self.acc = []
            self.vel = []
            while self.error_dist-self.displacement > 0:
                end_time = time.time()
                self.time_lapsed = end_time - self.start_time
                self.time.append(self.time_lapsed)
                self.acc.append(self.acc_x)
                self.vel_x = self.integrate(self.acc, self.time)
                self.vel.append(self.vel_x)
                self.displacement = -self.integrate(self.vel, self.time)
                print("displacement")
                print(self.displacement)

                self.throttle_to_map = self.getPID_mo(self.kd, self.ki, self.kp, self.displacement, self.error_dist, self.integrator_throttle, self.previous_throttle, self.acc_x)
                if self.throttle_to_map>=0:
                    self.throttle_fro = self.m(self.throttle_to_map)
                    self.throttle_rear = self.n(self.throttle_to_map)
                else:
                    self.throttle = self.n(self.throttle_to_map)
                print("map")
                print(self.throttle_to_map)
                self.g = dolphins()
                print("acc")
                print(self.acc_x)

                self.g.d1 = int(self.throttle)
                self.g.d2 = int(self.throttle)
                self.g.d3 = int(self.throttle)
                self.g.d4 = int(self.throttle)  
                self.publisher.publish(self.g)
                self.rate.sleep()

            

    def getPID_mo(self, kd, ki, kp, actual, desired, pid_i, previous_error, feedforward):
  
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
    

if __name__=='__main__':
    try:
        x = for_bac()
        x.start()
    except rospy.ROSInterruptException:
        pass
