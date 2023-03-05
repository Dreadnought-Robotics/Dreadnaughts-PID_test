#! /usr/bin/python3

import rospy
from calypso_msgs.msg import dolphins
from calypso_msgs.msg import buoy
from sensor_msgs.msg import Imu
import time
import math
from scipy.interpolate import interp1d

class for_bac:
    def __init__(self):

        rospy.init_node("for_back")
        self.rate = rospy.Rate(10)
        self.subscriber = rospy.Subscriber("/rosetta/imu/data", buoy, self.getimu)
        self.publisher  = rospy.Publisher("/rosetta/dolphins", dolphins,queue_size=1000)
        self.accdata=rospy.Subscriber("/calypso_sim/imu/data",Imu, self.talker2)
        self.x_coordinate = 0
        self.y_coordinate = 0
        self.req_yaw = math.degrees(math.atan2(self.y_coordinate, self.x_coordinate))


        self.kp_yaw = 15
        self.ki_yaw = 0 
        self.kd_yaw = 30
        self.previous_throttle = 0
        self.integrator_throttle = 0

        self.previous_error_yaw = 0
        self.pid_i_yaw = 0

        self.x = 0
        self.y = 0
        self.z = 0
        self.w = 0
        self.m = interp1d([-30, 30],[1158,2000])
        self.throttle1 = 1540
        self.throttle2 = 1540
        self.throttle3 = 1540 
        self.throttle4 = 1540

        self.angvel_z = 0

    def start(self):
        while not rospy.is_shutdown():
            self.roll , self.pitch , self.yaw = self.convert()
            self.yaw_init = self.yaw
            while math.sqrt(math.pow(self.req_yaw - (self.yaw - self.yaw_init)), 2) > 0:
                self.start_time = time.time()
                self.roll , self.pitch , self.yaw = self.convert()
                self.error_yaw = (self.req_yaw - (self.yaw - self.yaw_init))
                self.PID_yaw = self.getPID(self.kd_yaw, self.ki_yaw, self.kp_yaw, self.error_yaw, 0, self.pid_i_yaw, self.previous_error_yaw, self.angvel_z)
                self.d = dolphins()
                
                self.d.d1 = round(self.throttle1 + self.PID_yaw)
                self.d.d2 = round(self.throttle2 - self.PID_yaw)
                self.d.d3 = round(self.throttle3 + self.PID_yaw)
                self.d.d4 = round(self.throttle4 - self.PID_yaw)

                if self.d.d1<1538:   #stopping speed
                    self.throttle1 = 1468
                
                if self.d.d2<1538:   #stopping speed
                    self.throttle1 = 1468

                if self.d.d3<1538:   #stopping speed
                    self.throttle1 = 1468

                if self.d.d4<1538:   #stopping speed
                    self.throttle1 = 1468

                self.d.d1 = round(self.throttle1 + self.PID_yaw)
                self.d.d2 = round(self.throttle2 + self.PID_yaw)
                self.d.d3 = round(self.throttle3 + self.PID_yaw)
                self.d.d4 = round(self.throttle4 + self.PID_yaw)

                end_time = time.time()
                self.time_lapsed = end_time - self.start_time

            self.error_dist = math.sqrt((self.y_coordinate * self.y_coordinate) + (self.x_coordinate * self.x_coordinate))
            
            while self.error_dist > 0:
                self.start_time = time.time()
                # from linear acceleration find distance covered
                self.error_dist = math.sqrt((self.y_coordinate * self.y_coordinate) + (self.x_coordinate * self.x_coordinate))





    def getPID(self, kd, ki, kp, actual, desired, pid_i, previous_error, angvel):
  
        error = desired - actual
        pid_p = kp*error
        pid_i = pid_i + error
        # pid_d = kd*(error - previous_error)
        pid_d = kd*angvel
        if pid_i>max(90-pid_p-pid_d, 0):
            pid_i = max(90-pid_p-pid_d,0)
        elif pid_i<min(-90-pid_i-pid_d, 0):
            pid_i = min(-90-pid_p-pid_d,0)
        pid_i_final = ki*pid_i
        PID = pid_p + pid_i_final + pid_d

        if(PID > 90):
            PID=90
        if(PID < -90):
            PID=-90
        previous_error = error
        return PID