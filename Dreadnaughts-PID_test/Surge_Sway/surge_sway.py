#!usr/bin/python3

import rospy
from calypso_msgs.msg import dolphins
from calypso_msgs.msg import buoy
from sensor_msgs.msg import Imu
import time
from scipy.interpolate import interp1d
import scipy.integrate
import math

class surge_sway:
    def __init__(self):

        rospy.init_node("Surge_Sway",anonymous=False)
        self.rate = rospy.Rate(10)
        self.publisher  = rospy.Publisher("/rosetta/dolphins", dolphins,queue_size=1000)
        self.accdata=rospy.Subscriber("/calypso_sim/imu/data",Imu, self.talker2)
        
        #Dolphin Variables
        self.throttle = 0
        self.x_pos_pwm = 0
        self.x_neg_pwm = 0
        self.y_pos_pwm = 0
        self.y_neg_pwm = 0
        self.yaw_pos_pwm = 0
        self.yaw_neg_pwm = 0

        #Final Pts.
        self.x_desired = 0
        self.y_desired = 0

        #Accleration Variables
        self.time_lapsed = 0
        self.x_disp = 0
        self.y_disp = 0
        self.prev_throttle_x = 0
        self.prev_throttle_y = 0
        self.i_throttle = 0
        self.prev_acc_x = 0
        self.prev_acc_y = 0

        #PID Values
        self.kp_x = 1
        self.ki_x = 0
        self.kd_x = 0   
        self.kp_y = 1
        self.ki_y = 0
        self.kd_y = 0
        
        self.m = interp1d([0, 30],[1600,2000])
        self.n = interp1d([-30, 0], [1200,1500])
        self.start_time = time.time()

        self.time = []
        self.acc_x = []
        self.acc_y = []
        self.vel_x = []
        self.vel_y = []
        

    def start(self):
        
        self.x_desired = int(input("Enter x_desired: "))
        self.y_desired = int(input("Enter y_desired: "))

        while not rospy.is_shutdown():

            end_time = time.time()

            #Appending Time
            self.time_lapsed = end_time - self.start_time
            self.time.append(self.time_lapsed)

            #Appending Acceleration
            self.acc_x.append(self.acc_imu_x)
            self.acc_y.append(self.acc_imu_y)

            #To get velocity
            temp_vel_x = self.integrate(self.acc_x, self.time)
            temp_vel_y = self.integrate(self.acc_y, self.time)
            self.vel_x.append(temp_vel_x)
            self.vel_y.append(temp_vel_y)

            #To get displacement
            self.x_disp = self.integrate(self.vel_x, self.time)
            self.y_disp = self.integrate(self.vel_y, self.time)
            print("Displacement in X: ", self.x_disp)
            print("Displacement in Y: ", self.y_disp)

            
            x_pwm = self.getPID_xy(self.x_disp, self.x_desired, self.i_throttle, self.prev_throttle_x)
            if x_pwm >1500:
                self.x_pos_pwm = x_pwm
            else:
                self.x_neg_pwm = x_pwm

            y_pwm = self.getPID_xy(self.y_disp, self.y_desired, self.i_throttle, self.prev_throttle_y)
            if y_pwm > 1500:
                self.y_pos_pwm = y_pwm 
            else: 
                self.y_neg_pwm = y_pwm

            self.g = dolphins()

            self.g.d1 = int(self.throttle + self.x_neg_pwm + self.y_neg_pwm + self.yaw_pos_pwm)
            self.g.d2 = int(self.throttle + self.x_neg_pwm + self.y_pos_pwm + self.yaw_neg_pwm)
            self.g.d3 = int(self.throttle + self.x_pos_pwm + self.y_neg_pwm + self.yaw_pos_pwm)
            self.g.d4 = int(self.throttle + self.x_pos_pwm + self.y_pos_pwm + self.yaw_neg_pwm)

            self.rate.sleep()

    def talker2(self, Imu):
        self.acc_imu_x = Imu.linear_acceleration.x
        self.acc_imu_y = Imu.linear_acceleration.y

    def integrate(self, y, x):
        return scipy.integrate.trapz(y, x)
        
    def getPID_xy(self, actual, desired, pid_i, previous_error):
  
        error = desired - actual
        pid_p = self.kp_x*error
        pid_i = pid_i + error
        pid_d = self.kd_x*(error - previous_error)
        if pid_i>max(30-pid_p-pid_d, 0):
            pid_i = max(30-pid_p-pid_d,0)
        elif pid_i<min(-30-pid_i-pid_d, 0):
            pid_i = min(-30-pid_p-pid_d,0)
        pid_i_final = self.ki_x*pid_i
        PID = pid_p + (pid_i_final + pid_d)/self.time_lapsed

        if(PID > 30):
            PID=30
        if(PID < -30):
            PID=-30

        previous_error = error
        print("PID values: ", PID)
        return PID
    
    def getPID_yaw(self, actual, desired, pid_i, previous_error):
  
        #Pruthvi Ad your yaw PID Code here    
        return
    
    def set_1500(self):
        self.x_pos_pwm = 1500
        self.x_neg_pwm = 1500
        self.y_pos_pwm = 1500
        self.y_neg_pwm = 1500

if __name__=='__main__':
    try:
        x = surge_sway()
        x.start()
    except rospy.ROSInterruptException:
        pass
