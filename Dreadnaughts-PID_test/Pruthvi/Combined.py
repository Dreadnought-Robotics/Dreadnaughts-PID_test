#!usr/bin/python3

import rospy
from calypso_msgs.msg import dolphins
from sensor_msgs.msg import Imu
from calypso_msgs.msg import buoy
import time
from scipy.interpolate import interp1d
import scipy.integrate
import math

class surge_sway:
    def __init__(self):

        rospy.init_node("Surge_Sway",anonymous=False)
        self.rate = rospy.Rate(10)
        self.publisher  = rospy.Publisher("/rosetta/dolphins", dolphins,queue_size=1000)
        self.dolphins=rospy.Subscriber("/rosetta/imu/data",buoy, self.talker)
        self.accdata=rospy.Subscriber("/calypso_sim/imu/data",Imu, self.talker2)
        
        #Dolphin Variables
        self.throttle = 1500
        self.x_pos_pwm = 0
        self.x_neg_pwm = 0

        #Yaw 
        self.yaw = 0
        self.kp_yaw = 20
        self.kd_yaw = 40
        self.ki_yaw = 0

        #PWM variables 
        self.y_pos_pwm = 0
        self.y_neg_pwm = 0
        self.yaw_pos_pwm = 1580 #Test_Value
        self.yaw_neg_pwm = 1510 #Test_Value

        #Final Pts.
        self.x_desired = 0
        self.y_desired = 0

        #Acceleration Variables
        self.time_lapsed = 0
        self.x_disp = 0
        self.y_disp = 0
        self.prev_acc_x = 0
        self.prev_acc_y = 0
        self.acc_imu_x = 0
        self.acc_imu_y = 0

        #PID Values
        self.kp_x = 2
        self.ki_x = 0
        self.kd_x = 0   
        self.kp_y = 1
        self.ki_y = 0
        self.kd_y = 0
        self.pid_i_x = 0
        self.pid_i_y = 0 
        
        self.m = interp1d([0, 50],[100,300])
        # self.n = interp1d([-50, 0], [100,1500])
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

            self.set_zero()
            x_pwm = self.getPID_xy(self.x_disp, self.x_desired, self.pid_i_x)
            if x_pwm >0:
                self.x_pos_pwm = self.m(x_pwm)
            else:
                self.x_pos_pwm = self.m(-x_pwm)

            y_pwm = self.getPID_xy(self.y_disp, self.y_desired, self.pid_i_y)
            if y_pwm > 0:
                self.y_pos_pwm = self.m(y_pwm) 
            else: 
                self.y_pos_pwm = self.m(-y_pwm)

            #Call your yaw function here to a temp variable. Then check the direction to yaw and set the pwm for that
            self.yaw = self.convert()
            self.PID_yaw = self.getPID(self.kd_yaw, self.ki_yaw, self.kp_yaw, self.yaw, self.desired_yaw, self.pid_i_yaw, self.previous_error_yaw)
            #Yaw variables are self.yaw_pos_pwm and self.yaw_neg_pwm

            self.g = dolphins()

            #Integrate your variables into these while loops itself
            #I don't know how else to work Yaw
            
            while(self.yaw>self.desired_yaw):
                self.g.d1 = round(self.throttle2 + self.PID_yaw)
                self.g.d2 = round(self.throttle1)
                self.g.d3 = round(self.throttle2 + self.PID_yaw)
                self.g.d4 = round(self.throttle1)
            
            while(self.yaw<self.desired_yaw):
                self.g.d1 = round(self.throttle1)
                self.g.d2 = round(self.throttle2 + self.PID_yaw)
                self.g.d3 = round(self.throttle1)
                self.g.d4 = round(self.throttle2 + self.PID_yaw)
                
            while(self.yaw == self.desired_yaw):
                self.g.d1 = round(self.stable)
                self.g.d2 = round(self.stable)
                self.g.d3 = round(self.stable)
                self.g.d4 = round(self.stable)

            #self.g.d1 = int(self.throttle + self.x_neg_pwm + self.y_pos_pwm + self.yaw_pos_pwm)
            #self.g.d2 = int(self.throttle + self.x_neg_pwm + self.y_neg_pwm + self.yaw_neg_pwm)
            #self.g.d3 = int(self.throttle + self.x_pos_pwm + self.y_neg_pwm + self.yaw_pos_pwm)
            #self.g.d4 = int(self.throttle + self.x_pos_pwm + self.y_pos_pwm + self.yaw_neg_pwm)
            
            self.publisher.publish(self.g)
            self.rate.sleep()

    def talker(self, buoy):
        self.yaw = buoy.yaw

    def talker2(self, Imu):
        self.acc_imu_x = Imu.linear_acceleration.x
        self.acc_imu_y = Imu.linear_acceleration.y

    def integrate(self, y, x):
        return scipy.integrate.trapz(y, x)
        
    def getPID_xy(self, actual, desired, pid_i):
  
        error = desired - actual
        previous_error = error
        pid_p = self.kp_x*error
        pid_d = self.kd_x*(error - previous_error)
        pid_i = pid_i + error

        if pid_i>max(50-pid_p-pid_d, 0):
            pid_i = max(50-pid_p-pid_d,0)
        elif pid_i<min(-50-pid_i-pid_d, 0):
            pid_i = min(-50-pid_p-pid_d,0)

        pid_i_final = self.ki_x*pid_i
        PID = pid_p + pid_i_final + pid_d

        if(PID > 50):
            PID=50
        if(PID < -50):
            PID=-50
        print("PID values: ", PID)
        return PID
    
    def getPID(self, kd, ki, kp, actual, desired, pid_i, previous_error):
        error = actual - desired
        pid_p = kp*error
        
        if pid_i > 10:
            pid_i = 10
        elif pid_i < -10:
            pid_i = -10
        
        pid_i = pid_i + (ki*error)
        pid_d = kd*(error - previous_error)
        
        PID = pid_p + pid_i + pid_d
        
        if(PID > 34.5):
            PID = 34.5
        if(PID < -34.5):
            PID = -34.5
            
        previous_error = error
        
        return PID
    
    def set_zero(self):
        self.x_pos_pwm = 0
        self.x_neg_pwm = 0
        self.y_pos_pwm = 0
        self.y_neg_pwm = 0

if __name__=='__main__':
    try:
        x = surge_sway()
        x.start()
    except rospy.ROSInterruptException:
        pass
