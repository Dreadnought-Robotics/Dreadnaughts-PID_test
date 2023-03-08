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
        self.yaw_desired = 0
        self.pid_i_yaw = 0

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
        self.n = interp1d([-30, 30], [0,158])
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

            self.g = dolphins()

            self.set_zero()

            #X-Displacement PID code
            x_pwm = self.getPID_xy(self.x_disp, self.x_desired, self.pid_i_x)
            if x_pwm >0:
                self.x_pos_pwm = self.m(x_pwm)
            else:
                self.x_pos_pwm = self.m(-x_pwm)

            #Y-Displacement PID code
            y_pwm = self.getPID_xy(self.y_disp, self.y_desired, self.pid_i_y)
            if y_pwm > 0:
                self.y_pos_pwm = self.m(y_pwm) 
            else: 
                self.y_pos_pwm = self.m(-y_pwm)

            #Yaw PID Code
            yaw_temp = self.getPID_yaw(self.yaw, self.yaw_desired, self.pid_i_yaw, self.angvel_z)
        
            if(yaw_temp > self.yaw):
                self.yaw_pos_pwm = self.n(yaw_temp)
            else:
                self.yaw_neg_pwm = self.n(yaw_temp)

            self.g.d1 = int(self.throttle + self.x_neg_pwm + self.y_pos_pwm + self.yaw_pos_pwm)
            self.g.d2 = int(self.throttle + self.x_neg_pwm + self.y_neg_pwm + self.yaw_neg_pwm)
            self.g.d3 = int(self.throttle + self.x_pos_pwm + self.y_neg_pwm + self.yaw_pos_pwm)
            self.g.d4 = int(self.throttle + self.x_pos_pwm + self.y_pos_pwm + self.yaw_neg_pwm)
            
            self.publisher.publish(self.g)
            self.rate.sleep()

    def talker(self, buoy):
        self.yaw = buoy.yaw

    def talker2(self, Imu):
        self.angvel_z = Imu.angular_velocity.z
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

        # if pid_i>max(50-pid_p-pid_d, 0):
        #     pid_i = max(50-pid_p-pid_d,0)
        # elif pid_i<min(-50-pid_i-pid_d, 0):
        #     pid_i = min(-50-pid_p-pid_d,0)

        pid_i_final = self.ki_x*pid_i
        PID = pid_p + pid_i_final + pid_d

        if(PID > 50):
            PID=50
        if(PID < -50):
            PID=-50
        print("PID values: ", PID)
        return PID
    
    def getPID_yaw(self, actual, desired, pid_i, feedforward):

        error = desired - actual
        pid_p = self.kp_yaw*error
        pid_i = pid_i + error
        previous_error = error
        pid_d = self.kd_yaw*(error - previous_error)

        if pid_i>max(34.5-pid_p-pid_d, 0):
            pid_i = max(34.5-pid_p-pid_d,0)
        elif pid_i<min(-34.5-pid_i-pid_d, 0):
            pid_i = min(-34.5-pid_p-pid_d,0)

        pid_i_final = self.ki_yaw*pid_i

        PID = pid_p + (pid_i_final + pid_d)/self.time_lapsed + feedforward

        if(PID > 30):
            PID=30
        if(PID < -30):
            PID=-30

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
