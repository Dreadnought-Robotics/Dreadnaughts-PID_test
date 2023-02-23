#! /usr/bin/python3
import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Int64
from calypso_msgs.msg import buoy
from calypso_msgs.msg import dolphins
from calypso_msgs.msg import gypseas
from sensor_msgs.msg import Imu
import time
import math

class movement:
    def __init__(self):
        rospy.init_node('calypso_movement', anonymous=False)

        self.pwmspeed_dol = rospy.Publisher('/rosetta/dolphins', dolphins, queue_size=1000)
        self.pwmspeed_gyp = rospy.Publisher('/rosetta/gypseas', gypseas, queue_size=1000)
        self.rate = rospy.Rate(10)

        self.time_lapsed = 0

        self.desired_altitude = 7
        self.altitude = 0
        self.xdrift = 0
        self.ydrift = 0
        
        self.throttle = 1580
        self.kp_throttle = 0
        self.kd_throttle = 0
        self.ki_throttle = 0
        self.previous_throttle = 0
        self.integrator_throttle = 0

        self.kp_pitch = 0
        self.kd_pitch = 0
        self.ki_pitch = 0
        self.previous_pitch = 0
        self.integrator_pitch = 0

        self.kp_roll= 0
        self.kd_roll = 0
        self.ki_roll = 0
        self.previous_roll = 0
        self.integrator_roll = 0

        self.kp_yaw = 0
        self.kd_yaw = 0
        self.ki_yaw = 0
        self.previous_yaw = 0
        self.integrator_yaw = 0

        self.w=0
        self.x=0
        self.y=0
        self.z=0

        self.acc_z = 0
        self.acc_x = 0
        self.acc_y = 0
        self.previous_vel_z = 0
        self.previous_vel_y = 0
        self.previous_vel_x = 0

    def start(self):
        while not rospy.is_shutdown():
            
            self.imudata=rospy.Subscriber("/rosetta/imu/data",buoy, self.talker1)
            self.accdata=rospy.Subscriber("/calypso_sim/imu/data",Imu, self.talker2)

            start_time = time.time()
            self.roll , self.pitch , self.yaw = self.convert()

            self.altitude = self.getposition(self.acc_z, self.previous_vel_z, self.altitude)
            self.xdrift = self.getposition(self.acc_x, self.previous_vel_x, self.xdrift)
            self.ydrift = self.getposition(self.acc_y, self.previous_vel_y, self.ydrift)
            
            self.desired_yaw = self.getback()
            
            self.throttle = self.getPID(self.kp_throttle, self.ki_throttle, self.kd_throttle, self.altitude, self.desired_altitude, self.previous_throttle, 300, 300, self.integrator_throttle)
            self.pid_roll = self.getPID(self.kp_roll, self.ki_roll, self.kd_roll, self.roll, 0, self.previous_roll, -10, 10, self.integrator_roll)
            self.pid_yaw = self.getPID(self.kp_yaw, self.ki_yaw, self.kd_yaw, self.yaw, self.desired_yaw, self.previous_yaw, -10, 10, self.integrator_yaw)
            self.pid_pitch = self.getPID(self.kp_pitch, self.ki_pitch, self.kd_pitch, self.pitch, 0, self.previous_pitch, -10, 10, self.integrator_pitch)

            self.d = dolphins()
            self.g = gypseas()

            self.g.t1 = int(self.throttle - self.pid_pitch - self.pid_roll)
            self.g.t2 = int(self.throttle - self.pid_pitch + self.pid_roll)
            self.g.t3 = int(self.throttle + self.pid_pitch + self.pid_roll)
            self.g.t4 = int(self.throttle + self.pid_pitch - self.pid_roll)

            self.d.d1 = int(self.throttle + self.pid_yaw)
            self.d.d2 = int(self.throttle - self.pid_yaw)
            self.d.d3 = int(self.throttle + self.pid_yaw)
            self.d.d4 = int(self.throttle - self.pid_yaw)


            self.pwmspeed_dol.publish(self.d)
            self.pwmspeed_gyp.publish(self.g)
            self.rate.sleep()
            end_time = time.time()
            self.time_lapsed = end_time - start_time
            
    
    def getposition(self, acc, previous_vel, previous_distance):
        vel = previous_vel + acc*(self.time_lapsed)
        distance = vel*(self.time_lapsed) + previous_distance
        previous_vel = vel
        return distance

    def clamp(self, num, min, max):
        return min if num < min else max if num > max else num

    def getPID(self, kp, ki, kd, actual, desired, previous_error, min, max, integrator):
        
        error = actual - desired
        p_pid = kp*error

        d_pid = kd*(error - previous_error)

        integrator = integrator + error
        self.clamp(integrator, min, max)
        i_pid = ki*integrator
        previous_error = error
        return p_pid + d_pid + i_pid

    def talker1(self, buoy):
        self.x = buoy.x
        self.y = buoy.y
        self.z = buoy.z    
        self.w = buoy.w

    def talker2(self, Imu):
        self.acc_x = Imu.linear_acceleration.x
        self.acc_y = Imu.linear_acceleration.y
        self.acc_z = Imu.linear_acceleration.z

    def getback(self):
        if self.xdrift == 0 and self.ydrift == 0:
            return 0
        theta = math.degrees(math.atan(self.ydrift/self.xdrift))
        yaw = 180 + theta
        return math.degrees(yaw)
        
    def convert(self):

        t0 = +2.0 * (self.w * self.x + self.y * self.z)
        t1 = +1.0 - 2.0 * (self.x * self.x + self.y * self.y)
        self.X = math.degrees(math.atan2(t0, t1))

        t2 = +2.0 * (self.w * self.y - self.z * self.x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        self.Y = math.degrees(math.asin(t2))

        t3 = +2.0 * (self.w * self.z +self.x * self.y)
        t4 = +1.0 - 2.0 * (self.y * self.y + self.z * self.z)
        self.Z = math.degrees(math.atan2(t3, t4))

        return self.X, self.Y, self.Z

if __name__=='__main__':

    try:
        x = movement()
        x.start()
    except rospy.ROSInterruptException:
        pass
