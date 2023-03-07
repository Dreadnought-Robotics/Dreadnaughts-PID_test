#! /usr/bin/python3

import rospy
from calypso_msgs.msg import dolphins
from calypso_msgs.msg import buoy
from sensor_msgs.msg import Imu
import time
import math
from scipy.interpolate import interp1d
from scipy.integrate import trapezoid


class y:

  def __init__(self):

    rospy.init_node('yaw', anonymous=False)

    self.kp_yaw = 20
    self.kd_yaw = 40
    self.ki_yaw = 0

    self.pid_i_yaw = 0

    self.kp = 15
    self.ki = 0 
    self.kd = 30

    self.previous_throttle = 0
    self.integrator_throttle = 0

    self.throttle1 = 1540
    self.throttle2 = 1540
    self.throttle3 = 1540
    self.throttle4 = 1540
    self.displacement = 0
    self.time_lapsed = 0

    self.rate = rospy.Rate(10)
    self.pwmspeed = rospy.Publisher('/rosetta/dolphins', dolphins, queue_size=1000)
    self.dolphins=rospy.Subscriber("/rosetta/imu/data",buoy, self.talker)
    self.angvel = rospy.Subscriber("/calypso_sim/imu/data", Imu, self.getvel)
    
    self.w=0
    self.x=0
    self.y=0
    self.z=0
    self.m = interp1d([-30, 30],[1158,2000])
    self.n = interp1d([-30, 0], [1200,1468])

    self.x_coordinate = 1
    self.y_coordinate = 1
    self.req_yaw = math.degrees(math.atan2(self.y_coordinate, self.x_coordinate))
    # self.req_yaw = 0
    self.angvel_x = 0
    self.angvel_y = 0 
    self.angvel_z = 0
    self.acc_z = 0
    self.acc_x = 0
    self.ang_y = 0 


  def start(self):

    self.roll , self.pitch , self.yaw = self.convert()
    self.yaw_init = self.yaw
    print("required yaw")
    print(math.sqrt(math.pow(self.req_yaw - (self.yaw - self.yaw_init), 2)))
    self.g=dolphins()

    while not rospy.is_shutdown():
      while math.sqrt(math.pow(self.req_yaw - (self.yaw - self.yaw_init), 2)) > 0.2:
    
        # self.gypseas=rospy.Subscriber("/calypso_pid/topple_checker",dolphins, self.getgyp)
        self.roll , self.pitch , self.yaw = self.convert()
        print("yaw")
        print(self.yaw)
        self.error_yaw = self.req_yaw - (self.yaw - self.yaw_init)
        self.PID_yaw = self.getPID(self.kd_yaw, self.ki_yaw, self.kp_yaw, self.error_yaw, 0, self.pid_i_yaw, self.angvel_z)
  
        self.g.d1 = round(self.throttle1 - self.PID_yaw)
        self.g.d2 = round(self.throttle2 + self.PID_yaw)
        self.g.d3 = round(self.throttle3 - self.PID_yaw)
        self.g.d4 = round(self.throttle4 + self.PID_yaw)
      
        print("PID-yaw")
        print(self.PID_yaw)

        self.pwmspeed.publish(self.g)
      
        self.rate.sleep()
        
      self.g.d1 = round(1540)
      self.g.d2 = round(1540)
      self.g.d3 = round(1540)
      self.g.d4 = round(1540)

      self.pwmspeed.publish(self.g)
      self.error_dist = math.sqrt((self.y_coordinate * self.y_coordinate) + (self.x_coordinate * self.x_coordinate))
      self.start_time = time.time()
      self.time = []
      self.acc = []
      self.vel = []
      while self.error_dist > 0:
        end_time = time.time()
        self.time_lapsed = end_time - self.start_time
        self.time.append(self.time_lapsed)
        self.acc.append(self.acc_x)
        self.vel_x = self.integrate(self.acc, self.time)
        self.vel.append(self.vel_x)
        self.displacement = self.integrate(self.vel, self.time)
        print("displacement")
        print(self.displacement)

        self.throttle_to_map = self.getPID_mo(self.kd, self.ki, self.kp, self.displacement, self.error_dist, self.integrator_throttle, self.previous_throttle, self.acc_x)
        if self.throttle_to_map>=0:
            self.throttle_rear = self.m(self.throttle_to_map)
            self.throttle_fro = self.n(-self.throttle_to_map)
        else:
            self.throttle_fro = self.m(self.throttle_to_map)
            self.throttle_rear = self.n(-self.throttle_to_map)
        print("map")
        print(self.throttle_to_map)
        self.g = dolphins()
        print("acc")
        print(self.acc_x)

        self.g.d1 = int(self.throttle_fro)
        self.g.d2 = int(self.throttle_fro)
        self.g.d3 = int(self.throttle_rear)
        self.g.d4 = int(self.throttle_rear)  
        self.pwmspeed.publish(self.g)
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
  
  def getPID(self, kd, ki, kp, actual, desired, pid_i, angvel):
  
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
      return PID
  

  def integrate(self, y, x):
    return trapezoid(y, x)
  
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
  

  def getvel(self, Imu):
    self.angvel_x = Imu.angular_velocity.x
    self.angvel_y = Imu.angular_velocity.y
    self.angvel_z = Imu.angular_velocity.z
    self.acc_x = Imu.linear_acceleration.x
    self.acc_y = Imu.linear_acceleration.y
    self.acc_z = Imu.linear_acceleration.z

  def talker(self,buoy):

    self.x = buoy.x
    self.y = buoy.y
    self.z = buoy.z    
    self.w = buoy.w

if __name__=='__main__':
    try:
        x = y()
        x.start()
    except rospy.ROSInterruptException:
        pass