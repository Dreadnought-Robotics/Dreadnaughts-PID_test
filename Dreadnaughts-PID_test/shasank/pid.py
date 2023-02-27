#! /usr/bin/python3
import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Int64
from calypso_msgs.msg import buoy
from calypso_msgs.msg import gypseas
import pickle
import math


class pid:

  def __init__(self):

    rospy.init_node('calypso_pid', anonymous=False)

    self.kp_pitch = 30
    self.kd_pitch =   0
    self.ki_pitch = 0
    self.kp_roll = 30
    self.kd_roll = 0
    self.ki_roll = 0

    self.pid_i_pitch = 0
    self.pid_i_roll = 0
    self.previous_error_pitch = 0
    self.previous_error_roll = 0

    self.throttle1 = 0
    self.throttle2 = 0
    self.throttle3 = 0
    self.throttle4 = 0
    self.rate = rospy.Rate(10)
    self.pwmspeed = rospy.Publisher('/rosetta/gypseas', gypseas, queue_size=1000)
    
    self.w=0
    self.x=0
    self.y=0
    self.z=0
  
  def start(self):

    while not rospy.is_shutdown():

      self.dolphins=rospy.Subscriber("/rosetta/imu/data",buoy, self.talker)
      self.gypseas=rospy.Subscriber("/calypso_pid/topple_checker",gypseas, self.getgyp)
      self.roll , self.pitch , self.yaw = self.convert()
      print("pitch")
      print(self.pitch)
      self.PID_pitch = self.getPID(self.kd_pitch, self.ki_pitch, self.kp_pitch, self.pitch, 0, self.pid_i_pitch, self.previous_error_pitch)
      # self.PID_roll = self.getPID(self.kd_roll, self.ki_roll, self.kp_roll, self.roll, 0, self.pid_i_roll, self.previous_error_roll)
      
      self.g=gypseas()
      self.g.t1 = int(self.throttle1 + self.PID_pitch)# - self.PID_roll)
      self.g.t2 = int(self.throttle2 + self.PID_pitch)# + self.PID_roll)
      self.g.t3 = int(self.throttle3 - self.PID_pitch)# + self.PID_roll)
      self.g.t4 = int(self.throttle4 - self.PID_pitch)# - self.PID_roll)
      
      print("PID-pitch")
      print(self.PID_pitch)
      
      self.pwmspeed.publish(self.g)
      
      
      self.rate.sleep()

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
  
  def talker(self,buoy):

    self.x = buoy.x
    self.y = buoy.y
    self.z = buoy.z    
    self.w = buoy.w
    
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
  
  def getgyp(self, gypseas):
    self.throttle1 = gypseas.t1
    self.throttle2 = gypseas.t2
    self.throttle3 = gypseas.t3
    self.throttle4 = gypseas.t4

if __name__=='__main__':

  try:
      x = pid()
      x.start()
  except rospy.ROSInterruptException:
      pass