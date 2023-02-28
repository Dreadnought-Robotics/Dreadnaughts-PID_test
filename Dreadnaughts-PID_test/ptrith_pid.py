#! /usr/bin/python3
import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Int64
from calypso_msgs.msg import buoy
from calypso_msgs.msg import gypseas
from calypso_msgs.msg import dolphins
import pickle
import math

class pid:

  def _init_(self):

    rospy.init_node('calypso_pid', anonymous=False)
    self.kp_yaw = 1
    self.kd_yaw = 0
    self.ki_yaw = 0
    
    # To control Yaw	
    self.pid_i_yaw = 0
    self.previous_error_yaw = 0

    # To move forward/backward
    self.ki_surge  = 0
    self.kd_surge = 0
    self.kp_surge = 2

    self.pid_i_surge = 0
    self.previous_error_surge = 0
    
    self.throttle = 1600
    self.rate = rospy.Rate(10)
    self.pwm_surge_speed = rospy.Publisher('/rosetta/dolphins', dolphins, queue_size = 1000)
    
    self.rate = rospy.Rate(10)

    self.w=0
    self.x=0
    self.y=0
    self.z=0
  
  def start(self):

    while not rospy.is_shutdown():

      self.dolphins=rospy.Subscriber("/rosetta/imu/data",buoy, self.talker)
      
      self.yaw = self.convert()
      self.surge = self.convert_linear()

      self.PID_yaw = self.getPID(self.kd_yaw, self.ki_yaw, self.kp_yaw, self.yaw, 0, self.pid_i_yaw, self.previous_error_yaw)

      #Move the AUV forward
      self.PID_surge = self.getPID_linear(self.kd_surge, self.ki_surge, self.kp_surge, 5, 0, self.pid_i_surge, self.previous_error_surge)

      self.s=dolphins()
      self.s.d1 = int(self.throttle + self.PID_surge)
      self.s.d2 = int(self.throttle + self.PID_surge)
      self.s.d3 = int(self.throttle + self.PID_surge)
      self.s.d4 = int(self.throttle + self.PID_surge)
      
      self.pwm_surge_speed.publish(self.swhatsa)
      self.pwmspeed.publish(self.g)
      
      self.rate.sleep()

  def getPID(self, kd, ki, kp, actual, desired, pid_i, previous_error):
  
    error = actual - desired
    pid_p = kp*error
    pid_i = pid_i + (ki*error)
    pid_d = kd*(error - previous_error)

    PID = pid_p + pid_i + pid_d

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
    self.Z = math.degrees(math.atan2(2.0*self.y*self.z + self.w*self.x), self.w*self.w - self.x*self.x - self.y*self.y + self.z*self.z);
    return self.Z

  def convert_linear(self):
    return math.sqrt(self.x*self.x + self.y*self.y)
  
  def getPID_linear(self, kd, ki, kp, actual, desired, pid_i, previous_error):
    
    error = actual - desired
    pid_p = kp*error
    pid_i = pid_i + (ki*error)
    pid_d = kd*(error - previous_error)
    previous_error = error
    PID = pid_p + pid_i + pid_d
    
    return PID


if __name__=='__main__':
  try:
      x = pid()
      x.start()
  except rospy.ROSInterruptException:
      pass
