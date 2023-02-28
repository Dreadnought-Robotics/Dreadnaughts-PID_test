#! /usr/bin/python3
import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Int64
from std_msgs.msg import String
from calypso_msgs.msg import buoy
from calypso_msgs.msg import dolphins
from calypso_msgs.msg import gypseas
import pickle
import math

class pid:

  def __init__(self):

    rospy.init_node('calypso_pid', anonymous=False)
    # To move forward/backward
    self.ki_surge  = 0.02
    self.kd_surge = 0.2
    self.kp_surge = 2

    self.ki_sway = 0.02
    self.kd_sway = 0.2
    self.kp_sway = 2  

    self.previous_x = 0
    self.previous_y = 0

    self.pid_i_surge = 0
    self.pid_i_sway = 0
    self.previous_error_sway = 0
    self.previous_error_surge = 0

    self.test_desire_x = 0
    self.test_desire_y = 0

    self.x=0
    self.y=0
    
    self.throttle = 1000
    self.rate = rospy.Rate(10)
    self.pwm_surge_speed = rospy.Publisher('/rosetta/dolphins', dolphins, queue_size = 1000)
    self.pwmspeed = rospy.Publisher('/rosetta/gypseas', gypseas, queue_size=1000)
    # self.complete = rospy.Publisher('test_complete',String, queue_size=10)
    
    self.rate = rospy.Rate(100)
  
  def start(self):

    self.test_desire_x = int(input("Enter the location in x: "))
    self.test_desire_y = int(input("Enter the location in y: "))

    while not rospy.is_shutdown():

      self.linearxy=rospy.Subscriber('/rosetta/imu/data', buoy, self.talker)

      #MOve the AUV forward
      self.PID_surge = self.getPID_linear(self.kd_surge, self.ki_surge, self.kp_surge, self.y, self.test_desire_x, self.pid_i_surge, self.previous_error_surge)
      self.PID_sway = self.getPID_linear(self.kd_sway, self.kd_sway, self.kp_sway, self.x, self.test_desire_y, self.pid_i_sway, self.previous_error_sway)
      
      self.s=dolphins()
      
      #To move the AUV up to work surge and sway
      self.g=gypseas()
      self.g.t1 = int(1574)
      self.g.t2 = int(1574)
      self.g.t3 = int(1574)
      self.g.t4 = int(1574)
      self.pwmspeed.publish(self.g)
      
      #To sway (translate in y)
      if self.test_desire_x != self.x:
        self.s.d2 = int(self.throttle + self.PID_sway)
        self.s.d3 = int(self.throttle + self.PID_sway)
        # print("Publishing x values")
        # print(self.x)
        self.pwm_surge_speed.publish(self.s)

      #To surge (translate in x)
      elif self.test_desire_y != self.y:
        self.s.d3 = int(self.throttle + self.PID_surge)
        self.s.d4 = int(self.throttle + self.PID_surge)
        # print("Publishing y values")
        # print(self.y)
        self.pwm_surge_speed.publish(self.s)
      
      self.rate.sleep()

  def talker(self,buoy):
    self.x = round(buoy.x,5)
    self.y = round(buoy.y,5)

  def getPID_linear(self, kd, ki, kp, actual, desired, pid_i, previous_error):

    error = desired - actual
    pid_p = error
    pid_i += error
    pid_d = error - previous_error
    previous_error = error
    PID = kp*pid_p + ki*pid_i + kd*pid_d
    print(error," ",PID)
    return PID


if __name__=='__main__':

  try:
      x = pid()
      x.start()
  except rospy.ROSInterruptException:
      pass
