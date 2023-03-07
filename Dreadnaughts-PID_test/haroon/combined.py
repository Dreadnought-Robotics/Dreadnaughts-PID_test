#! /usr/bin/python3
import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Int64
from calypso_msgs.msg import buoy
from calypso_msgs.msg import gypseas
from calypso_msgs.msg import dolphins
import pickle
import math
from scipy.integrate import trapezoid

class pid:

  def _init_(self):

    rospy.init_node('calypso_pid', anonymous=False)

    self.kp_yaw = 2
    self.kd_yaw = 0
    self.ki_yaw = 0

    self.kp_linear=2
    self.kd_linear=0
    self.ki_linear=0
    
    self.pid_i_yaw = 0
    self.pid_i_horizontal=0
    self.pid_i_vertical=0
    self.previous_error_yaw = 0
    self.previous_error_linear=0
    
    self.throttle = 1540
    self.rate = rospy.Rate(10)
    self.pwm_surge_speed = rospy.Publisher('/rosetta/dolphins', dolphins, queue_size = 1000)

    self.x=0
    self.y=0
    self.yaw=0

    self.current_time=0
    self.time=[0]
    self.acc_x=[]
    self.vel_x=[]
    self.acc_y=[]
    self.vel_y=[]

    self.cord=(1,1,15)

  
  def integrate(self, y, x):
        return trapezoid(y, x)

  def PID(self, kd, ki, kp, actual, desired, pid_i, previous_error, feedforward,clamp_min=None,clamp_max=None):
    
    self.imu_data=rospy.Subscriber("/rosetta/imu/data",buoy, self.talker)

    error = desired - actual
    pid_p = kp*error
    pid_d = kd*(error - previous_error)

    if(clamp_min or clamp_max):
      if(clamp_min < error < clamp_max):
        pid_i = pid_i + (ki*error)
    
    val = pid_p + (pid_i + pid_d)/self.time_lapsed + feedforward

    if(val > 300):
        val=300
    if(val < -300):
        val=-300

    previous_error = error
    return val
  
  def get_pose(self,buoy):

    self.w = buoy.w
    self.acc_x.append(buoy.acc_x)
    self.vel_x.append(self.integrate(self.acc_x, self.time))
    self.acc_y.append(buoy.acc_y)
    self.vel_y.append(self.integrate(self.acc_y, self.time))
    self.x = self.integrate(self.vel_x, self.time)
    self.y = self.integrate(self.vel_y, self.time)
    self.yaw=math.degrees(buoy.yaw)
    self.time.append(self.time[-1]+0.1)
  
  def publish(self,yaw,vertical,horizontal):
     
    # d1=t+yl+b+r
    # d2=t+yr+b+l
    # d3=t+yl+f+l
    # d4=t+yr+f+r
    # yaw clamp -7 to 7
    s=dolphins()
    s.d1=self.thrust-yaw-vertical+horizontal
    s.d2=self.thrust+yaw-vertical-horizontal
    s.d3=self.thrust-yaw+vertical-horizontal
    s.d4=self.thrust+yaw+vertical+horizontal

  def start(self):

    while not rospy.is_shutdown():
      
      self.yaw = self.convert()
      self.surge = self.convert_linear()

      PID_yaw = self.PID(self.kd_yaw, self.ki_yaw, self.kp_yaw, self.yaw, self.cord[2], self.pid_i_yaw, self.previous_error_yaw,,-7,7)
 
      PID_vertical = self.PID(self.kd_linear, self.ki_linear, self.kp_linear, self.x, self.cord[0] , self.pid_i_vertical, self.previous_error_vertical)

      PID_horizontal= self.PID(self.kd_linear, self.ki_linear, self.kp_linear, self.x, self.cord[0] , self.pid_i_horizontal, self.previous_error_horizontal)
      
      self.publish(PID_yaw,PID_vertical,PID_horizontal)
      
      self.rate.sleep()


if __name__=='__main__':

  try:
      x = pid()
      x.start()
  except rospy.ROSInterruptException:
      pass
