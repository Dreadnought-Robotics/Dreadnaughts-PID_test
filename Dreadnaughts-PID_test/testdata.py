import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Int64
from calypso_msgs.msg import buoy
from std_msgs.msg import String
from sensor_msgs.msg import Imu
import pickle
import math


class rosetta :
  def __init__(self):

    rospy.init_node('bentest_data', anonymous=False)
    print(rospy.get_name())
    
    self.check = True
    self.x=0
    self.y=0
    self.z=0

    self.imu_pub = rospy.Publisher('testdata', buoy, queue_size=1000)
    self.pos = rospy.Subscriber('test_complete', String, self.check)
    self.rate = rospy.Rate(10)
    
  def check(self):
    self.check = String.test

  def start(self):

    while check:

      self.b=buoy()
      self.x = int(input("Enter x: "))
      self.y = int(input("Enter y: "))
      self.b.x=self.x
      self.b.y=self.y

      self.imu_pub.publish(self.b)
      
      self.rate.sleep()

if __name__=='__main__':

  try:
      x = rosetta()
      x.start()
  except rospy.ROSInterruptException:
      pass


  