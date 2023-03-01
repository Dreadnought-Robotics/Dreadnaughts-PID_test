#!/usr/bin/python3.8
from calypso_msgs.msg import gypseas
from calypso_msgs.msg import dolphins
import rospy

if __name__ == '__main__':

  rospy.init_node("rosetta_test",anonymous=False)
  rate = rospy.Rate(10)
  gpub=rospy.Publisher("/calypso_pid/topple_checker",gypseas,queue_size=1000)
  g=gypseas()
  g.t1=1580
  g.t2=1580
  g.t3=1580
  g.t4=1580
  while(True):
    gpub.publish(g)
    print("done")
    rate.sleep()




