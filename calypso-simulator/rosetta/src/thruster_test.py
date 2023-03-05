#! /usr/bin/python3
from calypso_msgs.msg import gypseas
from calypso_msgs.msg import dolphins
import rospy

if __name__ == '__main__':

  rospy.init_node("rosetta_test",anonymous=False)
  rate = rospy.Rate(10)
  # gpub=rospy.Publisher("/calypso_pid/topple_checker",gypseas,queue_size=1000)
  gpub=rospy.Publisher("/rosetta/gypseas",gypseas,queue_size=1000)

  dpub=rospy.Publisher("/rosetta/dolphins",dolphins,queue_size=1000)
  g=gypseas()
  g.t1=1540
  g.t2=1540
  g.t3=1540
  g.t4=1540
  d=dolphins()
  d.d1=1540
  d.d2=1650 
  d.d3=1540
  d.d4=1650
  while(True):
    gpub.publish(g)
    dpub.publish(d)
    print("done")
    rate.sleep()




