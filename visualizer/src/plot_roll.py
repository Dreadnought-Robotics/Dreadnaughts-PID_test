import matplotlib.pyplot as plt
import rospy
import tf
from sensor_msgs.msg import Imu
import numpy as np
from matplotlib.animation import FuncAnimation
import math


class Visualiser:
    
  def __init__(self):
      
      self.fig, self.ax = plt.subplots()
      self.fig.suptitle('roll')
      self.ln, = plt.plot([],[],color='r')
      self.x_data, self.y_roll_data= [] , [] 

  def plot_init(self):
      
      return self.ln

  def callback(self, imu_data):

    #t=imu_data.header.seq 
    t=0

    w=imu_data.orientation.w
    x=imu_data.orientation.x
    y=imu_data.orientation.y  
    z=imu_data.orientation.z 

    ysqr = y * y

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + ysqr)
    self.y_roll_data.append(math.degrees(math.atan2(t0, t1)))

    self.x_data.append(t)
    t=t+1
  
  def update_plot(self, frame):
      self.ax.set_xlim(self.y_roll_data[0], self.y_roll_data[-1])
      self.ax.set_ylim(self.x_data[0], self.x_data[-1])
      # self.ln.set_data(np.array(self.x_data),np.array(self.y_roll_data))
      self.ln.set_data(self.y_roll_data,self.x_data)
      return self.ln


rospy.init_node('vis_roll_node')
vis = Visualiser()
sub = rospy.Subscriber('/calypso_sim/imu/data',Imu, vis.callback)
ani = FuncAnimation(vis.fig, vis.update_plot, init_func=vis.plot_init,interval = (1/20)*1000,cache_frame_data=False)
plt.show() 
