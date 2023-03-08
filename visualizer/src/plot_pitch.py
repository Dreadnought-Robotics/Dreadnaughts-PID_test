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
      self.fig.suptitle('pitch')
      self.ln, = plt.plot([],[],color='r')
      self.x_data, self.y_pitch_data= [] , [] 

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

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    self.y_pitch_data.append(math.degrees(math.asin(t2)))

    self.x_data.append(t)
    t=t+1
  
  def update_plot(self, frame):
      self.ax.set_xlim(self.y_pitch_data[0], self.y_pitch_data[-1])
      self.ax.set_ylim(self.x_data[0], self.x_data[-1])
      # self.ln.set_data(np.array(self.x_data),np.array(self.y_pitch_data))
      self.ln.set_data(self.y_pitch_data,self.x_data)
      return self.ln


rospy.init_node('vis_pitch_node')
vis = Visualiser()
sub = rospy.Subscriber('/calypso_sim/imu/data',Imu, vis.callback)
ani = FuncAnimation(vis.fig, vis.update_plot, init_func=vis.plot_init,interval = (1/20)*1000,cache_frame_data=False)
plt.show() 
