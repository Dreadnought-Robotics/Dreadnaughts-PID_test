#! /usr/bin/python3
import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Int64
from thruster.msg import buoy
from thruster.msg import dolphins
import pickle
import math

class pid:
	def _init_(self):
		rospy.init_node('calypso_pid', anonymous=False)
		
		self.kp_yaw = 1
		self.kd_yaw = 0
		self.ki_yaw = 0
    	
		self.pid_i_yaw = 0
		self.previous_error_yaw = 0
	
		self.throttle = 1560 
		self.rate = rospy.Rate(10)
		self.pwm_speed = rospy.Publisher('/rosetta/dolphins', dolphins, queue_size = 1000)
		self.rate = rospy.Rate(10)
		
		self.x=0
		self.y=0
		self.z=0
		self.w=0
		self.Z=0
    
	def start(self):
		while not rospy.is_shutdown():
			self.dolphins=rospy.Subscriber("/rosetta/imu/data",buoy, self.talker)
			self.yaw = self.convert()
			self.PID_yaw = self.getPID(self.kd_yaw, self.ki_yaw, self.kp_yaw, self.yaw, 0, self.pid_i_yaw, self.previous_error_yaw)
			
			self.s=dolphins()
			self.s.d1 = int(self.throttle + self.PID_yaw)
			self.s.d2 = int(self.throttle + self.PID_yaw)
			self.s.d3 = int(self.throttle + self.PID_yaw)
			self.s.d4 = int(self.throttle + self.PID_yaw)
			
			print("PID-Yaw")
			print(self.PID_yaw)
			self.pwmspeed.publish(self.s)
			self.rate.sleep()
			
	def getPID(self, kd, ki, kp, actual, desired, pid_i, previous_error):
  		
		error = actual - desired
		pid_p = kp*error
		pid_i = pid_i + (ki*error)
		pid_d = kd*(error - previous_error)
	      
		PID = pid_p + pid_i + pid_d
		  
		if(PID > 34.5):
			PID = 34.5
		if(PID < -34.5):
			PID = -34.5
		
		previous_error = error
		return PID
	
	def talker(self,buoy):
		
		self.x = buoy.x
		self.y = buoy.y
		self.z = buoy.z    
		self.w = buoy.w
		
	def convert(self):
		t3 = +2.0 * (self.w * self.z +self.x * self.y)
		t4 = +1.0 - 2.0 * (self.y * self.y + self.z * self.z)
		self.Z = math.degrees(math.atan2(t3, t4))
		
		return self.Z
	
	def getdol(self, dolphins):
		self.throttle = dolphins.d1 = dolphins.d2 = dolphins.d3 = dolphins.d4
	
if __name__=='__main__':
	
	try:
		x = pid()
		x.start()
	except rospy.ROSInterruptException:
		pass
