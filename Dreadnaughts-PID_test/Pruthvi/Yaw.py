#! /usr/bin/python3
import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Int64
from calypso_msgs.msg import buoy
from calypso_msgs.msg import dolphins
from calypso_msgs.msg import gypseas
import math

class pid:
	def __init__(self):
		rospy.init_node('calypso_pid', anonymous=False)
		
		self.kp_yaw = 0.1
		self.kd_yaw = 0.3
		self.ki_yaw = 0
    	
		self.pid_i_yaw = 0
		self.previous_error_yaw = 0
	
		self.throttle = 1574
		self.rate = rospy.Rate(10)
		self.pwm_speed = rospy.Publisher('/rosetta/dolphins', dolphins, queue_size = 1000)		

		self.x=0
		self.y=0
		self.z=0
		self.w=0
		self.Z=0
    
	def start(self):
		while not rospy.is_shutdown():
		
			self.dolphins = rospy.Subscriber("/rosetta/imu/data",buoy, self.talker)

			self.yaw = self.convert()
			self.PID_yaw = self.getPID(self.kd_yaw, self.ki_yaw, self.kp_yaw, self.yaw, 0, self.pid_i_yaw, self.previous_error_yaw)

			self.s = dolphins()
			self.s.d1 = round(self.throttle + self.PID_yaw)
			self.s.d2 = round(self.throttle + self.PID_yaw)
			self.s.d3 = round(self.throttle + self.PID_yaw)
			self.s.d4 = round(self.throttle + self.PID_yaw)
			
			print("Yaw")
			print(self.yaw)
			self.pwm_speed.publish(self.s)
			self.rate.sleep()
			
	def getPID(self, kd, ki, kp, actual, desired, pid_i, previous_error):
  		
		error = actual - desired
		pid_p = kp*error

		if pid_i > 10:
			pid_i = 10
		elif pid_i < -10:
			pid_i = -10

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
		t3 = +2.0 * (self.w * self.z + self.x * self.y)
		t4 = +1.0 - 2.0 * (self.y * self.y + self.z * self.z)
		self.Z = math.degrees(math.atan2(t3, t4))
		
		while self.Z < 0:
			self.Z = 360-self.Z

		return self.Z
		
if __name__=='__main__':
	try:
		x = pid()
		x.start()
	except rospy.ROSInterruptException:
		pass
