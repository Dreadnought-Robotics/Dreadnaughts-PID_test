#! /usr/bin/python3
import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Int64
from calypso_msgs.msg import buoy
from calypso_msgs.msg import dolphins
from calypso_msgs.msg import gypseas
import math

class PID:
	def __init__(self):
        rospy.init_node('calypso_pid', anonymous=False)

        self.KP = 0.5
        self.KI = 0.01
        self.KD = 0.1

        self.throttle = 1574

yaw_error = 0
yaw_integral = 0
yaw_derivative = 0
previous_yaw_error = 0

thruster_front_left = 0
thruster_front_right = 0
thruster_back_left = 0
thruster_back_right = 0

def yaw_callback(data):
    global current_yaw
    current_yaw = data.data

def pid_loop():
    thruster_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.init_node('yaw_pid', anonymous=True)
    rate = rospy.Rate(10)

    rospy.Subscriber('/yaw_angle', Float32, yaw_callback)

    while not rospy.is_shutdown():
        yaw_error = desired_yaw - current_yaw

        yaw_integral += yaw_error

        if yaw_integral > 10:
            yaw_integral = 10
        elif yaw_integral < -10:
            yaw_integral = -10

        yaw_derivative = yaw_error - previous_yaw_error

        pid_output = KP * yaw_error + KI * yaw_integral + KD * yaw_derivative

        thruster_front_left = THRUSTER_FORWARD * pid_output * THRUSTER_LEFT
        thruster_front_right = THRUSTER_FORWARD * pid_output * THRUSTER_RIGHT
        thruster_back_left = THRUSTER_BACKWARD * pid_output * THRUSTER_LEFT
        thruster_back_right = THRUSTER_BACKWARD * pid_output * THRUSTER_RIGHT

        twist_msg = Twist()
        twist_msg.linear.x = thruster_front_left + thruster_front_right + thruster_back_left + thruster_back_right
        twist_msg.linear.y = 0
        twist_msg.linear.z = 0
        twist_msg.angular.x = 0
        twist_msg.angular.y = 0
        twist_msg.angular.z = 0

        thruster_pub.publish(twist_msg)

        previous_yaw_error = yaw_error

        rate.sleep()

if __name__ == '__main__':
    try:
        pid_loop()
    except rospy.ROSInterruptException:
        pass
