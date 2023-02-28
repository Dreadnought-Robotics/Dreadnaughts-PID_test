#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32

def imu_callback(msg):
    global imu_data
    imu_data = msg.data

# Initialize the ROS node
rospy.init_node('imu_plot')

rospy.Subscriber('/imu_topic', Float32, sensor_callback)

# Loop until the node is shut down
while not rospy.is_shutdown():
    rospy.loginfo(sensor_data)
    rospy.sleep(0.1) # Sleep for 100 milliseconds

