import rospy
from std_msgs.msg import Float32
from calypso_msgs.msg import buoy

class plot:
    def __init__(self):
        self.x=0;
        self.y=0
        self.z=0

    def imu_callback(msg,self):
        b=buoy()
        self.x=b.x
        self.y=b.y
        self.z=b.z

# Initialize the ROS node
rospy.init_node('imu_plot')

rospy.Subscriber('/rosetta/imu/data', buoy, imu_callback)

# Loop until the node is shut down
while not rospy.is_shutdown():
    rospy.loginfo(sensor_data)
    rospy.sleep(0.1) # Sleep for 100 milliseconds

