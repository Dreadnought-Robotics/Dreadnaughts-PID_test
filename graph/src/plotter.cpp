#include <ros/ros.h>
#include <std_msgs/Float32.h>

void myCallback(const std_msgs::Float32::ConstPtr& msg)
{
  // This function is called every time a new message is received on the "my_topic" topic
  ROS_INFO("Received  X data: [%f]", msg->data);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "my_plotter_node");

  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("my_topic", 1000, myCallback);

  ros::spin();

  return 0;
}

