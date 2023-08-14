#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

if __name__ == '__main__':
	rospy.init_node('hello_python_node', anonymous=True)     # 初始化 hello_python_node
	pub = rospy.Publisher('publisher_topic', String, queue_size=10)
	rate = rospy.Rate(10)
	
	while not rospy.is_shutdown():
		rospy.loginfo("blink") 
		rospy.sleep(0.01)
