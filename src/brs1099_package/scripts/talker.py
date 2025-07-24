#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from datetime import datetime

def talker():
	topic_name='aravinds_topic'
	pub = rospy.Publisher(topic_name, String,queue_size=10)
	rospy.init_node('talker_node', anonymous=True)
	rate = rospy.Rate(1)

	while not rospy.is_shutdown():
		now = datetime.now()
		readable = now.strftime("%A, %d %B %Y, %I:%M %p")
		msg = f"Hello from Aravind 22BRS1099! {readable}"
		rospy.loginfo(f"Publishing at /{topic_name}")
		pub.publish(msg)
		rate.sleep()

if __name__ == '__main__':
	talker()
