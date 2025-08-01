#!/usr/bin/env python3
import rospy
from brs1099_package.msg import MyMsg

def talker():
    rospy.init_node('talker_node')  # Initialize the ROS node
    pub = rospy.Publisher('chatter', MyMsg, queue_size=10)  # Publisher on topic 'chatter'

    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        msg = MyMsg()
        msg.id = 1
        msg.content = "Hello custom ROS message 22BRS1099"
        rospy.loginfo(f"Publishing: id={msg.id}, content={msg.content}")
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass