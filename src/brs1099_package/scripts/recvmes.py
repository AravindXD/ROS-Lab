#!/usr/bin/env python3
import rospy
from brs1099_package.msg import MyMsg

def callback(msg):
    rospy.loginfo(f"Received message: id={msg.id}, content='{msg.content}'")

def listener():
    rospy.init_node('listener_node')
    rospy.Subscriber('chatter', MyMsg, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()