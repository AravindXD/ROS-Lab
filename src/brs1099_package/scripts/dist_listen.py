#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, String

# Define the distance threshold (in cm)
THRESHOLD_CM = 30.0

def distance_callback(msg):
    """
    Called whenever a new distance value arrives.
    Determines status and publishes message accordingly.
    """
    distance = msg.data
    status = "WARNING" if distance < THRESHOLD_CM else "OK"
    if status == "WARNING":
        colored_status = f"\033[1;91m{status}\033[0m"  # Bright red
    else:
        colored_status = f"\033[1;92m{status}\033[0m"  # Bright green
    rospy.loginfo(f"Distance: {distance:.2f} cm → {colored_status}")
    status_pub.publish(status)

def listener():
    """
    Initializes the subscriber node, and sets up the publisher.
    """
    global status_pub
    rospy.init_node('distance_monitor', anonymous=True)

    # Publisher for status
    status_pub = rospy.Publisher('distance_status', String, queue_size=10)

    # Subscriber to distance values
    rospy.Subscriber('ultrasonic_distance', Float32, distance_callback, queue_size=10)

    rospy.loginfo("distance_monitor node started, threshold=%0.2f cm", THRESHOLD_CM)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
