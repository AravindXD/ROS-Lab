#!/usr/bin/env python3

import rospy
import random
from geometry_msgs.msg import Point
from brs1099_package.msg import DroneTelemetry

def publisher():
    pub = rospy.Publisher('telemetry', DroneTelemetry, queue_size=10)
    rospy.init_node('drone_publisher', anonymous=True)
    rate = rospy.Rate(1) 

    drone_id = rospy.get_param("~drone_id", "drone1")
    battery_level = 50.0  

    while not rospy.is_shutdown():
        msg = DroneTelemetry()
        msg.drone_id = drone_id

        # Random walk for position
        msg.position = Point(
            x=random.uniform(0.0, 50.0),
            y=random.uniform(0.0, 50.0),
            z=random.uniform(0.0, 10.0)
        )

        battery_level -= 10.0 
        msg.battery_level = battery_level
        msg.is_emergency = battery_level < 20.0

        pub.publish(msg)
        rate.sleep()
        print("Sending...")
        if  not battery_level:
            print("Bye")
            break

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
