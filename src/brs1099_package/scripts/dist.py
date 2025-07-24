#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from datetime import datetime
import random

# === PIN CONFIGURATION ===

def read_distance_real():
    """
    Use HC-SR04 ultrasonic sensor to get real distance in cm.
    """
    try:
        import RPi.GPIO as GPIO
        import time
        TRIG = 23  # GPIO Pin number for TRIG
        ECHO = 24  # GPIO Pin number for ECHO
    except ImportError:
        rospy.logwarn("RPi.GPIO not available. Defaulting to simulation.")
        return read_distance_simulated()

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(TRIG, GPIO.OUT)
    GPIO.setup(ECHO, GPIO.IN)

    # Send pulse
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    # Wait for echo
    pulse_start, pulse_end = time.time(), time.time()

    timeout = time.time() + 0.04  # 40ms timeout
    while GPIO.input(ECHO) == 0:
        pulse_start = time.time()
        if pulse_start > timeout:
            return -1  # Timeout

    while GPIO.input(ECHO) == 1:
        pulse_end = time.time()
        if pulse_end - pulse_start > 0.04:
            return -1  # Timeout

    pulse_duration = pulse_end - pulse_start
    distance_cm = round(pulse_duration * 17150, 2)  # Speed of sound calc

    GPIO.cleanup()
    return distance_cm

def read_distance_simulated():
    """
    Simulate distance for testing purposes.
    """
    return round(random.uniform(2.0, 400.0), 2)

def ultrasonic_publisher(use_simulated=False):
    topic_name = 'ultrasonic_distance'
    pub = rospy.Publisher(topic_name, Float32, queue_size=10)
    rospy.init_node('ultrasonic_sensor_node', anonymous=True)
    rate = rospy.Rate(2)  # 2 Hz

    while not rospy.is_shutdown():
        distance = read_distance_simulated() if use_simulated else read_distance_real()
        timestamp = datetime.now().strftime("%I:%M:%S %p")

        # if distance == -1:
        #     rospy.logwarn(f"[{timestamp}] Measurement timeout or error.")
        # else:
        rospy.loginfo(f"Publishing at [/{topic_name} {timestamp}]")
        pub.publish(distance)

        rate.sleep()

if __name__ == '__main__':
    try:
        # Set to False if using real sensor on Raspberry Pi
        ultrasonic_publisher(use_simulated=True)
    except rospy.ROSInterruptException:
        pass

