#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32

# Define the battery threshold (in percentage)
BATTERY_THRESHOLD = 20

def get_status_art(level):
    """
    Returns ASCII art status display
    """
    blocks = "█" * int(level/10) + "░" * (10 - int(level/10))
    color = "\033[1;91m" if level <= BATTERY_THRESHOLD else "\033[1;92m"  # Red if low, Green if OK
    status = "LOW BATTERY!" if level <= BATTERY_THRESHOLD else "BATTERY OK"
    
    return f"""
    {color}╔════════════════╗
    ║ {blocks} ║ {level:3d}%   {status}
    ╚════════════════╝\033[0m"""

def battery_callback(msg):
    """
    Called whenever a new battery status arrives
    """
    battery_level = msg.data
    status_art = get_status_art(battery_level)
    if battery_level <= BATTERY_THRESHOLD:
        rospy.logwarn(f"Battery Status Monitor:{status_art}")
    else:
        rospy.loginfo(f"Battery Status Monitor:{status_art}")

def listener():
    """
    Initializes the subscriber node and listens for battery status
    """
    rospy.init_node('battery_monitor', anonymous=True)
    
    # Print startup banner
    print("\033[1;96m" + """
    ╔══════════════════════════════════╗
    ║      Battery Status Monitor      ║
    ║          Starting up...          ║
    ║                                  ║
    ║         Threshold: """ + f"{BATTERY_THRESHOLD:2d}%" + """           ║
    ╚══════════════════════════════════╝
    """ + "\033[0m")
    
    rospy.Subscriber('battery_status', Int32, battery_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        print("\033[1;93m" + """
    ╔══════════════════════════════════╗
    ║      Battery Status Monitor      ║
    ║        Shutting down...         ║
    ╚══════════════════════════════════╝
    """ + "\033[0m")
    except KeyboardInterrupt:
        print("\033[1;93m" + """
    ╔══════════════════════════════════╗
    ║      Battery Status Monitor      ║
    ║      Keyboard Interrupt...      ║
    ╚══════════════════════════════════╝
    """ + "\033[0m")
