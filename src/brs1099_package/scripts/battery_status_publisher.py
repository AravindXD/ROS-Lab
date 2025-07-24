#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32

def get_battery_art(level):
    """
    Returns ASCII art battery with charge level
    """
    blocks = "█" * int(level/10) + "░" * (10 - int(level/10))
    color = "\033[1;92m" if level > 20 else "\033[1;91m"  # Green if OK, Red if low
    return f"""
    {color}╔════════════════╗
    ║ {blocks} ║ {level:3d}%
    ╚════════════════╝\033[0m"""

def publish_battery_status():
    """
    Publishes battery status in descending order from 100% to 0%
    """
    rospy.init_node('battery_status_publisher', anonymous=True)
    pub = rospy.Publisher('battery_status', Int32, queue_size=10)
    
    # Print startup banner
    print("\033[1;96m" + """
    ╔══════════════════════════════════╗
    ║     Battery Status Publisher     ║
    ║        Starting up...            ║
    ╚══════════════════════════════════╝
    """ + "\033[0m")
    
    battery_level = 30 
    discharge_rate = 5   
    
    while not rospy.is_shutdown():
        try:
            pub.publish(battery_level)
            battery_art = get_battery_art(battery_level)
            rospy.loginfo(f"Published battery level:{battery_art}")
            
            # Decrease battery level
            battery_level = max(0, battery_level - discharge_rate)  # Don't go below 0
            
            if battery_level == 0:
                print("DEAD")
                break
                
            rospy.sleep(2)
        except KeyboardInterrupt:pass
        
    print("\033[1;93m" + """
    ╔══════════════════════════════════╗
    ║     Battery Status Publisher     ║
    ║     KeyINT. Shutting Down...     ║
    ╚══════════════════════════════════╝
    """ + "\033[0m")
    
if __name__ == '__main__':
    publish_battery_status()

