#!/usr/bin/env python3

import rospy
from brs1099_package.srv import SetSpeed

def speed_client():
    rospy.init_node('speed_client')
    
    # Wait for the service to become available
    rospy.wait_for_service('set_speed')
    try:
        # Create a service proxy
        set_speed = rospy.ServiceProxy('set_speed', SetSpeed)
        
        while not rospy.is_shutdown():
            try:
                # Get user input
                linear = float(input("Enter linear velocity (m/s): "))
                angular = float(input("Enter angular velocity (rad/s): "))
                
                # Call the service
                response = set_speed(linear, angular)
                
                # Display the result
                rospy.loginfo(f"Service response - Success: {response.success}, Message: {response.message}")
                
            except ValueError:
                rospy.logerr("Please enter valid numbers")
            except KeyboardInterrupt:
                break
            
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

if __name__ == '__main__':
    speed_client()
