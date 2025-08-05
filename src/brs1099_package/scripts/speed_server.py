#!/usr/bin/env python3

import rospy
from brs1099_package.srv import SetSpeed, SetSpeedResponse

class SpeedServer:
    def __init__(self):
        rospy.init_node('speed_server')
        self.service = rospy.Service('set_speed', SetSpeed, self.handle_set_speed)
        rospy.loginfo("Speed Server is ready")

    def handle_set_speed(self, req):
        response = SetSpeedResponse()
        
        # Log the received speed values
        rospy.loginfo(f"Received speed request - Linear: {req.linear}, Angular: {req.angular}")
        
        # Process the request (you can add validation here if needed)
        response.success = True
        response.message = f"Speed set successfully to linear: {req.linear}, angular: {req.angular}"
        
        return response

if __name__ == '__main__':
    server = SpeedServer()
    rospy.spin()
