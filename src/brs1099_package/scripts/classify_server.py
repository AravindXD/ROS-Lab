#!/usr/bin/env python3

import rospy
from brs1099_package.srv import ClassifyObject, ClassifyObjectResponse

def classify_object(req):
    # Calculate volume
    volume = req.length * req.width * req.height
    
    # Classify based on volume
    if volume < 1000:
        classification = "Small"
    elif volume < 5000:
        classification = "Medium"
    else:
        classification = "Large"
        
    rospy.loginfo(f"Object volume: {volume}, Classification: {classification}")
    return ClassifyObjectResponse(classification)

def classify_server():
    rospy.init_node('classify_server')
    s = rospy.Service('classify_object', ClassifyObject, classify_object)
    rospy.loginfo("Ready to classify objects based on volume.")
    rospy.spin()

if __name__ == "__main__":
    try:
        classify_server()
    except rospy.ROSInterruptException:
        pass
