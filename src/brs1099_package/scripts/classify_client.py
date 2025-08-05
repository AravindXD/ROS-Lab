#!/usr/bin/env python3

import rospy
from brs1099_package.srv import ClassifyObject
import sys

def classify_object_client(length, width, height):
    rospy.wait_for_service('classify_object')
    try:
        classify_object = rospy.ServiceProxy('classify_object', ClassifyObject)
        resp = classify_object(length, width, height)
        return resp.classification
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return None

def usage():
    return f"{sys.argv[0]} [length width height]"

if __name__ == "__main__":
    if len(sys.argv) == 4:
        length = float(sys.argv[1])
        width = float(sys.argv[2])
        height = float(sys.argv[3])
    else:
        # Default test values
        length, width, height = 10.0, 10.0, 10.0
        rospy.loginfo("Using default test values: length=10, width=10, height=10")
    
    rospy.init_node('classify_client', anonymous=True)
    result = classify_object_client(length, width, height)
    if result:
        rospy.loginfo(f"Object classification: {result}")
