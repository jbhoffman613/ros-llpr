#!/usr/bin/env python
import rospy
import sys
import IPython

from turtlebot_go1.srv import *
from std_msgs.msg import Int16

def go_client(number):
    rospy.wait_for_service('/turtlebot01/go_service')
    my_object = rospy.ServiceProxy('/turtlebot01/go_service', go_service_srv)
    
    result = my_object(number)
    return result

if __name__ == "__main__":
    number = int(sys.argv[1])
    p = Int16()
    p.data = number - 1
    re=go_client(p)
