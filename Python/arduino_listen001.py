#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
from std_msgs.msg import Float32

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %0.3f", data.data)
    
def arduino_listen001():
    
    rospy.init_node('arduino_listen001', anonymous=True)
    
    rospy.Subscriber("temperature", Float32, callback)
    
    rospy.spin()
    
if __name__ == '__main__':
    arduino_listen001()