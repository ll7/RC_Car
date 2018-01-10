#!/usr/bin/env python
# license removed for brevity
import rospy
import joystick_P2500 as js


from std_msgs.msg import Int16


max_throttle = 30.0



pub_js_steer = rospy.Publisher('steering_servo', Int16, queue_size=10)
pub_js_throttle = rospy.Publisher('esc_throttle', Int16, queue_size=10)
pub_js_record = rospy.Publisher('record', Int16, queue_size = 10)
pub_js_autonom = rospy.Publisher('autonom', Int16, queue_size = 10)

rospy.init_node('ros_joystick_publisher4', anonymous=True)

rate = rospy.Rate(15) # 10hz

joy = js.Joystick(refreshRate=60)

while not rospy.is_shutdown():
    # adjusted the value, so it fits to the arduino and rc car
    steering_angle = int((-1.0 * (joy.rightX_P2500() + 1.0) * 90.0 + 180.0)/2.0 + 90.0/2.0)
    
    # somehow impossible to go backwards...
    esc_throttle = int(((joy.rightY_P2500() + 1.0) + 2.0 )*max_throttle)    
    rospy.loginfo(esc_throttle)
    
    record = joy.rightBumper()
    autonom = joy.rightTrigger()
    
    pub_js_steer.publish(steering_angle)
    pub_js_throttle.publish(esc_throttle)
    pub_js_record.publish(record)
    pub_js_autonom.publish(autonom)

    rate.sleep()

joy.close()