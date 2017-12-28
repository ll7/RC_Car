#!/usr/bin/env python
# license removed for brevity
import rospy
import joystick_P2500 as js


from std_msgs.msg import Int16



def ros_joystick_publisher():
    pub_js_steer = rospy.Publisher('steering_servo', Int16, queue_size=10)
    rospy.init_node('ros_joystick_publisher', anonymous=True)
    rate = rospy.Rate(15) # 10hz
    steering_angle = 0
    joy = js.Joystick()
    while not rospy.is_shutdown():
        steering_angle = int((joy.rightX_P2500() + 1.0) * 90.0)
        rospy.loginfo(steering_angle)
        pub_js_steer.publish(steering_angle)
#        if steering_angle == 180 :
#            steering_angle = 0
#        else:
#            steering_angle += 1
        rate.sleep()

if __name__ == '__main__':
    try:
        ros_joystick_publisher()
    except rospy.ROSInterruptException:
        pass