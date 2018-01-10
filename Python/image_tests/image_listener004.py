#!/usr/bin/env python
"""
http://wiki.ros.org/rospy_tutorials/Tutorials/WritingImagePublisherSubscriber

changes by ll7:
flags change according to opencv3
https://stackoverflow.com/questions/19013961/cv2-imread-flags-not-found

removed feature detection, because its not included in opencv3, only in 
opencv3_contrib
https://www.pyimagesearch.com/2015/07/16/where-did-sift-and-surf-go-in-opencv-3/

Adaptions from
https://github.com/hamuchiwa/AutoRCCar/blob/master/computer/collect_training_data.py

Meine Idee:
public array fuer das transformierte Bild und die Features
Start stop ueber eine topic festlegen
Nach stop exportieren

"""
"""
OpenCV feature detectors with ros CompressedImage Topics in python.

This example subscribes to a ros topic containing sensor_msgs 
CompressedImage. It converts the CompressedImage into a numpy.ndarray, 
then detects and marks features in that image. It finally displays 
and publishes the new image - again as CompressedImage topic.
"""
__author__ =  'Simon Haller <simon.haller at uibk.ac.at>'
__version__=  '0.1'
__license__ = 'BSD'
# Python libs
import sys, time

# numpy and scipy
import numpy as np
from scipy.ndimage import filters

# OpenCV
import cv2

# Ros libraries
import roslib
import rospy

import os

# Ros Messages
from sensor_msgs.msg import CompressedImage
#from std_msgs.msg import String
from std_msgs.msg import Int16
from std_msgs.msg import Bool
# We do not use cv_bridge it does not support CompressedImage in python
# from cv_bridge import CvBridge, CvBridgeError

VERBOSE=True

# image rescaled, width = 280, new_hight = 100; 50:150
image_array = np.zeros((1, 28000))
# two labels: steering and throttle
label_array = np.zeros((1, 2), 'float')
if VERBOSE:
    print 'The image array was created with the size: ', image_array.size
    print 'The label array was created with the size: ', label_array.size
    
# steering data
steering_angle = 90

esc_throttle = 90

# frame counter
frame = 0

class image_feature:

    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''
        # topic where we publish
        self.image_pub = rospy.Publisher("/output/image_raw/compressed",
            CompressedImage, queue_size = 1)
        #self.image_info_pub = rospy.Publisher("output/image_raw/info", String)

        # subscriber to /raspicam_node/image/compressed
        self.subscriber = rospy.Subscriber("/raspicam_node/image/compressed",
            CompressedImage, self.callback,  queue_size = 1)
        if VERBOSE :
            print "/raspicam_node/image/compressed"    
            
            
        self.steering_sub = rospy.Subscriber("/steering_servo", Int16, self.steering_cb)
        self.throttle_sub = rospy.Subscriber("/esc_throttle", Int16, self.throttle_cb)
        
        self.save_sub = rospy.Subscriber("/save_traindata", Bool, self.save_cb)


    def callback(self, ros_data):
        # global variables:
        global image_array        
        global label_array
        global steering_angle
        global esc_throttle
        global frame
        
        
        '''Callback function of subscribed topic. 
        Here images get converted and features detected'''
        if VERBOSE :
            print 'received image of type: "%s"' % ros_data.format

        #### direct conversion to CV2 ####
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_GRAYSCALE)
        
        # recieved a new frame
        frame += 1        
        
        # select middle of camera image
        resized_image = image_np[50:150, :]
        
        # 1-D vector of all the interesting pixels
        temp_array = resized_image.reshape(1, 28000).astype(np.float32)        
        
        # adds the temp_array to the image_array
        image_array = np.vstack((image_array, temp_array))
        
        # add row to label_array
        label_array = np.vstack((label_array, np.array((steering_angle, esc_throttle))))
        
        cv2.imshow('cv_img', resized_image)
        cv2.waitKey(2)
        
        if VERBOSE:
            print 'Steering angle is: ', steering_angle
            print 'Throttle is: ', esc_throttle

        #### Create CompressedIamge ####
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', image_np)[1]).tostring()
        # Publish new image
        self.image_pub.publish(msg)
        
        #self.subscriber.unregister()
        
    def steering_cb(self, data):
        # global variables
        global steering_angle
        steering_angle = data.data
        
    def throttle_cb(self, data):
        global esc_throttle
        esc_throttle = data.data
        
    def save_cb(self, data):
        global image_array
        global label_array
        
        train = image_array[1:,:]
        train_labels = label_array[1:, :]
        if data.data:
            file_name = str(int(time.time()))
            directory = "training_data"
            if not os.path.exists(directory):
                os.makedirs(directory)
            try:    
                np.savez(directory + '/' + file_name + '.npz', train=train, train_labels=train_labels)
            except IOError as e:
                print(e)
        
        
        
def main(args):
    '''Initializes and cleanup ros node'''
    ic = image_feature()
    rospy.init_node('image_listener001', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Image feature detector module"
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)