#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sat Dec 23 16:24:52 2017

@author: lennartluttkus
"""

import serial, time

portName = "/dev/cu.usbmodemFD131"

arduino = serial.Serial(portName, 9600, timeout=.1)

time.sleep(1)

while True:
    arduino.write(90)
    print "I wrote 90"
    print "Arduino read: " + arduino.readline()[:-2]
    time.sleep(1)
    arduino.write(110)
    print "I wrote 110"
    print "Arduino read: " + arduino.readline()[:-2]
    time.sleep(1)
