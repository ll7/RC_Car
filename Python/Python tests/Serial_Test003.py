#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sat Dec 23 16:58:33 2017

@author: lennartluttkus
"""

import serial, time
arduino = serial.Serial("/dev/cu.usbmodemFD131", 115200, timeout=.1)
time.sleep(1) #give the connection a second to settle
arduino.write("1+1=")
print arduino.read()