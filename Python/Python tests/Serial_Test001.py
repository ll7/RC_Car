# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""

import serial
import time

portName = "/dev/cu.usbmodemFD131"

arduino = serial.Serial(portName, 115200, timeout=.1)

while True:
    data = arduino.readline()[:-2]
    
    print data
    time.sleep(1)