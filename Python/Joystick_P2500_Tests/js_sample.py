# -*- coding: utf-8 -*-
"""
Created on Thu Dec 28 13:51:24 2017

@author: ll7
"""
import joystick_P2500 as js

def fmtFloat(n):
    return '{:6.3f}'.format(n)
    
joy = js.Joystick()

print "Xbox controller sample: Press Back button to exit"
# Loop until back button is pressed
while not joy.Back():
    # Show connection status
    if joy.connected():
        print "Connected   ",
    else:
        print "Disconnected",
    print "Rx raw: ", fmtFloat(joy.rightX_P2500()),
    print "Ry raw: ", fmtFloat(joy.rightY_P2500()),
    
        
    # Move cursor back to start of line
    print chr(13),
# Close out when done
joy.close()