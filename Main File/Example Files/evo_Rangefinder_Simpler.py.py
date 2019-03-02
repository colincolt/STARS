import serial
import serial.tools.list_ports
import time
import sys
import os
import binascii
import smtplib
import statistics

distArray = []
evo = serial.Serial(portname, baudrate=115200, timeout=2)   ##MANUALY INPUT the 'portname' /tty/.../
set_text = (0x00, 0x11, 0x01, 0x45)
evo.flushInput()
evo.write(set_text)
evo.flushOutput()
print 'Text mode'
print 'Serial port opened'

## Infinite Loop
while True:
    distance = evo.readline()
    while distance == "-Inf" or distance == "+Inf": #This catches any out of range measurements
        distance = evo.readline()
    try:                                            #This catches anything else that isnt a number
        distance = int(distance)
        print('Distance(mm):   ' + str(distance))
    except ValueError,e:
        print "error",e

evo.close()
file.close()

sys.exit()
