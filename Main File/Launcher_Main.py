import time
import serial
import serial.tools.list_ports
import sys
import os
import binascii
import smtplib
import statistics

#Global Variables:




# Startup STEREOSCOPIC Sub-process _________________________________________

from subprocess import Popen, PIPE

process = Popen(['cat', 'Stereo_Yaw_program.py'], stdout=PIPE, stderr=PIPE) #'cat' ia UNIX program
stdout, stderr = process.communicate()
print stdout

# STARTUP PROCEDURE__________________________________________________________

#Open Serial Ports: "dmesg -s 1024" to find port name

Uno=serial.Serial("/dev/ttyACM0",115200)        #Arduino Uno
Mega=serial.Serial("/dev/ttyUSB1",115200)        #Arduino Mega
Range=serial.Serial("/dev/ttyUSB2",115200)        #Rangefinder  

Uno.baudrate=115200
Mega.baudrate=115200
Range.baudrate=115200

# Sensors check:

float Temp=777.0
while Temp == 777.0:
    Uno.write('')


#USER Input:


    
# Static Drill Func________________________________________________________________________





# Dynamic Drill Func________________________________________________________________________






# Manual Drill Func________________________________________________________________________
