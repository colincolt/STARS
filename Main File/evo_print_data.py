import serial
import serial.tools.list_ports
import time
import sys
import os
import binascii
import smtplib
import statistics

#Dist = distanceMeasure()

#def distanceMeasure()
def findEvo():
    # Find Live Ports, return port name if found, NULL if not
    print 'Scanning all live ports on this PC'
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        #print p # This causes each port's information to be printed out.
        if "5740" in p[2]:
            print 'Evo found on port ' + p[0]
            return p[0]
    return 'NULL'


def openEvo(portname):
    print 'Attempting to open port'
    # Open the Evo and catch any exceptions thrown by the OS
    print portname
    evo = serial.Serial(portname, baudrate=115200, timeout=2)

    set_text = (0x00, 0x11, 0x01, 0x45)
    evo.flushInput()
    evo.write(set_text)
    evo.flushOutput()
    print 'Text mode'
    print 'Serial port opened'
    return evo


if __name__ == "__main__":

    print 'Starting Evo Streaming data'

    # Get the port the evo has been connected to
    port = findEvo()

    if port == 'NULL':
        print "Sorry couldn't find the Evo. Exiting."
        sys.exit()
    else:
        evo = openEvo(port)
        
    distArray = []
    
    while len(distArray) < 10:
        distance = evo.readline()
        while distance == "-Inf" or distance == "+Inf":
            distance = evo.readline()
        try:
            distance = float(distance)
            distArray.append(distance)
            print distance
        except ValueError,e:
            print "error",e
    print "Average Distance:" + str((sum(distArray)/len(distArray)))
        #if statistics.stdev(distArray) < 20:
        #    averageDist = Average(distArray)
        #else:
        #    pass
       
    evo.close()
    file.close()

    sys.exit()






