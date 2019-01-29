# _________________________________READ ME______________________________________#
# THREADING:
# This file uses the python "threading" module to run any number of "Threads"
# simultaneously, anything that needs to run continuously can be put into an
# infinite loop, anything that needs to run independantly and not interfere
# with other tasks. THREADS are ideal for input output operations, we do not
# have a lot of math being done.
#
# DEQUEUE:
# Notice the use of Queues to input output data to and from threads, read the
# documentation to get a better understanding (https://docs.python.org/3/library/asyncio-queue.html)
#
# BASIC PROGRAM STRUCTURE:
# 

# PROGRAM OPTIONS:

# Stereo packages
from collections import deque           # < Lib for Stacks
from imutils.video import VideoStream
import numpy as np
import argparse
import cv2
import imutils
import time
import socket
import serial
from threading import *
import serial.tools.list_ports
import sys
import os
import binascii
import smtplib
import statistics
from picamera.array import PiRGBArray
from picamera import PiCamera
# from queue import LifoQueue
# import struct
# import math
# import signal


class DistanceData:
    def __init__(self, p1, p2, p3, p4):
        self.p1 = p1
        self.p2 = p2
        self.p3 = p3
        self.p4 = p4


class GuiInput:
    def __init__(self, p1, p2, p3):
        self.p1 = p1
        self.p2 = p2
        self.p3 = p3


class StereoOutput:
    def __init__(self, p1, p2, p3, p4):
        self.p1 = p1
        self.p2 = p2
        self.p3 = p3
        self.p4 = p4
        
        
class FinalDist:
     def __init__(self, p1, p2, p3):
        self.p1 = p1
        self.p2 = p2
        self.p3 = p3
        
        
class Stack:
    def __init__(self):
        self.items = []

    def isEmpty(self):
        return self.items == []

    def push(self, item):
        self.items.append(item)

    def pop(self):
        return self.items.pop()

    def peek(self):
        if not (self.isEmpty()):
            return self.items[len(self.items) - 1]
        return

    def size(self):
        return len(self.items)


# Define Global Constants and Stacks --> Comm pipes
stereoStack = Stack()
distanceStack = Stack()
guiStack = Stack()
voiceCommandStack = Stack()
finalDistStack = Stack()
startCommandStack = Stack()

distArray = []
# #____________________________________THREADS__________________________________###

# PITCH_YAW_THREAD: communicates to the Arduino Uno in order to provide Angle
# values to both motors, request temperature's from the Uno, and distance measurements
# from the Arduino Mega or Serial.
# These distance measurements are also provided to the LAUNCHER THREAD

# _______PITCH AND YAW CONTROL________ #
class PitchYaw(Thread):
    def __init__(self, getstereoStack, getguiStack):  # getdistanceStack
        Thread.__init__(self)
        self.getstereoStack = getstereoStack
        #self.getdistanceStack = getdistanceStack  # # << OPTIMIZATION - grab FINAL_DIST
        self.getguiStack = getguiStack

    def run(self):
        # _ARDUINO_UNO__SERIAL_OPEN
        ser = serial.Serial("/dev/ttyACM0", 9600)  # change ACM number as found from "ls /dev/tty*"
        ser.baudrate = 9600
                        
        guiData = self.getguiStack.peek()
        speed = guiData.speed
        difficulty = guiData.difficulty
        drillType = guiData.drilltype
        # print("Speed:  " + str(speed) + "  Diff:  " + str(difficulty) + "  Type:  " + str(drillType))
        
        while True:
            try:
                #print ("starting PitchYaw Thread")
                # distance = self.getdistanceStack.peek()   # << OPTIMIZATION - grab FINAL_DIST
                result = self.getstereoStack.peek()
                if not(result):
                    continue
                LeftXcoord = result.masterval
                RightXcoord = result.slaveval
                distance = int(result.distance)
                
                #print(distance)
                
                if distance <= 10:
                    pitchAngle = 25  # 10 DEGREES
                if 10 < distance <= 45:
                    pitchAngle = 45  # 20 DEGREES
##                if 10 < distance <= 15:
##                    pitchAngle = 30  # # 30 DEGREES
##                if 15 < distance <= 20:
##                    pitchAngle = 40  # # 30 DEGREES
##                if 20 < distance <= 50:
##                    pitchAngle = 45  # # 30 DEGREES
                else:
                    pitchAngle = 45  # 20 DEGREES

                # inequality = 2464 - LeftXcoord
                # print("LeftXcoord = " + str(LeftXcoord))
                # print("RightXcoord = " + str(RightXcoord))
                # print("inequality = 2464 - LeftXcoord =" + str(inequality))
                ser.reset_input_buffer()
                # ser.set_output_flow_control(True)

                if (abs(2464 - LeftXcoord - RightXcoord) < 150):
                    #value = '0'
                    data = '<0, '+ str(pitchAngle) + '>'
                    ser.write(data.encode())
                    #print(0)
                    # sys.stdout.flush()
                    #print("locked on to target")
                    time.sleep(0.1)

                elif (2464 - LeftXcoord < RightXcoord):
                    data = '<-80, '+ str(pitchAngle) + '>'
                    ser.write(data.encode())                    
                    #print(-80)
                    # sys.stdout.flush()
                    #print("moving CW")
                    time.sleep(0.1)

                elif (2464 - LeftXcoord > RightXcoord):
                    data = '<80, '+ str(pitchAngle) + '>'
                    ser.write(data.encode())                    
                    # print(80)
                    # sys.stdout.flush()
                    #print("moving CCW")
                    time.sleep(0.1)
                else:
                    continue
                    
            except Exception as e:
                print('Pitch yaw thread failed because of exception ' + e)
                continue


# __________ Sterepscopic Thread___________ #
def StereoscopicsThread(stereoStack):
    #print('Starting Stereoscopics THREAD')
    focalsize = 3.04e-03
    pixelsize = 1.12e-06
    baseline = 0.737
    datapoints = 5
    centroid = (0, 0)
    compvalue = 1
    
    ##________________Sockets TCP/IP Communication__________________________________###########

    TCP_IP = '169.254.116.12'
    TCP_PORT = 5025
    BUFFER_SIZE = 1024
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    connected = False
    while not connected:  # Wait for client
        try:
            s.bind((TCP_IP, TCP_PORT))
            s.listen(1)
            # sock.settimeout(5)
            clientPort, addr = s.accept()
            connected = True
        except socket.error:
            print('Stereoscopics:       No Client')
            time.sleep(3)
            continue

    connected = True

    print("Client connected")

    ap = argparse.ArgumentParser()
    ap.add_argument("-v", "--video",
                    help="path to the (optional) video file")
    ap.add_argument("-b", "--buffer", type=int, default=8, help="max buffer size")
    args = vars(ap.parse_args())

    # define the lower and upper boundaries of the jersey ball in the HSV color space, then initialize the list of tracked points
    jerseyLower1 = (0, 50, 50)  # currently set for red
    jerseyUpper1 = (5, 255, 255)
    jerseyLower2 = (175, 50, 50)  # currently set for red
    jerseyUpper2 = (180, 255, 255)
    pts = deque(maxlen=args["buffer"])

    frameRate = 10
    framesize=(600, 450)
    # initialize array of distance values
    distvals = []

    vs = VideoStream(src=0, resolution = framesize, framerate = frameRate).start()

    time.sleep(0.1)
    print('starting stereo loop')

    ##________________BEGINNING OF LOOP________________##
    while True:
        # grab the current frame
        image = vs.read()
        
        blurred = cv2.GaussianBlur(image, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        mask0 = cv2.inRange(hsv, jerseyLower1, jerseyUpper1)
        mask1 = cv2.inRange(hsv, jerseyLower2, jerseyUpper2)
        mask = mask0 + mask1
        # mask = cv2.inRange(hsv, greenLower, greenUpper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        # find contours in the mask and initialize the current (x, y) center of the ball
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        center = None
        # centroid = None

        # only proceed if at least one contour was found
        if len(cnts) > 0:
            # find the largest contour in the mask, then use it to compute the minimum enclosing circle and centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            centroid = (round((M["m10"] / M["m00"]), 3), round((M["m01"] / M["m00"]), 3))
            centroid = (centroid[0] * 2464 / 600, centroid[1] * 2464 / 600)

            # only proceed if the radius meets a minimum size
            if radius > 0.1:
                # draw the circle and centroid on the frame, then update the list of tracked points
                cv2.circle(image, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                cv2.circle(image, center, 5, (0, 0, 255), -1)

        # update the points queue
        pts.appendleft(center)

        # loop over the set of tracked points
        for i in range(1, len(pts)):
            # if either of the tracked points are None, ignore
            # them
            if pts[i - 1] is None or pts[i] is None:
                continue

            # otherwise, compute the thickness of the line and draw the connecting lines
            thickness = int(np.sqrt(args["buffer"] / float(i + 1)) * 2.5)
            cv2.line(image, pts[i - 1], pts[i], (0, 0, 255), thickness)

        try:
            print('waiting for data')
            data = clientPort.recv(BUFFER_SIZE)
            print ("received data:", data)
            clientPort.send(data)           # SEND DATA BACK (COULD BE USED FOR STOP COMMAND)
            compvalue = data.decode()
        except socket.error:
            connected = False
            while not connected:
                try:
                    data = clientPort.recv(BUFFER_SIZE)
                    print ("received data:", data)
                    clientPort.send(data)           # SEND DATA BACK (COULD BE USED FOR STOP COMMAND)
                    compvalue = data.decode()
                except socket.error:
                    time.sleep(2)

        if len(cnts) > 0:
            # print("Decoded value: " + compvalue)
            slaveval = float(compvalue)
            masterval = centroid[0]
            # masterval = x*2464/600
            # disparity = abs(float(compvalue)-centroid[0])
            disparity = abs(masterval - slaveval)
            distance = (focalsize * baseline) / (disparity * pixelsize)
            print(distance)
            # SENDS DATA TO Class, which can be "Put" using Queue's______________________________
            results = StereoOutput
            results.distance = distance
            results.disparity = disparity
            results.masterval = masterval
            results.slaveval = slaveval
            stereoStack.push(results)



# Connects and communicates with the Arduino Mega: Launcher Motors, Ball Feeder, Wifi, Accelerometer? (Rangefinder?)
class Launcher(Thread):
    def __init__(self, sendVoiceCommandStack, sendDistanceStack, guiStack, getStereoStack, getStartCommand):
        Thread.__init__(self)
        self.sendVoiceCommandStack = sendVoiceCommandStack
        self.sendDistanceStack = sendDistanceStack
        self.guiStack = guiStack
        self.getStereoStack = getStereoStack
        self.getStartCommand = getStartCommand
        self.sendDistanceStack = sendDistanceStack


    def run(self):
        try:
            oldLidarDist = 0.0
            # __GET GUI DATA __ #
            guiData = self.guiStack.peek()
            speed = guiData.speed
            difficulty = guiData.difficulty
            drillType = guiData.drilltype
            print("LAUNCHER: Speed:  " + str(speed) + "  Diff:  " + str(difficulty) + "  Type:  " + str(drillType))
            
            # speedScale = speed / 2
            
            # open serial port
            MEGA = serial.Serial('/dev/ttyACM1', 9600)
            MEGA.timeout = None
            baudrate=9600,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            print("Connected to MEGA")
            
            oldtemp = 0
        except Exception as e:
            print("Error in Launcher setup"+e)
        while True:
            try:
                print("In the Launcher LOOP")
                # __ GET DATA\
                getdata = False
                dataPresent = False

                while getdata == False:
                    print('in the while')
                    if MEGA.is_open:
                        print('in the Mega.is_open')
                        while not dataPresent:
                            print('data no present')
                            try:
                                print('in the try')
                                size = MEGA.read().decode()
                            except Exception as e:
                                print('LAUNCHER EXCEPTION'+ e)
                            if size == "<":
                                megaDataStr = MEGA.read(25)  # READ DATA FORMATTED AS ('< 1 2 3 4 5 >')
                                megaDataTemp = list(megaDataStr.decode())
                                megaDataTemp.insert(0, size)
                                megaData = megaDataTemp[:megaDataTemp.index(">")+1]
                                print("MegaData", megaData)
                                temp = "".join(megaData)
                                print("Test", temp)
                                dataPresent = True
                                print('data read from Mega')
                    else:
                        continue
                        
                    # lidarDistance = int(cm)
                    # temperature = int()
                    # voice commands = int(from 1 to 5)
                    # targetTiming = float(0.0)
                    # targetBallSpeed
                    
                    # ____RECIEVE MEGA DATA STRING _____#
                    #print(megaDataStr)
                    time.sleep(0.4)
                    #megaDataList =  megaDataStr.decode().split(',')  # < split() divides the string at the spaces                
                    

                        
                    
##                    if (megaDataList[0] == '<' and megaDataList[6] == '>'):  # check beginning and end markers
##                        lidarDist = megaDataList[1]
##                        if lidarDist != oldLidarDist:
##                            print('LIDAR dist sent to MAIN')
##
##                        newtemp = megaDataList[2]
##                        
##                        if newtemp == ' ':
##                            temperature = oldtemp
##                        else:
##                            temperature = int(newtemp)
##                        
##                        voiceCommand = megaDataList[3]
##
##                        if voiceCommand == 2:  # << 2 = STOP commmand
##                            break
##                        elif voiceCommand == ' ':
##                            userCommand = 0
##                        else:
##                            userCommand = int(voiceCommand)
##                            
##                        targetTiming = megaDataList[4]
##                        
##                        if targetTiming == ' ':
##                            targetTime = -1
##                        else:
##                            targetTime = float(targetTiming)
##                        
##                        targetBallSpeed = megaDataList[5]
##                        
##                        if targetBallSpeed == ' ':
##                            targetSpeed = -1
##                        else:
##                            targetSpeed = float(targetBallSpeed)
##                            
####                        print(temperature)
##                        
##                    else:
##                        continue
                    
                    getdata = True
                    
                lidarDist = int(temp.strip("<").strip().split(",")[0])
                LidarDist = lidarDist/100 # << convert to meters
                self.sendDistanceStack.push(LidarDist)

                # __Get Final Distance from main
                time.sleep(1)
                result = self.getStereoStack.peek()
                FINAL_DIST = int(result.distance)

##                FINAL_DIST = self.getFinalDistStack.peek()
##                while FINAL_DIST == None:
##                    FINAL_DIST = self.getFinalDistStack.peek()

                print('Launcher Thread: Final Distance:  '+ str(FINAL_DIST))
                # ___QUERY LOOKUP TABLE ____ #
                if FINAL_DIST < 5:
                    motorSpeed1= motorSpeed2 = 50
                elif 5 <= FINAL_DIST < 10:
                    motorSpeed1= motorSpeed2 = 80
                elif 10 <= FINAL_DIST < 20:                
                    motorSpeed1= motorSpeed2 = 100
                elif 20 <= FINAL_DIST < 50:                
                    motorSpeed1= motorSpeed2 = 125
                else:
                    motorSpeed1= motorSpeed2 = 0
                    print('out of range')
                
                # ____SEND MEGA DATA STRING _____#    IF NO VALUE SEND AS '-1'
                #
                MotorSpeed1 = str(motorSpeed1)
                MotorSpeed2 = str(motorSpeed2)
                targetChoice = '2'
                data = '<' + MotorSpeed1 + ',' + MotorSpeed2 + ',' + targetChoice + ',' + str(difficulty) + '>'
                print(data)
                MEGA.write(data.encode())
                
                
                # sendVoiceCommandStack.push(data)
                
            except Exception as e:
                print('Launcher thread failed because of exception ' + e)
                continue
##
###    Evo Rangefinder Port:
##
##
##
##def findEvo():
##    # Find Live Ports, return port name if found, NULL if not
##    print ('Scanning all live ports on this PC')
##    ports = list(serial.tools.list_ports.comports())
##    for p in ports:
##        #print p # This causes each port's information to be printed out.
##        if "5740" in p[2]:
##            print ('Evo found on port ' + p[0])
##            return p[0]
##    return 'NULL'
##
##
##def openEvo(portname):
##    print('Attempting to open port')
##    # Open the Evo and catch any exceptions thrown by the OS
##    print (portname)
##    evo = serial.Serial(portname, baudrate=115200, timeout=2)
##
##    set_text = (0x00, 0x11, 0x01, 0x45)
##    evo.flushInput()
##    evo.write(set_text)
##    evo.flushOutput()
##    print ('Text mode')
##    print ('Serial port opened')
##    return evo
##
##
##def EvoRanger():
##    print ('Starting Evo Streaming data')
##
##    # Get the port the evo has been connected to
##    port = findEvo()
##
##    if port == 'NULL':
##        print ("Sorry couldn't find the Evo. Exiting.")
##        sys.exit()
##    else:
##        evo = openEvo(port)
##    
##    distance = evo.readline()
##    
##    distance = evo.readline()
##    while distance == "-Inf" or distance == "+Inf":
##        distance = evo.readline()
##        try:
##            distance = float(distance)
##            print (distance)
##        except Exception as e:
##            print ("error" +e)
##            continue
##        
##            return distance
##    # evo.close()
##    # file.close()
##    # sys.exit()

# MAIN FILE START
def startMainFile(speed, difficulty, drilltype):  # , args): ## NOT A THREAD, performs the bulk of calculation
    # NOTE: cannot gets args from command line so must be passed in via the gui
    # # construct the argument parse and parse the arguments
    # ap = argparse.ArgumentParser()
    # ap.add_argument("-v", "--video",
    #     help="path to the (optional) video file")
    # ap.add_argument("-b", "--buffer", type=int, default=8,
    #     help="max buffer size")
    # args = vars(ap.parse_args())

    # # _______________________________Main Processing_____________________________________

    # __ VARIABLES _____#
    total = 2
    startCommand = 0

    # __ GUI Input __ #

    # print(difficulty)
    guiData = GuiInput
    guiData.speed = speed
    guiData.difficulty = difficulty
    guiData.drilltype = drilltype
    print("Speed:  " + str(guiData.speed) + "  " + "Diff:  " + str(guiData.difficulty) + "  " + "Drill:  " + guiData.drilltype)

    guiStack.push(guiData)

    # Start Threads

    # ___Begin Stereoscopic Thread:____ #
    try:
        process = Thread(target=StereoscopicsThread, args=[stereoStack])
        process.start()
    except Exception as e:
        print('Stereo thread failed because of exception ' + e)

    time.sleep(6)
    try:
        pitchYawthread = PitchYaw(stereoStack, guiStack)   # distanceStack
        pitchYawthread.start()
    except Exception as e:
        print('Pitch and Yaw thread didnt start because of exception ' + e)
    try:
        startLauncherThread = Launcher(voiceCommandStack, distanceStack, guiStack, stereoStack, startCommandStack)
        startLauncherThread.start()
    except Exception as e:
        print('Launcher thread didnt start because of exception ' + e)

        time.sleep(6)
    while True:  # << FINAL DISTANCE CALCULATIONS
        start_time = time.time()
        start_Command = 0
        try:
            stereoResult = stereoStack.peek()
            STEREO_DIST = stereoResult.distance
        except:
            continue
##        while not(stereoResult):
##            stereoResult = stereoStack.peek()
                
        print("MainFile STEREO_DIST =  " + str(STEREO_DIST))

        lidarDist = distanceStack.peek()
        while not(lidarDist):
            lidarDist = distanceStack.peek()
            
##        evoDist = EvoRanger()
##        print(evoDist)
##        EVO_DIST = int(evoDist)/1000
        # CONVERT ALL DISTANCES TO METERS

        # Check Validity of DISTANCES
        if isinstance(STEREO_DIST, float):
            finalDist = STEREO_DIST
        else:
            continue  # << Goes back to top of while loop
        
        if isinstance(lidarDist, float):
            if STEREO_DIST - lidarDist > 5:
                total = total - 1
            else:
                finalDist += lidarDist
        else:
            total = total - 1

##        if isinstance(EVO_DIST, float):
##            if STEREO_DIST - EVO_DIST > 5:
##                total = total - 1
##            else:
##                finalDist += EVO_DIST
##        else:
##            total = total - 1
        try:
            FINALDIST = finalDist / total
        except:
            print('in main except')
            continue
        
        end_time = time.time()
        sleeptime = 10 - (end_time - start_time)
        time.sleep(sleeptime)
        if 5 <= float(FINALDIST) <= 25:
            startCommand = 1
            startCommandStack.push(startCommand)
            finalDistStack.push(FINALDIST)
            print('Final Dist Sent')

        # check if variable is an integer isinstance(<var>, int)
        # Ensure Each Distance has a value FIRST



# ________ Run The Program ____________ #
# startMainFile(3,4,"manual")  # for testing purposes only

