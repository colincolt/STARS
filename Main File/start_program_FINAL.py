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
        ser = serial.Serial("/dev/ttyACM1", 9600)  # change ACM number as found from "ls /dev/tty*"
        ser.baudrate = 9600
                        
        guiData = self.getguiStack.peek()
        speed = guiData.speed
        difficulty = guiData.difficulty
        drillType = guiData.drilltype
        # print("Speed:  " + str(speed) + "  Diff:  " + str(difficulty) + "  Type:  " + str(drillType))
        
        while True:
            try:
                print ("starting PitchYaw Thread")
                # distance = self.getdistanceStack.peek()   # << OPTIMIZATION - grab FINAL_DIST
                result = self.getstereoStack.peek()
                if not(result):
                    continue
                LeftXcoord = result.masterval
                RightXcoord = result.slaveval
                distance = int(result.distance)
                
                print(distance)
                
                if distance <= 5:
                    pitchAngle = 25  # 25 DEGREES
                if 5 < distance <= 25:
                    pitchAngle = 45  # # 35 DEGREES

                # inequality = 2464 - LeftXcoord
                # print("LeftXcoord = " + str(LeftXcoord))
                # print("RightXcoord = " + str(RightXcoord))
                # print("inequality = 2464 - LeftXcoord =" + str(inequality))
                ser.reset_input_buffer()
                # ser.set_output_flow_control(True)

                if (abs(2464 - LeftXcoord - RightXcoord) < 150):
                    value = '0'
                    data = '<0, '+ str(pitchAngle) + '>'
                    ser.write(data.encode())
                    #print(0)
                    # sys.stdout.flush()
                    print("locked on to target")
                    time.sleep(0.1)

                elif (2464 - LeftXcoord < RightXcoord):
                    data = '<-80, '+ str(pitchAngle) + '>'
                    ser.write(data.encode())                    
                    #print(-80)
                    # sys.stdout.flush()
                    print("moving CW")
                    time.sleep(0.1)

                elif (2464 - LeftXcoord > RightXcoord):
                    data = '<80, '+ str(pitchAngle) + '>'
                    ser.write(data.encode())                    
                    # print(80)
                    # sys.stdout.flush()
                    print("moving CCW")
                    time.sleep(0.1)
                    
            except Exception as e:
                print('Pitch yaw thread failed because of exception ' + e)
                continue


# __________ Sterepscopic Thread___________ #
def StereoscopicsThread(stereoStack):
    print('Starting Stereoscopics THREAD')
    focalsize = 3.04e-03
    pixelsize = 1.12e-06
    baseline = 0.737
    datapoints = 5
    centroid = (0, 0)
    compvalue = 1

    TCP_IP = '192.168.137.200'
    TCP_PORT = 5005
    BUFFER_SIZE = 20
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    no_talk = True
    while no_talk == True:  # Wait for client
        try:
            s.bind((TCP_IP, TCP_PORT))
            s.listen(1)
            # sock.settimeout(5)
            clientPort, addr = s.accept()
            no_talk = False
        except:
            print('Stereoscopics:       No Client')
            time.sleep(1)
            continue

    print("Client connected")

    ap = argparse.ArgumentParser()
    ap.add_argument("-v", "--video",
                    help="path to the (optional) video file")
    ap.add_argument("-b", "--buffer", type=int, default=8, help="max buffer size")
    args = vars(ap.parse_args())
    ##________________Sockets TCP/IP Communication__________________________________###########

    # define the lower and upper boundaries of the jersey ball in the HSV color space, then initialize the list of tracked points
    jerseyLower1 = (0, 50, 50)  # currently set for red
    jerseyUpper1 = (10, 255, 255)
    jerseyLower2 = (170, 50, 50)  # currently set for red
    jerseyUpper2 = (180, 255, 255)
    pts = deque(maxlen=args["buffer"])

    # initialize array of distance values
    distvals = []

    # if a video path was not supplied, grab the reference
    # to the webcam
    if not args.get("video", False):
        vs = VideoStream(src=0).start()

    # otherwise, grab a reference to the video file
    else:
        vs = cv2.VideoCapture(args["video"])

    # allow the camera or video file to warm up
    time.sleep(2.0)
    print('starting stereo loop')

    ##________________BEGINNING OF LOOP________________##
    while True:
        # grab the current frame
        frame = vs.read()
        # handle the frame from VideoCapture or VideoStream
        frame = frame[1] if args.get("video", False) else frame

        # if we are viewing a video and we did not grab a frame,
        # then we have reached the end of the video
        while frame is None:
            frame = vs.read()
            frame = frame[1] if args.get("video", False) else frame
            

        # resize the frame, blur it, and convert it to the HSV
        # color space
        frame = imutils.resize(frame, width=600)
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # construct a mask for the color "green", then perform
        # a series of dilations and erosions to remove any small
        # blobs left in the mask
        mask0 = cv2.inRange(hsv, jerseyLower1, jerseyUpper1)
        mask1 = cv2.inRange(hsv, jerseyLower2, jerseyUpper2)
        mask = mask0 + mask1
        # greenLower = (29, 86, 6)
        #   greenUpper = (64, 255, 255)
        #   mask = cv2.inRange(hsv, greenLower, greenUpper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        # find contours in the mask and initialize the current
        # (x, y) center of the ball
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)
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
            # print("Center: " + center[0])
            centroid = (round((M["m10"] / M["m00"]), 3), round((M["m01"] / M["m00"]), 3))
            centroid = (centroid[0] * 2464 / 600, centroid[1] * 2464 / 600)
            # print("Centroid: " + str(centroid[0]))

            # only proceed if the radius meets a minimum size
            if radius > 0.1:
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                cv2.circle(frame, center, 5, (0, 0, 255), -1)

        # update the points queue
        pts.appendleft(center)
        # print("center appended")

        # loop over the set of tracked points
        for i in range(1, len(pts)):
            # if either of the tracked points are None, ignore
            # them
            if pts[i - 1] is None or pts[i] is None:
                continue

            # otherwise, compute the thickness of the line and
            # draw the connecting lines
            thickness = int(np.sqrt(args["buffer"] / float(i + 1)) * 2.5)
            cv2.line(frame, pts[i - 1], pts[i], (0, 0, 255), thickness)

        while 1:
            data = clientPort.recv(BUFFER_SIZE)
            while not data:
                data = clientPort.recv(BUFFER_SIZE)
            # print ("received data:", data)
            clientPort.send(data)  # echo
            compvalue = data.decode()
            break

        if len(cnts) > 0:
            # print("Decoded value: " + compvalue)
            slaveval = float(compvalue)
            masterval = centroid[0]
            # masterval = x*2464/600
            # disparity = abs(float(compvalue)-centroid[0])
            disparity = abs(masterval - slaveval)
            distance = (focalsize * baseline) / (disparity * pixelsize)
            # SENDS DATA TO Class, which can be "Put" using Queue's______________________________
            results = StereoOutput
            results.distance = distance
            results.disparity = disparity
            results.masterval = masterval
            results.slaveval = slaveval
            stereoStack.push(results)
            # ##_______________________________________________
##            distvals.append(distance)
##            length = len(distvals)
##            sumweights = 0
##            weight = 0
##            denominator = 0
##            if length < datapoints:
##                average = sum(distvals) / len(distvals)
##            if length > datapoints:
##                distvals.pop(0)
##                for i in range(0, datapoints - 1):
##                    weight = ((i + 1) * distvals[i])
##                    sumweights = sumweights + weight
##                for j in range(1, datapoints):
##                    denominator = denominator + j
##                average = sumweights / denominator
##
##            # average = sum(distvals)/len(distvals)
##            # show the frame to our screen
##            cv2.putText(frame, "Center: " + str(center), (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50, 170, 50), 2)
##            cv2.putText(frame, "Radius: " + str(int(radius)), (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50, 170, 50),
##                        2)
##            cv2.putText(frame, "Distance: " + str(round(average, 2)), (50, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.75,
##                        (50, 170, 50), 2)

        #cv2.imshow("Frame", frame)  # Show the Video feed

    # if we are not using a video file, stop the camera video stream
    if not args.get("video", False):
        vs.stop()

    # otherwise, release the camera
    else:
        vs.release()

    #end = time.time()
    # print(end-start)
    # close all windows
    cv2.destroyAllWindows()
    clientPort.close()


# Connects and communicates with the Arduino Mega: Launcher Motors, Ball Feeder, Wifi, Accelerometer? (Rangefinder?)

class Launcher(Thread):
    def __init__(self, sendVoiceCommandStack, sendDistanceStack, guiStack, getFinalDistStack):
        Thread.__init__(self)
        self.sendVoiceCommandStack = sendVoiceCommandStack
        self.sendDistanceStack = sendDistanceStack
        self.guiStack = guiStack

    def run(self):
        currRestPos = ""
        currDistance = ""
        while True:
            try:
                print("In the Launcher LOOP")
                guiData = self.guiStack.peek()
                speed = guiData.speed
                difficulty = guiData.difficulty
                drillType = guiData.drilltype
                print("LAUNCHER: Speed:  " + str(speed) + "  Diff:  " + str(difficulty) + "  Type:  " + str(drillType))
                
                MEGA = serial.Serial('/dev/ttyACM0', 9600)
                baudrate=9600,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                
                
                # _Send Data As:
                # <2000,3000>
                # __ READ TEMP\

                getdata = False

                while not getdata:
                    tempstr = MEGA.readline()
                    temperature = int(tempstr)
                    connected = True
                
                # receive data from MEGA like this (< 1 2 3 >)

                # GET DISTANCE    
                sendDistanceStack.push(currDistance)
                
                
                # sendVoiceCommandStack.push(data)
                
            except Exception as e:
                print('Launcher thread failed because of exception ' + e)
                continue

#    Evo Rangefinder Port:



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


def EvoRanger():
    print 'Starting Evo Streaming data'

    # Get the port the evo has been connected to
    port = findEvo()

    if port == 'NULL':
        print "Sorry couldn't find the Evo. Exiting."
        sys.exit()
    else:
        evo = openEvo(port)
    
    distance = evo.readline()
    
    while len(distArray) < 2:
        distance = evo.readline()
        while distance == "-Inf" or distance == "+Inf":
            distance = evo.readline()
        try:
            distance = float(distance)
            distArray.append(distance)
            print distance
        except ValueError,e:
            print "error",e
            continue
    
    return distArray
    # evo.close()
    # file.close()
    # sys.exit()

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

    time.sleep(5)
    try:
        pitchYawthread = PitchYaw(stereoStack, guiStack)   # distanceStack
        pitchYawthread.start()
    except Exception as e:
        print('Pitch and Yaw thread didnt start because of exception ' + e)
    try:
        startLauncherThread = Launcher(voiceCommandStack, distanceStack, guiStack)
        startLauncherThread.start()
    except Exception as e:
        print('Launcher thread didnt start because of exception ' + e)


    while True:  # << FINAL DISTANCE CALCULATIONS
        
        stereoResult = stereoStack.peek()
        while not(stereoResult):
            stereoResult = stereoStack.peek()
        stereoDist = stereoResult.distance
        print("MainFile STEREO_DIST =  " + str(STEREO_DIST))

        lidarDist = distanceStack.peek()
        while not(lidarDist):
            lidarDist = distanceStack.peek()
            
        EvoDist  = EvoRanger()
        evoDist1, EvoDist2 = EvoDist
        
        # CONVERT ALL DISTANCES TO METERS
        # check if variable is an integer isinstance(<var>, int)
        # Ensure Each Distance has a value FIRST

        if(stereoDist - lidarDist > 5 and  stereoDist - evoDist1 > 5)
            FINAL_DIST = stereoDist

# ________ Run The Program ____________ #
# startMainFile(3,4,"manual")  # for testing purposes only

