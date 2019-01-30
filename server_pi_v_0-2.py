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

# Packages
from collections import deque  # < Lib for Stacks
from imutils.video import VideoStream
import numpy as np
import argparse
import cv2
import imutils
import time
import socket
import serial
from threading import Event, Thread, Lock
import serial.tools.list_ports
import sys
import os
import binascii
import smtplib
# import statistics
# from picamera.array import PiRGBArray
# from picamera import PiCamera
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
lidar2Stack = Stack()
guiStack = Stack()
voiceCommandStack = Stack()
finalDistStack = Stack()
lidar1Stack = Stack()
temperatureStack = Stack()
futureDistStack = Stack()

shutdown_flag = Event()
distArray = []


# #____________________________________THREADS__________________________________###

# PITCH_YAW_THREAD: communicates to the Arduino Uno in order to provide Angle
# values to both motors, request temperature's from the Uno, and distance measurements
# from the Arduino Mega or Serial.
# These distance measurements are also provided to the LAUNCHER THREAD

# _______PITCH AND YAW THREAD________ #

class PitchYaw(Thread):
    def __init__(self, getstereoStack, getguiStack):  #finalDistStack
        Thread.__init__(self)
        self.getstereoStack = getstereoStack
        # self.getfinalDistStack = getfinalDistStack  # # << OPTIMIZATION - grab FINAL_DIST
        self.getguiStack = getguiStack

    def run(self):
        startData = False
        while not startData:
            # _ARDUINO_UNO__SERIAL_OPEN
            try:
                ser = serial.Serial("/dev/uno", 9600)  # change ACM number as found from "ls /dev/tty*"
                ser.baudrate = 9600
            except serial.SerialException as e:
                print("[PitchYaw(Thread)] : Cannot find Arduino Uno... connect it and restart the application" + e)

            guiData = self.getguiStack.peek()
            # speed = guiData.speed
            # difficulty = guiData.difficulty
            drillType = guiData.drilltype
            if drillType == "Manual" or drillType == "Static" or drillType == "Dynamic":
                startData = True
            else:
                print("[PitchYaw(Thread)] : No start data")
                time.sleep(0.5)
                continue

        # print("Speed:  " + str(speed) + "  Diff:  " + str(difficulty) + "  Type:  " + str(drillType))

        while True:
            try:
                # print ("starting PitchYaw Thread")
                # FINAL_DIST = self.getfinalDistStack.peek()   # << OPTIMIZATION - grab FINAL_DIST
                result = self.getstereoStack.peek()
                if not (result):
                    continue
                LeftXcoord = result.masterval
                RightXcoord = result.slaveval
                distance = int(result.distance)

                # print(distance)

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
                    # value = '0'
                    data = '<0, ' + str(pitchAngle) + '>'
                    ser.write(data.encode())
                    # print(0)
                    # sys.stdout.flush()
                    # print("locked on to target")
                    time.sleep(0.1)

                elif (2464 - LeftXcoord < RightXcoord):
                    data = '<-80, ' + str(pitchAngle) + '>'
                    ser.write(data.encode())
                    # print(-80)
                    # sys.stdout.flush()
                    # print("moving CW")
                    time.sleep(0.1)

                elif (2464 - LeftXcoord > RightXcoord):
                    data = '<80, ' + str(pitchAngle) + '>'
                    ser.write(data.encode())
                    # print(80)
                    # sys.stdout.flush()
                    # print("moving CCW")
                    time.sleep(0.1)
                else:
                    continue

            except Exception as e:
                print('[PitchYaw(Thread)] : failed because of exception ' + e)
                continue

# __________ Sterepscopic Thread___________ #

def StereoscopicsThread(stereoStack):
    # print('Starting Stereoscopics THREAD')
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
    framesize = (600, 450)

    vs = VideoStream(src=0, resolution=framesize, framerate=frameRate).start()

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
            print("received data:", data)
            clientPort.send(data)  # SEND DATA BACK (COULD BE USED FOR STOP COMMAND)
            compvalue = data.decode()
        except socket.error:
            connected = False
            while not connected:
                try:
                    data = clientPort.recv(BUFFER_SIZE)
                    print("received data:", data)
                    clientPort.send(data)  # SEND DATA BACK (COULD BE USED FOR STOP COMMAND)
                    compvalue = data.decode()
                    connected = True
                except socket.error:
                    time.sleep(0.5)

        if len(cnts) > 0:
            # print("Decoded value: " + compvalue)
            slaveval = float(compvalue)
            masterval = centroid[0]
            # masterval = x*2464/600
            # disparity = abs(float(compvalue)-centroid[0])
            disparity = abs(masterval - slaveval)
            distance = (focalsize * baseline) / (disparity * pixelsize)
            # print(distance)
            # SENDS DATA TO Class, which can be "Put" using Queue's______________________________
            results = StereoOutput
            results.distance = distance
            results.disparity = disparity
            results.masterval = masterval
            results.slaveval = slaveval
            stereoStack.push(results)

# _____________LAUNCHER THREAD_____________________#
# Connects and communicates with the Arduino Mega: Launcher Motors, Ball Feeder, Wifi, Accelerometer? (Rangefinder?)
class Launcher(Thread):
    def __init__(self, sendVoiceCommandStack, sendLidar2Stack, guiStack, getStereoStack, getLidar1Stack, sendfinalDistStack, sendTemperatureStack, getfutureDist):
        Thread.__init__(self)
        self.sendVoiceCommandStack = sendVoiceCommandStack
        self.sendLidar2Stack = sendLidar2Stack
        self.guiStack = guiStack
        self.getStereoStack = getStereoStack
        self.getLidar1Stack = getLidar1Stack
        self.sendfinalDistStack = sendfinalDistStack
        self.sendTemperatureStack = sendTemperatureStack
        self.getfutureDist = getfutureDist
        self.shutdown_flag = Event()                # <<< SHUTDOWN FLAG


    def run(self):
        # _____ Launcher common variables:
        beginVC = 1
        stopVC = 2
        fasterVC = 3
        slowerVC = 4
        pauseVC = 5
        stereo_Distance = 0.0
        drillCount = 0
        FUT_FINAL_DIST = 0.0

        # _____ open MEGA serial port
        try:
            MEGA = serial.Serial('/dev/mega', 9600)
            MEGA.timeout = None
            baudrate = 9600,
            parity = serial.PARITY_NONE,
            stopbits = serial.STOPBITS_ONE,
            bytesize = serial.EIGHTBITS,
            print("[Launcher(Thread)] : Connected to MEGA")
        except serial.SerialException as e:
            print("[Launcher(Thread)] : Cannot find Arduino Mega... connect it and restart the application")
            self.shutdown_flag.set()


        def GetMegaData():
            getdata = False
            dataPresent = False

            while getdata == False:
                print('[Launcher(Thread)] : in Mega.Serial while 1')
                if MEGA.is_open:
                    print('[Launcher(Thread)] : in the Mega.is_open')
                    while not dataPresent:
                        print('[Launcher(Thread)] : in Mega.Serial while 2')
                        try:
                            print('[Launcher(Thread)] : trying to read startMarker')
                            startMarker = MEGA.read().decode()
                        except Exception as e:
                            print('[Launcher(Thread)] : didnt get Mega data' + e)
                        if startMarker == "<":
                            megaDataStr = MEGA.read(25)  # READ DATA FORMATTED AS ('< 1 2 3 4 5 >')
                            megaDataTemp = list(megaDataStr.decode())
                            megaDataTemp.insert(0, size)
                            megaData = megaDataTemp[:megaDataTemp.index(">") + 1]
                            print("[Launcher(Thread)] : acquired MegaData  ->  ", megaData)
                            tempData = "".join(megaData)
                            print("[Launcher(Thread)] : tempData string  ->  ", tempData)
                            dataPresent = True
                            print('[Launcher(Thread)] : data read from Mega')
                            getdata = True
                            return tempData
                else:
                    continue
                    time.sleep(0.4)


                # print(megaDataStr)
                # time.sleep(0.4) # < MAYBE IMPORTANT
                # megaDataList =  megaDataStr.decode().split(',')  # < split() divides the string at the spaces



        try:
            getMegaData = GetMegaData()
            # Get TEMPERATURE and send to PITCH_YAW Thread
            temperature = int(getMegaData.strip().split(",")[1])  # temperature = int()
            voiceCommand = int(getMegaData.strip().split(",")[2])  # voice commands = int(from 1 to 5)
            self.sendTemperatureStack.push(temperature)

            # ****SET TEMPERATURE CORRECTION FACTOR*****
            tempCorrect = temperature/25
            # *****************************************

            # Get drillType data from GUI__ #
            guiData = self.guiStack.peek()
            drillSpeed = guiData.speed
            difficulty = guiData.difficulty
            drillType = guiData.drilltype
            #print("LAUNCHER: Speed:  " + str(drillSpeed) + "  Diff:  " + str(difficulty) + "  Type:  " + str(drillType))
            while voiceCommand != beginVC:
                getMegaData = GetMegaData()
                voiceCommand = int(getMegaData.strip().split(",")[2])  # voice commands = int(from 1 to 5)
                time.sleep(0.5)

        except Exception as e:
            print("[Launcher(Thread)] : Error in setup" + e)
            self.shutdown_flag.set()


        print("[Launcher(Thread)] : Starting LOOP")

        while drillCount <= 5 and not self.shutdown_flag.isSet():
        #while voiceCommand != stopVC: # VC[2] = "STOP"
            try:
                if drillType == 'Dynamic':

                    TimeSinceLastLaunch = time.time()
                    # ___________________ RECEIVE STEREO DISTANCE ______________________________________________#

                    try:
                        oldstereo_Distance = stereoDist
                        stereoData = self.getStereoStack.peek()
                        stereoDist = float(stereoData.distance)
                    except Exception as q:
                        print("[Launcher(Thread)] : No data in stereoStack" + q)
                        while stereo_Distance == oldstereo_Distance:
                            stereoResult = stereoStack.peek()
                            stereo_Distance = stereoResult.distance

                    distanceTotal = stereo_Distance
                    rationaleDistMeasures = 1
                    # ___________________ RECEIVE MEGA DATA STRING ______________________________________________#
                    try:
                        tempData = GetMegaData
                    except:
                        success = False
                        while not success:
                            try:
                                tempData = GetMegaData
                                success = True
                            except Exception as r:
                                print("[Launcher(Thread)] : GetMegaData failed" + r)
                                continue

                    lidar_2_Distance = int(tempData.strip("<").strip().split(",")[0])  # lidarDistance = int(cm)
                    # temperature = int(tempData.strip().split(",")[1])                            # temperature = int()
                    voiceCommand = int(tempData.strip().split(",")[2])  # voice commands = int(from 1 to 5)
                    targetTiming = float(tempData.strip().split(",")[3])  # targetTiming = float(0.0)
                    targetBallSpeed = float(tempData.strip().split(",").strip(">")[4])  # targetBallSpeed
                    # ___________________END OF RECEIVE MEGA DATA STRING ______________________________________________#
                    if lidar_2_Distance == 0:
                        lidar_2_Distance = None

                    #self.sendVoiceCommandStack.push()
                    # Handle Voice input:
                    if voiceCommand == stopVC:
                        self.shutdown_flag.set()
                    elif voiceCommand == fasterVC:
                        drillSpeed += 1
                    elif voiceCommand == slowerVC:
                        drillSpeed -= 1
                    elif voiceCommand == pauseVC:
                        print("[Launcher(Thread)] : Sorry no Pause function")


                    if lidar_2_Distance is not None and abs(stereo_Distance - lidar_2_Distance) <= 5:
                        distanceTotal += lidar_2_Distance
                        rationaleDistMeasures += 1
                        lidar_2_Distance = lidar_2_Distance / 100  # << convert LIDAR 2 from cm to meters
                        self.sendLidar2Stack.push(lidar_2_Distance)
                    else:
                        self.sendLidar2Stack.push(None)     # <<send empty value so we know theres no new data

                    try:
                        lidar_1_Distance = self.getlidar1Stack.peek()
                        if lidar_1_Distance is not None and abs(stereo_Distance - lidar_1_Distance) <= 5:  # <<<<<<USE WEIGHTING FACTOR INSTEAD
                            rationaleDistMeasures += 1
                            distanceTotal += lidar_1_Distance

                    except Exception as w:
                        print("[Launcher(Thread)] : LIDAR_1 -> nothing in Lidar1Stack" + w)
                        continue

                    # CALCULATE AND SEND TOTAL TO MAIN THREAD
                    FINAL_DIST = distanceTotal / rationaleDistMeasures
                    self.sendfinalDistStack.push(FINAL_DIST)

                    # ____QUERY LOOKUP TABLE FOR MOTOR_SPEED VALUES ___ #

                    #
                    #
                    # To determine:
                    motorSpeed1 = 100   # Value between 0-255 (On 24 V: 0-5000 RPM)
                    motorSpeed2 = 100


                    # In DYNAMIC Mode, the motors spin up before receiving final instructions from MAIN THREAD
                    # ____SEND MEGA DATA STRING _____#    IF NO VALUE SEND AS '-1'
                    MotorSpeed1 = str(motorSpeed1)
                    MotorSpeed2 = str(motorSpeed2)
                    # ***Randomize targetChoice
                    targetChoice = '2'
                    # **** NEED TO ADD A FIFTH VALUE FOR FEED/NO-FEED OF BALL****
                    ballFeed = 0
                    data = '<' + MotorSpeed1 + ',' + MotorSpeed2 + ',' + targetChoice + ',' + str(difficulty) + '>' # + ',' + str(ballFeed)
                    # Write data to MEGA
                    try:
                        MEGA.write(data.encode())
                    except Exception as e:
                        sendsuccess = False
                        while not sendsuccess:
                            try:
                                MEGA.write(data.encode())
                                sendsuccess = True
                            except:
                                if time.time() - TimeSinceLastLaunch >= 8:
                                    self.shutdown_flag.set()


                    OLD_FUT_FINAL_DIST = FUT_FINAL_DIST
                    try:
                        FUT_FINAL_DIST = self.getfutureDist.peek()      # <<<<< GET PREDICTED LOCATION
                    except:
                        print("[Launcher(Thread)] : No FUT_FINAL_DIST data in futureDistStack")

                    if FUT_FINAL_DIST == OLD_FUT_FINAL_DIST:
                        print("[Launcher(Thread)] : No NEW value for FUT_FINAL_DIST")

                        # ****GET READY SIGNAL/EVENT/FLAG FROM PITCHYAW THREAD*****
                        #
                        #
                        #

                        ballFeed = 1
                        data = '<' + MotorSpeed1 + ',' + MotorSpeed2 + ',' + targetChoice + ',' + str(difficulty) + '>'  # + ',' + str(ballFeed)
                        # write data to MEGA
                        if 5 <= (time.time() - TimeSinceLastLaunch) <= 8:       # << Not elegant but restricts continuous launch signals
                            try:
                                MEGA.write(data.encode())
                                drillCount += 1  # << ON SUCCESSFUL LAUNCH
                                print("[Launcher(Thread)] : Launch signal sent to MEGA")
                            except Exception as e:
                                sendsuccess = False
                                while not sendsuccess:
                                    try:
                                        MEGA.write(data.encode())
                                        drillCount += 1  # << ON SUCCESSFUL LAUNCH
                                        print("[Launcher(Thread)] : Launch signal sent to MEGA")
                                        sendsuccess = True
                                    except:
                                        if time.time() - TimeSinceLastLaunch >= 8:
                                            print("[Launcher(Thread)] : process took too long to keep up with drill")
                                            self.shutdown_flag.set()
                        else:
                            print("[Launcher(Thread)] : process took too long to keep up with drill")
                            self.shutdown_flag.set()
                    else:
                        # ____QUERY LOOKUP TABLE FOR NEW MOTOR_SPEED VALUES ___ #

                        #
                        #
                        # To determine:
                        motorSpeed1 = 120  # Value between 0-255 (On 24 V: 0-5000 RPM)
                        motorSpeed2 = 120

                        # In DYNAMIC Mode, the motors spin up before receiving final instructions from MAIN THREAD
                        # ____SEND MEGA DATA STRING _____#    IF NO VALUE SEND AS '-1'
                        MotorSpeed1 = str(motorSpeed1)
                        MotorSpeed2 = str(motorSpeed2)

                        # **** NEED TO ADD A FIFTH VALUE FOR FEED/NO-FEED OF BALL****
                        ballFeed = 1
                        data = '<' + MotorSpeed1 + ',' + MotorSpeed2 + ',' + targetChoice + ',' + str(difficulty) + '>'  # + ',' + str(ballFeed)
                        # Write data to MEGA
                        if 5 <= (time.time() - TimeSinceLastLaunch) <= 8:       # << Not elegant but restricts continuous launch signals
                            try:
                                MEGA.write(data.encode())
                                drillCount += 1  # << ON SUCCESSFUL LAUNCH
                                print("[Launcher(Thread)] : Launch signal sent to MEGA")
                            except Exception as e:
                                sendsuccess = False
                                while not sendsuccess:
                                    try:
                                        MEGA.write(data.encode())
                                        drillCount += 1  # << ON SUCCESSFUL LAUNCH
                                        print("[Launcher(Thread)] : Launch signal sent to MEGA")
                                        sendsuccess = True
                                    except:
                                        if time.time() - TimeSinceLastLaunch >= 8:
                                            self.shutdown_flag.set()
                        else:
                            print("[Launcher(Thread)] : process took too long to keep up with drill")
                            self.shutdown_flag.set()

                    # FOR MANUAL LOOKUP W/OUT TABLES:
                    # print("Launcher(Thread)] : FINAL_DIST = " + str(FINAL_DIST))
                    # if FINAL_DIST < 5:
                    #     motorSpeed1 = motorSpeed2 = 50
                    # elif 5 <= FINAL_DIST < 10:
                    #     motorSpeed1 = motorSpeed2 = 80
                    # elif 10 <= FINAL_DIST < 20:
                    #     motorSpeed1 = motorSpeed2 = 100
                    # elif 20 <= FINAL_DIST < 50:
                    #     motorSpeed1 = motorSpeed2 = 125
                    # else:
                    #     motorSpeed1 = motorSpeed2 = 0
                    #     print('out of range')

                    # sendVoiceCommandStack.push(data)
                if drillType == 'Static':
                    # Do some static stuff
                if drillType == 'Manual':
                    # Do some manual stuff

            except Exception as e:
                print('[Launcher(Thread)] : failed because of exception: ' + e)
                self.shutdown_flag.set()

        print("[Launcher(Thread)] : shutdown_flag detected!")

###    Evo Rangefinder: LIDAR_1:
def Lidar1Dist():
    while (1):
        try:
            distance = evo.readline()
            if distance == "-Inf" or distance == "+Inf":
                print(distance)
            else:
                try:
                    distance = float(distance)
                    # print(distance)
                    return distance
                except Exception as e:
                    print("[MainThread/Lidar1Dist] : error converting to float", e)
                    return None
        except serial.SerialException as a:
            print("[MainThread/Lidar1Dist] : No Evo Lidar present... connect it and restart the application" + a)
            return None


# MAIN FILE START
def startMainFile(speed, difficulty, drillType):  # , args): ## NOT A THREAD, performs the bulk of calculation
    # # _______________________________Main Processing_____________________________________

    # __ VARIABLES _____#
    # startCommand = 0
    stereo_Distance = 0.0

    # __ GUI Input __ #
    guiData = GuiInput
    guiData.speed = speed
    guiData.difficulty = difficulty
    guiData.drilltype = drillType
    print("[MainThread] : _____________")
    print("Speed:  " + str(guiData.speed) + "  " + "Diff:  " + str(guiData.difficulty) + "  " + "Drill:  " + guiData.drilltype)
    print("_______________________________________")

    guiStack.push(guiData)
    # ___ OPEN SERIAL PORT/S ___ #
    try:
        evo = serial.Serial("/dev/evo", baudrate=115200, timeout=2)
        set_text = (0x00, 0x11, 0x01, 0x45)
        evo.flushInput()
        evo.write(set_text)
        evo.flushOutput()
        print("[MainThread] : Connected to Evo (LIDAR1)")
    except serial.SerialException as e:
        print("[MainThread] : Cannot find Evo LIDAR1... connect it and restart the application")
    # ___ Start Threads ___ #

    # ___Begin Stereoscopic Thread:____ #
    try:
        process = Thread(target=StereoscopicsThread, args=[stereoStack])
        process.start()
    except Exception as e:
        print('[MainThread] : Stereo thread failed because of exception ' + e)

    time.sleep(6)
    try:
        pitchYawthread = PitchYaw(stereoStack, guiStack)  # distanceStack
        pitchYawthread.start()
    except Exception as e:
        print('[MainThread] : Pitch and Yaw thread didnt start because of exception ' + e)
    try:
        startLauncherThread = Launcher(voiceCommandStack, lidar2Stack, guiStack, stereoStack, lidar1Stack, finalDistStack, temperatureStack, futureDistStack)
        startLauncherThread.start()
    except Exception as e:
        print('[MainThread] : Launcher thread didnt start because of exception ' + e)

        time.sleep(1)

    # ___________ "MAIN THREAD" LOOP __________ #

    while True:
        if drillType == "Dynamic":
            # Get(WAIT) for stereoDistance _____________________________
            try:
                oldstereo_Distance = stereo_Distance    # ****< NEED TO ENSURE THE STEREO.DISTANCE IS NEVER THE SAME TWICE IN A ROW
                stereoResult = stereoStack.peek()
                stereo_Distance = stereoResult.distance
            except Exception as q:
                print("[MainThread] : No data in stereoStack" + q)
                while stereo_Distance == oldstereo_Distance:
                    stereoResult = stereoStack.peek()
                    stereo_Distance = stereoResult.distance

            if 5 <= stereo_Distance <= 30: # Meters
                rationaleDistMeasures = 1
                distanceTotal = stereo_Distance

                # Get(NO_WAIT) for Lidar_1_Dist _____________________________
                try:
                    LIDAR_1_Distance = Lidar1Dist
                    if LIDAR_1_Distance is not None and abs(stereo_Distance - LIDAR_1_Distance) <= 5:  # <<<<<<USE WEIGHTING FACTOR INSTEAD
                        rationaleDistMeasures += 1
                        distanceTotal += LIDAR_1_Distance
                        lidar1Stack.push(LIDAR_1_Distance)
                    else:
                        lidar1Stack.push(None)  # << None value indicates no GOOD new data

                except Exception as w:
                    print("[MainThread] : LIDAR_1 -> no data" + w)
                    continue

                # Get no_wait(Run on New Data EVENT() trigger?) for Lidar_2_Dist _____________________________
                try:
                    LIDAR_2_Distance = lidar2Stack.peek()
                    if LIDAR_2_Distance is not None and abs(stereo_Distance - LIDAR_2_Distance) <= 5:  # <<<<<<USE WEIGHTING FACTOR INSTEAD
                        rationaleDistMeasures += 1
                        distanceTotal += LIDAR_2_Distance
                except Exception as w:
                    print("[MainThread] : LIDAR_2 -> no data" + w)
                    continue

                PRE_FINAL_DIST = distanceTotal / rationaleDistMeasures

                # MAKE SURE LAUNCHER THREAD OBTAINS SIMILAR VALUE:
                # if newFinalDistLauncher_flag() = True
                    # Launcher_FINAL_DIST = finalDistStack.peek()
                    # if abs(Launcher_FINAL_DIST - PRE_FINAL_DIST) <= 2 # Meters
                        # Do everything
                    # else:
                        # print("[Main Thread] : Something is wrong with the data flow")

                # ______________ **SOME PREDICTION METHOD** ______________
                # Collect 10-20 PRE_FINAL_DIST + rationaleDistMeasures in an indexed list or dictionary
                # weight the accuracy of PRE_FINAL_DIST base on the # of rationaleDistMeasures
                # Calculate speed and z-travel direction
                # Determine location in 3-4 seconds assuming trajectory is continued

                FUT_FINAL_DIST = PRE_FINAL_DIST # For now

                futureDistStack.push(FUT_FINAL_DIST)

            else:
                print("Player is not in view")
                # Activate GPIO pin for notification LED
                time.sleep(0.5)

        elif drillType == "Static":
            print("[MainThread] : Main doesnt do much here")

        elif drillType == "Manual":
            print("[MainThread] : Main doesnt do much here")

        else:
            print("[MainThread] : no GUI data")


# ________ Run The Program ____________ #
# startMainFile(3,4,"manual")  # for testing purposes only

