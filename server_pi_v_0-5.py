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
import math

# import statistics
# from picamera.array import PiRGBArray
# from picamera import PiCamera
# from queue import LifoQueue
# import struct
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


class MegaDataClass:
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
megaDataStack = Stack()
finalDistStack = Stack()
lidar1Stack = Stack()
temperatureStack = Stack()
futureDistStack = Stack()

shutdown_flag = Event()
py_shutdown_flag = Event()
py_lock = Lock()
distArray = []


# #____________________________________THREADS__________________________________###

# PITCH_YAW_THREAD: communicates to the Arduino Uno in order to provide Angle
# values to both motors, request temperature's from the Uno, and distance measurements
# from the Arduino Mega or Serial.
# These distance measurements are also provided to the LAUNCHER THREAD

# _______PITCH AND YAW THREAD________ #

class PitchYaw(Thread):
    def __init__(self, getstereoStack, getguiStack, getTemperatureStack, getmegaDataStack, getfinalDistStack, getfutureDistStack):
        Thread.__init__(self)
        self.getstereoStack = getstereoStack
        self.getguiStack = getguiStack
        self.getTemperatureStack = getTemperatureStack
        self.getmegaDataStack = getmegaDataStack
        self.getfinalDistStack = getfinalDistStack
        self.getfutureDistStack = getfutureDistStack
        self.py_shutdown_flag = Event()

    def run(self):
        # ** __ RECORD A LIST OF OLD MEASUREMENTS FOR TRACKING: __ ** #
        pyDistDeque = deque([])
        latDispDeque = deque([])
        latTimeDeque = deque([])

        # ** __ DATA TRACKERS: __ ** #
        startData = False
        oldLatStartTime = None
        oldzDist = None

        # ** __ IMPORTANT VARIABLES: __ ** #
        LEAD_SPEED = 25                         # << Adjust the added motorSpeed of the YAW for DYNAMIC MODE (0-255)
        MAX_YAW_SPEED = 0.15                    # << Maximum speed of Yaw in radians
        pitchAngleTable = ([[6, 16]             # << Pitch angle lookup table based on estimations
                            [8, 18]
                            [10, 20]
                            [12, 22]
                            [14, 24]
                            [16, 26]
                            [18, 28]
                            [20, 30]
                            [22, 32]
                            [24, 34]
                            [26, 36]
                            [28, 38]
                            [30, 40]
                            [32, 42]
                            [34, 44]])

        while not startData:
            # _ARDUINO_UNO__SERIAL_OPEN
            connectUno = False
            while not connectUno:
                try:
                    UNO = serial.Serial("/dev/uno", 9600)  # change ACM number as found from "ls /dev/tty*"
                    UNO.baudrate = 9600
                    connectUno = True
                except serial.SerialException as e:
                    print("[PitchYaw(Thread)] : Cannot find Arduino Uno... connect it and restart the application" + e)
                    continue

            # ACQUIRE ACCELEROMETER DATA (Single Pitch reading) _______ #
            tempAngle = GetUnoData(UNO)
            launcherAngle = round(tempAngle.strip("<").strip().strip(">")[0])  # lidarDistance = int(cm)
            # _____________________________________________________ #

            temperaturedata = False
            # Get Temperature Data
            while not temperaturedata:
                try:
                    temperature = self.getTemperatureStack.peek()
                    tempCorrection = temperature / 25               # <<<tempCorrection factor
                    temperaturedata = True
                except:
                    print("[PithYaw(Thread)] : no data in temperatureStack")
                    continue
            # Get GUI Data
            guiData = self.getguiStack.peek()
            drillSpeed = guiData.speed
            # difficulty = guiData.difficulty
            drillType = guiData.drilltype
            if drillType == "Manual" or drillType == "Static" or drillType == "Dynamic":
                startData = True
            else:
                print("[PitchYaw(Thread)] : No start data")
                continue

        # print("[PitchYaw(Thread)] : Speed:  " + str(speed) + "  Diff:  " + str(difficulty) + "  Type:  " + str(drillType))
        print ("[PitchYaw(Thread)] : starting Loop")

        while not self.py_shutdown_flag.isSet():    # <<< BEGINNING OF PITCHYAW LOOP _________________________________ ** #
            if drillType == "Dynamic":
                try:
                    cameradata = False
                    while not cameradata:
                        try:
                            stereodata = self.getstereoStack.peek()
                            cameradata = True
                        except:
                            print("[PithYaw(Thread)] : no data in stereoStack")
                            self.py_shutdown_flag.set()

                    LeftXcoord = stereodata.masterval
                    RightXcoord = stereodata.slaveval
                    stereoDist = int(stereodata.distance)


                    latStartTime = time.time()
                    if oldLatStartTime is None:
                        oldLatStartTime = latStartTime
                        latTimeDeque.appendleft(None)
                    else:
                        latDispTime = oldLatStartTime - latStartTime
                        latTimeDeque.appendleft(latDispTime)
                        if len(latTimeDeque) > 10:
                            latTimeDeque.pop()

                    # Try for FUT_FINAL_DIST _______________
                    try:
                        FUT_FINAL_DIST = self.getfutureDistStack.peek()
                        futureDist = True
                    except:
                        futureDist = False
                        # Try for FINAL_DIST from Launcher(Thread) _______________
                        try:
                            FINAL_DIST = self.getfinalDistStack.peek()
                            finalDist = True
                        except:
                            finalDist = False
                            pass

                    # **________ TWO POSSIBLE CASES: FUTURE_DIST IS AVAILABLE OR NOT ________** #

                    if futureDist:
                        # << CASE 1: Only have to 'predict'/anticipate future lateral displacement _________
                        usedDistance = FUT_FINAL_DIST
                        pyDistDeque.appendleft(usedDistance)
                        if len(pyDistDeque) > 10:
                            pyDistDeque.pop()

                        # ** _________________ PITCH ANGLE: __________________ ** #
                        # Query table for angle at usedDistance
                        row = 6 - int(round(usedDistance / 2)) * 2
                        if row < 0: row = 0
                        elif row > 14: row = 14
                        col = 1
                        pitchAngle = pitchAngleTable(row, col) + launcherAngle # << ANGLE IS ALREADY SET BASED ON FUTURE DISTANCE

                        # ** ___________________ YAW MOTOR SPEED: ______________________ ** #
                        latPixelDisp = (2464 - LeftXcoord - RightXcoord)                            # << DICTATES SIGN OF MOTORSPEED
                        PixToDistApprox = usedDistance / 250  # <<< nonsense Conversion at the moment
                        lateralDisplacement = latPixelDisp * PixToDistApprox
                        latDispDeque.appendleft(lateralDisplacement)
                        if len(latDispDeque) > 10:
                            latDispDeque.pop()

                        # IF MULTIPLE RECORDED MEASUREMENTS, BASE MOTORSPEED ESTIMATION OFF PLAYERS SPEED ESTIMATION (V = omega*R):
                        if latTimeDeque[0] is not None:
                            latSpeed = (latDispDeque[1] - latDispDeque[0]) / latTimeDeque[0]
                            angularSpeed = ( abs(latSpeed) / usedDistance ) * 3.14              #<< Convert to radians
                            # CURRENT MAXIMUM ANGULAR SPEED: Omega = 6 deg/sec OR 0.15
                            motorSpeed = ( angularSpeed / MAX_YAW_SPEED) * 255
                            # For Dynamic, we want to be leading the players movement:
                            motorSpeed = motorSpeed + LEAD_SPEED
                            if motorSpeed >= 255: motorSpeed = 255
                            if motorSpeed <= 80: motorSpeed = 80
                            # ** ___ SEND DATA ___ ** #
                            data = '<' + str(motorSpeed) + ', ' + str(pitchAngle) + '>'
                            UNO.write(data.encode())
                        # ** ____________________________________________________________________ ** #

                        # IF THIS IS THE FIRST MEASUREMENT JUST USE 150 for YAW SPEED
                        else:
                            motorSpeed = 150
                            if (abs(latPixelDisp) < 150):
                                data = '<0, ' + str(pitchAngle) + '>'
                                UNO.write(data.encode())
                                # sys.stdout.flush()
                                # print("locked on to target")
                                time.sleep(0.1)

                            elif (2464 - LeftXcoord < RightXcoord):
                                data = '<' + str(motorSpeed) + ', ' + str(pitchAngle) + '>'
                                UNO.write(data.encode())
                                # sys.stdout.flush()
                                # print("moving CW")
                                time.sleep(0.1)

                            elif (2464 - LeftXcoord > RightXcoord):
                                data = '<' + str(motorSpeed) + ', ' + str(pitchAngle) + '>'
                                UNO.write(data.encode())
                                # sys.stdout.flush()
                                # print("moving CCW")
                                time.sleep(0.1)
                            else:
                                continue
                        # ** ____________________________________________________________________ ** #

                    else:
                        if finalDist:
                            usedDistance = FINAL_DIST
                        else:
                            usedDistance = stereoDist
                        # << CASE 2: Have to predict FUT_FINAL_DIST ourselves if we dont get it
                        # ** ________"PREDICT" FUT_FINAL_DIST (z-distance) of the player from available data: ________ ** #
                        # __________________ IF THIS IS THE FIRST MEASUREMENT: ____________
                        if oldzDist is None:
                            oldzDist = stereoDist
                            speed = 0
                            FUT_FINAL_DIST = usedDistance
                        else:
                            tempDist = usedDistance - oldzDist
                            speed = tempDist / latTimeDeque[0] # meters/second
                            # the idea is to stay ahead of the player by at least a second or two
                            FUT_FINAL_DIST = usedDistance + speed * 2

                        # ** ________________________PITCH ANGLE: ______________________ ** #
                        # Query table for angle at usedDistance
                        row = 6 - int(
                            round((FUT_FINAL_DIST / 2) * 2))  # << ENSURE THIS IS A MULTIPLE OF 2 BETWEEN 6-34
                        if row < 0:
                            row = 0
                        elif row > 14:
                            row = 14
                        col = 1
                        pitchAngle = pitchAngleTable(row, col)  # << ANGLE IS ALREADY SET BASED ON FUTURE DISTANCE

                        # ** ____________________________________________________________________ ** #

                        pyDistDeque.appendleft(FUT_FINAL_DIST)
                        if len(pyDistDeque) > 10:
                            pyDistDeque.pop()

                        # ** ___________________ YAW MOTOR SPEED: ______________________ ** #
                        latPixelDisp = (2464 - LeftXcoord - RightXcoord)
                        PixToDistApprox = usedDistance / 250  # <<< nonsense Conversion at the moment
                        lateralDisplacement = latPixelDisp * PixToDistApprox
                        latDispDeque.appendleft(lateralDisplacement)
                        if len(latDispDeque) > 10:
                            latDispDeque.pop()

                        # IF MULTIPLE RECORDED MEASUREMENTS, BASE MOTORSPEED ESTIMATION OFF PLAYERS SPEED ESTIMATION (V = omega*R):
                        if latTimeDeque[0] is not None:
                            latSpeed = (latDispDeque[1] - latDispDeque[0]) / latTimeDeque[0]
                            angularSpeed = (abs(latSpeed) / usedDistance) * 3.14  # << Convert to radians
                            # CURRENT MAXIMUM ANGULAR SPEED: Omega = 6 deg/sec OR 0.15
                            motorSpeed = (angularSpeed / MAX_YAW_SPEED) * 255
                            # For Dynamic, we want to be leading the players movement:
                            motorSpeed = motorSpeed + LEAD_SPEED
                            if motorSpeed >= 255: motorSpeed = 255
                            if motorSpeed <= 80: motorSpeed = 80
                            # ** ___ SEND DATA ___ ** #
                            data = '<' + str(motorSpeed) + ', ' + str(pitchAngle) + '>'
                            UNO.write(data.encode())
                        # ** ____________________________________________________________________ ** #

                        # IF THIS IS THE FIRST MEASUREMENT JUST USE 150 for YAW SPEED
                        else:
                            motorSpeed = 150
                            if (abs(latPixelDisp) < 150):
                                data = '<0, ' + str(pitchAngle) + '>'
                                UNO.write(data.encode())
                                # sys.stdout.flush()
                                # print("locked on to target")
                                time.sleep(0.1)

                            elif (2464 - LeftXcoord < RightXcoord):
                                data = '<' + str(motorSpeed) + ', ' + str(pitchAngle) + '>'
                                UNO.write(data.encode())
                                # sys.stdout.flush()
                                # print("moving CW")
                                time.sleep(0.1)

                            elif (2464 - LeftXcoord > RightXcoord):
                                data = '<' + str(motorSpeed) + ', ' + str(pitchAngle) + '>'
                                UNO.write(data.encode())
                                # sys.stdout.flush()
                                # print("moving CCW")
                                time.sleep(0.1)
                            else:
                                continue
                        # ** ____________________________________________________________________ ** #

                except Exception as e:
                    print('[PitchYaw(Thread)] : failed because of exception ' + e)
                    continue

            if drillType == "Static" or drillType == "Manual":
                try:
                    cameradata = False
                    while not cameradata:
                        try:
                            stereodata = self.getstereoStack.peek()
                            cameradata = True
                        except:
                            print("[PithYaw(Thread)] : no data in stereoStack")
                            self.py_shutdown_flag.set()

                    LeftXcoord = stereodata.masterval
                    RightXcoord = stereodata.slaveval
                    stereoDist = int(stereodata.distance)

                    latStartTime = time.time()
                    if oldLatStartTime is None:
                        oldLatStartTime = latStartTime
                        latTimeDeque.appendleft(None)
                    else:
                        latDispTime = oldLatStartTime - latStartTime
                        latTimeDeque.appendleft(latDispTime)            # << Deque for time between measurements
                        oldLatStartTime = latStartTime
                        if len(latTimeDeque) > 10:
                            latTimeDeque.pop()

                    # Try for FINAL_DIST _______________
                    try:
                        FINAL_DIST = self.getfinalDistStack.peek()
                        finalDist = True
                    except:
                        finalDist = False
                        pass

                    if finalDist:
                        usedDistance = FINAL_DIST
                    else:
                        usedDistance = stereoDist

                    pyDistDeque.appendleft(usedDistance)
                    if len(pyDistDeque) > 10:
                        pyDistDeque.pop()

                    # ** _________________ PITCH ANGLE: __________________ ** #
                    # Query table for angle at usedDistance
                    row = 6 - int(round(usedDistance / 2)) * 2
                    if row < 0: row = 0
                    elif row > 14: row = 14
                    col = 1
                    pitchAngle = pitchAngleTable(row, col) + launcherAngle # << ANGLE IS ALREADY SET BASED ON FUTURE DISTANCE

                    # ** ___________________ YAW MOTOR SPEED: ______________________ ** #
                    latPixelDisp = (2464 - LeftXcoord - RightXcoord)  # << DICTATES SIGN OF MOTORSPEED
                    PixToDistApprox = usedDistance / 250  # <<< nonsense Conversion at the moment
                    lateralDisplacement = latPixelDisp * PixToDistApprox
                    latDispDeque.appendleft(lateralDisplacement)
                    if len(latDispDeque) > 10:
                        latDispDeque.pop()

                    # IF MULTIPLE RECORDED MEASUREMENTS, BASE MOTORSPEED ESTIMATION OFF PLAYERS SPEED ESTIMATION (V = omega*R):
                    if latTimeDeque[0] is not None:
                        latSpeed = (latDispDeque[1] - latDispDeque[0]) / latTimeDeque[0]
                        angularSpeed = (abs(latSpeed) / usedDistance) * 3.14  # << Convert to radians
                        # CURRENT MAXIMUM ANGULAR SPEED: Omega = 6 deg/sec OR 0.15
                        motorSpeed = (angularSpeed / MAX_YAW_SPEED) * 255
                        if motorSpeed >= 255: motorSpeed = 255
                        if motorSpeed <= 80: motorSpeed = 80
                        # ** ___ SEND DATA ___ ** #
                        data = '<' + str(motorSpeed) + ', ' + str(pitchAngle) + '>'
                        UNO.write(data.encode())
                    # ** ____________________________________________________________________ ** #

                    # IF THIS IS THE FIRST MEASUREMENT JUST USE 150 for YAW SPEED
                    else:
                        motorSpeed = 150
                        if (abs(latPixelDisp) < 150):
                            data = '<0, ' + str(pitchAngle) + '>'
                            UNO.write(data.encode())
                            # sys.stdout.flush()
                            # print("locked on to target")
                            time.sleep(0.1)

                        elif (2464 - LeftXcoord < RightXcoord):
                            data = '<' + str(motorSpeed) + ', ' + str(pitchAngle) + '>'
                            UNO.write(data.encode())
                            # sys.stdout.flush()
                            # print("moving CW")
                            time.sleep(0.1)

                        elif (2464 - LeftXcoord > RightXcoord):
                            data = '<' + str(motorSpeed) + ', ' + str(pitchAngle) + '>'
                            UNO.write(data.encode())
                            # sys.stdout.flush()
                            # print("moving CCW")
                            time.sleep(0.1)
                        else:
                            continue

                except Exception as e:
                    print('[PitchYaw(Thread)] : failed because of exception ' + e)
                    continue

# __________ Sterepscopic Thread___________ #

# _____________LAUNCHER THREAD_____________________#
# Connects and communicates with the Arduino Mega: Launcher Motors, Ball Feeder, Wifi, Accelerometer? (Rangefinder?)
class Launcher(Thread):
    def __init__(self, sendMegaDataStack, sendLidar2Stack, guiStack, getStereoStack, getLidar1Stack, sendfinalDistStack, sendTemperatureStack, getfutureDist):
        Thread.__init__(self)
        self.sendMegaDataStack = sendMegaDataStack
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
        LaunchTime = None
        oldstereo_Distance = None

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

        try:
            getMegaData = GetMegaData(MEGA)
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
            try:
                if drillType == 'Dynamic':


                    # ___________________ RECEIVE STEREO DISTANCE (Wait)______________________________________________#
                    try:
                        stereoData = self.getStereoStack.peek()
                        stereo_Distance = float(stereoData.distance)
                        oldstereo_Distance = stereo_Distance
                    except:
                        print("[Launcher(Thread)] : No StereoData")
                        pass

                    while oldstereo_Distance is not None or stereo_Distance == oldstereo_Distance:
                        while stereo_Distance == oldstereo_Distance:
                            stereoData = stereoStack.peek()
                            stereo_Distance = float(stereoData.distance)

                    distanceTotal = stereo_Distance
                    rationaleDistMeasures = 1

                    # Limit the speed at which the launching sequence runs:
                    if LaunchTime is None:
                        pass
                    else:
                        while time.time() - LaunchTime < 5:
                        print("[Launcher(Thread)] : Waiting for results from target:  " + targetChoice + "'s  WIFI")
                        time.sleep(0.5)

                    TimeSinceLastLaunch = time.time()

                    # _______ RECEIVE MEGA DATA STRING (Wait): lidar_2_Distance, voiceCommand, targetTiming, targetBallSpeed _______#
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
                                continue # < Try again

                    lidar_2_Distance = int(tempData.strip("<").strip().split(",")[0])  # lidarDistance = int(cm)
                    # temperature = int(tempData.strip().split(",")[1])                            # temperature = int()
                    voiceCommand = int(tempData.strip().split(",")[2])  # voice commands = int(from 1 to 5)
                    targetTiming = float(tempData.strip().split(",")[3])  # targetTiming = float(0.0)
                    targetBallSpeed = float(tempData.strip().split(",").strip(">")[4])  # targetBallSpeed

                    # ___________________END OF RECEIVE MEGA DATA STRING ______________________________________________#
                    if lidar_2_Distance == 0:
                        lidar_2_Distance = None

                    # ___________________ SEND MEGA DATA TO megaDataStack ______________________________________________#
                    MegaData = MegaDataClass()
                    MegaData.voiceCommand = voiceCommand
                    if targetTiming != 0.0:
                        MegaData.targetTiming = targetTiming
                        print(targetTiming)
                    else:
                        MegaData.targetTiming = None
                    if targetBallSpeed != 0.0:
                        MegaData.targetBallSpeed = targetBallSpeed
                        print(targetBallSpeed)
                    else:
                        MegaData.targetBallSpeed = None

                    self.sendMegaDataStack.push(MegaData)

                    # ___________________       Handle Voice input:     ______________________________________________ #
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
                        lidar_1_Distance = self.getLidar1Stack.peek()
                        if lidar_1_Distance is not None and abs(stereo_Distance - lidar_1_Distance) <= 5:  # <<<<<<USE WEIGHTING FACTOR INSTEAD
                            rationaleDistMeasures += 1
                            lidar_1_Distance = lidar_1_Distance / 1000 # <<<<<< CONVERT to meters from mm
                            distanceTotal += lidar_1_Distance

                    except Exception as w:
                        print("[Launcher(Thread)] : LIDAR_1 -> nothing in Lidar1Stack" + w)
                        continue

                    # CALCULATE AND SEND TOTAL TO MAIN THREAD
                    FINAL_DIST = distanceTotal / rationaleDistMeasures
                    self.sendfinalDistStack.push(FINAL_DIST)

                    # ____QUERY LOOKUP TABLE FOR MOTOR_SPEED VALUES BASED ON FINAL_DIST___ #

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

                    # ____________________ Write data to MEGA ____________________
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

                    # _____ GET FUT_FINAL_DIST (No Wait)
                    OLD_FUT_FINAL_DIST = FUT_FINAL_DIST
                    try:
                        FUT_FINAL_DIST = self.getfutureDist.peek()      # <<<<< GET PREDICTED LOCATION
                    except:
                        print("[Launcher(Thread)] : No FUT_FINAL_DIST data in futureDistStack")
                        pass

                    if FUT_FINAL_DIST == OLD_FUT_FINAL_DIST:
                        ballFeed = 1
                        data = '<' + MotorSpeed1 + ',' + MotorSpeed2 + ',' + targetChoice + ',' + str(difficulty) + '>'  # + ',' + str(ballFeed)
                        # write data to MEGA
                        if 5 <= (time.time() - TimeSinceLastLaunch) <= 8:       # << Not elegant but restricts continuous launch signals
                            try:
                                MEGA.write(data.encode())
                                drillCount += 1  # << ON SUCCESSFUL LAUNCH
                                LaunchTime = time.time()
                                print("[Launcher(Thread)] : Launch signal sent to MEGA")
                            except Exception as e:
                                sendsuccess = False
                                while not sendsuccess:
                                    try:
                                        MEGA.write(data.encode())
                                        drillCount += 1  # << ON SUCCESSFUL LAUNCH
                                        LaunchTime = time.time()
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
                        # ____QUERY LOOKUP TABLE FOR NEW MOTOR_SPEED VALUES based on FUT_FINAL_DIST ___ #

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
                                LaunchTime = time.time()
                                print("[Launcher(Thread)] : Launch signal sent to MEGA")
                            except Exception as e:
                                sendsuccess = False
                                while not sendsuccess:
                                    try:
                                        MEGA.write(data.encode())
                                        drillCount += 1  # << ON SUCCESSFUL LAUNCH
                                        LaunchTime = time.time()
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
            # SENDS DATA TO Class, which can be "Put" using Stacks's______________________________
            results = StereoOutput
            results.distance = distance
            results.disparity = disparity
            results.masterval = masterval
            results.slaveval = slaveval
            stereoStack.push(results)

def GetUnoData(UNO):
    getdata = False
    dataPresent = False
    while getdata == False:
        print('[PitchYaw(Thread)] : in Mega.Serial while 1')
        if UNO.is_open:
            print('[PitchYaw(Thread)] : in the Mega.is_open')
            while not dataPresent:
                print('[PitchYaw(Thread)] : in Mega.Serial while 2')
                try:
                    print('[PitchYaw(Thread)] : trying to read startMarker')
                    startMarker = UNO.read().decode()
                except Exception as e:
                    print('[PitchYaw(Thread)] : didnt get Mega data' + e)
                if startMarker == "<":
                    unoDataStr = UNO.read(15)                                      # READ DATA FORMATTED AS ('< 1 >')
                    unoDataTemp = list(unoDataStr.decode())
                    unoDataTemp.insert(0, startMarker)
                    unoData = unoDataTemp[:unoDataTemp.index(">") + 1]
                    print("[PitchYaw(Thread)] : acquired MegaData  ->  ", unoData)
                    tempData = "".join(unoData)
                    print("[PitchYaw(Thread)] : tempData string  ->  ", tempData)
                    dataPresent = True
                    print('[PitchYaw(Thread)] : data read from Mega')
                    getdata = True
                    return tempData
        else:
            continue
            time.sleep(0.4)
        time.sleep(0.2)  # << MAY NEED TO BE CHANGED IF READ DATA IS GARBAGE

def GetMegaData(MEGA):
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
                    megaDataStr = MEGA.read(25)                                     # READ DATA FORMATTED AS ('< 1 2 3 4 5 >')
                    megaDataTemp = list(megaDataStr.decode())
                    megaDataTemp.insert(0, startMarker)
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
                    time.sleep(0.1)
                # print(megaDataStr)
                time.sleep(0.4) # < MAYBE IMPORTANT

def Lidar1Dist(evo):
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
        time.sleep(5)
    # ___ Start Threads ___ #

    # ___Begin Stereoscopic Thread:____ #
    try:
        process = Thread(target=StereoscopicsThread, args=[stereoStack])
        process.start()
    except Exception as e:
        print('[MainThread] : Stereo thread failed because of exception ' + e)

    time.sleep(6)
    try:
        pitchYawthread = PitchYaw(stereoStack, guiStack, temperatureStack, megaDataStack, finalDistStack, futureDistStack)
        pitchYawthread.start()
    except Exception as e:
        print('[MainThread] : Pitch and Yaw thread didnt start because of exception ' + e)
    try:
        startLauncherThread = Launcher(megaDataStack, lidar2Stack, guiStack, stereoStack, lidar1Stack, finalDistStack, temperatureStack, futureDistStack)
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
                    LIDAR_1_Distance = Lidar1Dist(evo)
                    if LIDAR_1_Distance is not None and abs(stereo_Distance - LIDAR_1_Distance) <= 5:  # <<<<<<USE WEIGHTING FACTOR INSTEAD
                        rationaleDistMeasures += 1
                        distanceTotal += LIDAR_1_Distance
                        lidar1Stack.push(LIDAR_1_Distance)
                    else:
                        lidar1Stack.push(None)  # << None value indicates no GOOD new data

                except Exception as w:
                    print("[MainThread] : LIDAR_1 -> no data" + w)
                    continue

                # Get(NO_WAIT)for Lidar_2_Dist (Run on New Data EVENT() trigger?)  _____________________________
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
                print("Player is not in Range")
                # Activate GPIO pin for notification LED
                # ***Do something about this***
                time.sleep(0.5)

        elif drillType == "Static":
            print("[MainThread] : Main doesnt do much here")

        elif drillType == "Manual":
            print("[MainThread] : Main doesnt do much here")

        else:
            print("[MainThread] : no GUI data")


# ________ Run The Program ____________ #
# startMainFile(3,4,"manual")  # for testing purposes only

