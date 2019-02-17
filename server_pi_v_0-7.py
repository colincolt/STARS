# _________________________________READ ME______________________________________#
# THREADING:
# This file uses the python "threading" module to run any number of "Threads"
# simultaneously, anything that needs to run continuously can be put into an
# infinite loop, anything that needs to run independantly and not interfere
# with other tasks. THREADS are ideal for input output operations, we do not
# have a lot of math being done.
#
# STACKS:
# <     https://interactivepython.org/runestone/static/pythonds/BasicDS/ImplementingaStackinPython.html       >
# <     https://www.pythoncentral.io/stack-tutorial-python-implementation/     >

# Packages
from collections import deque
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
import random
import PID                          # Custom Module


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
        else:
            return None

    def size(self):
        return len(self.items)


# GLOBAL STACKS:
stereoStack = Stack()
lidar2Stack = Stack()
guiStack = Stack()
megaDataStack = Stack()
finalDistStack = Stack()
lidar1Stack = Stack()
temperatureStack = Stack()
futureDistStack = Stack()

# GLOBAL FLAGS/EVENTS:
shutdown_flag = Event()
py_shutdown_flag = Event()
py_lock = Lock()

# GLOBAL VARIABLES:


# PITCH_YAW_THREAD: ---> Arduino UNO provide Angle values to both motors, request temperature's from the Uno, and distance measurements
    # RECEIVE from UNO:
    # - launchAngle (from accelerometer)
    # SEND to UNO:
    # - pitchAngle <-- to linear actuator
    # - motorSpeed <-- to the yaw motor

# Stack IO
# RECEIVE from STACKS:
    # - drillType from guiStack
    # - temperature from temperatureStack
    # - stereoDist from stereoStack
    # - FINAL_DIST from finalDistStack
    # - FUT_FINAL_DIST from futureDistStack

# SEND to STACKS:
    # - launcherAngle -->  launcherAngleStack --> Launcher(Thread)

# _______PITCH AND YAW THREAD________ #

class PitchYaw(Thread):
    def __init__(self, getstereoStack, getguiStack, getTemperatureStack, getmegaDataStack, getfinalDistStack,
                 getfutureDistStack):
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
        dist_deque = deque([])
        x_disp_deque = deque([])
        measure_time_deque = deque([])

        # ** __ DATA TRACKERS: __ ** #
        startData = False
        last_start_time = None

        # ** __ IMPORTANT VARIABLES: __ ** #
        avg_measure = 10
        LEAD_SPEED = 25  # << Adjust the added motorSpeed of the YAW for DYNAMIC MODE (0-255)
        MAX_YAW_SPEED = 0.15  # << Maximum speed of Yaw in radians
        pitchAngleTable = np.array([[5, 27],  # << Pitch angle lookup table based on estimations
                                    [7, 28],
                                    [9, 29],
                                    [11, 30],
                                    [13, 31],
                                    [15, 32],
                                    [17, 33],
                                    [19, 34],
                                    [21, 35],
                                    [23, 36],
                                    [25, 37]])

        while not startData:
            # _ARDUINO_UNO__SERIAL_OPEN
            try:
                # Initialize Arduino UNO
                UNO = serial.Serial("/dev/uno", 9600)  # change ACM number as found from "ls /dev/tty*"
                UNO.baudrate = 9600
                # ACQUIRE ACCELEROMETER DATA (Single Pitch reading) _______ #
                tempAngle = GetUnoData(UNO)
                launcherAngle = round(int(tempAngle.strip("<").strip().strip(">")[0]))  # lidarDistance = int(cm)
                # Get Temperature Data
                temperature = self.getTemperatureStack.peek()
                tempCorrection = temperature / 25  # <<<tempCorrection factor

                # Get GUI Data
                guiData = self.getguiStack.peek()
                # drillSpeed = guiData.speed
                # difficulty = guiData.difficulty
                drillType = guiData.drilltype
            except Exception as err:
                print('[PithYaw(Thread)] : failed to get startup data... trying again' + str(err))
                continue
            else:
                print("[PitchYaw(Thread)] : Received start data")
                startData = True
                continue

        # print("[PitchYaw(Thread)] : Speed:  " + str(speed) + "  Diff:  " + str(difficulty) + "  Type:  " + str(drillType))
        print("[PitchYaw(Thread)] : starting Loop")

        while not self.py_shutdown_flag.isSet():
            # <<< BEGINNING OF PITCHYAW LOOP _________________________________ ** #
            MEGAdata = self.getmegaDataStack.peek()
            voiceCommand = MEGAdata.voicecommand

            while voiceCommand != "beginVC":
                try:
                    MEGAdata = self.getmegaDataStack.peek()
                    voiceCommand = MEGAdata.voicecommand  # voice commands = int(from 1 to 5)
                except:
                    time.sleep(0.5)

            start_time = time.time()

            if last_start_time is None:
                last_start_time = start_time
                first_measure = True
            else:
                temp_time = last_start_time - start_time
                measure_time_deque.appendleft(temp_time)
                if len(measure_time_deque) > avg_measure:
                    measure_time_deque.pop()
                first_measure = False
            # ________________________________________________________________________ ##

            if drillType == "Dynamic":
                try:
                    cameradata = False
                    while not cameradata:
                        try:
                            stereodata = self.getstereoStack.peek()
                            cameradata = True
                        except:  # EmptyStackError # <- return EmptyStackError if Stack is empty in peek #####!!!!!!!
                            print("[PithYaw(Thread)] : no data in stereoStack")
                            self.py_shutdown_flag.set()

                    LeftXcoord = stereodata.masterval
                    RightXcoord = stereodata.slaveval
                    stereoDist = int(stereodata.distance)


                    # Try for FUT_FINAL_DIST _______________
                    try:
                        FUT_FINAL_DIST = self.getfutureDistStack.peek()
                        if FUT_FINAL_DIST is not None:
                            futureDist = True
                        else:
                            futureDist = False
                            # Try for FINAL_DIST from Launcher(Thread) _______________
                            try:
                                FINAL_DIST = self.getfinalDistStack.peek()
                                if FINAL_DIST is not None:
                                    finalDist = True
                                else:
                                    finalDist = False
                            except:
                                finalDist = False
                                pass
                    except:
                        futureDist = False
                        # Try for FINAL_DIST from Launcher(Thread) _______________
                        try:
                            FINAL_DIST = self.getfinalDistStack.peek()
                            if FINAL_DIST is not None:
                                finalDist = True
                            else:
                                finalDist = False
                        except:
                            finalDist = False
                            pass

                    # **________ TWO POSSIBLE CASES: FUTURE_DIST IS AVAILABLE OR NOT ________** #

                    if futureDist:
                        # << CASE 1: Only have to 'predict'/anticipate future lateral displacement _________
                        usedDistance = FUT_FINAL_DIST

                        if not first_measure:       #
                            dist_deque.appendleft(usedDistance)

                        if len(dist_deque) > avg_measure:
                            dist_deque.pop()

                        if usedDistance > 25.0: usedDistance = 25.0

                        # ** _________________ PITCH ANGLE: __________________ ** #
                        # Query table for angle at usedDistance
                        row = 4 - (int(round(usedDistance / 2)) * 2) - 1
                        if row < 0: row = 0
                        elif row > 11: row = 11
                        pitchAngle = pitchAngleTable[row, 1] + launcherAngle  # << ANGLE IS ALREADY SET BASED ON FUTURE DISTANCE (via row)

                        # ** ___________________ YAW MOTOR SPEED: ______________________ ** #
                        latPixelDisp = (2464 - LeftXcoord - RightXcoord)
                        if (2464 - LeftXcoord < RightXcoord):       # <<< CHOOSE MOTOR DIRECTION
                            latPixelDisp = -latPixelDisp

                        PixToDistApprox = usedDistance / 250  # <<< nonsense Conversion at the moment
                        x_displacement = latPixelDisp * PixToDistApprox
                        x_disp_deque.appendleft(x_displacement)
                        if len(x_disp_deque) > avg_measure:
                            x_disp_deque.pop()

                        # IF MULTIPLE RECORDED MEASUREMENTS, BASE MOTORSPEED ESTIMATION OFF PLAYERS SPEED ESTIMATION (V = omega*R):
                        if len(measure_time_deque) == avg_measure:
                            temp_time = sum([elem for elem in measure_time_deque])  # Time for avg_measure measurements
                            latSpeed = (x_disp_deque[0] - x_disp_deque[avg_measure - 1]) / temp_time
                            angularSpeed = (abs(latSpeed) / usedDistance) * 3.14  # << Convert to radians
                            # CURRENT MAXIMUM ANGULAR SPEED: Omega = 6 deg/sec OR 0.15
                            motorSpeed = (angularSpeed / MAX_YAW_SPEED) * 255
                            # For Dynamic, we want to be leading the players movement:
                            motorSpeed = motorSpeed + LEAD_SPEED
                            if motorSpeed >= 255: motorSpeed = 255
                            if motorSpeed <= 80: motorSpeed = 80
                            if (2464 - LeftXcoord < RightXcoord):
                                motorSpeed = -motorSpeed

                            # ** ___ SEND DATA ___ ** #
                            data = '<' + str(motorSpeed) + ', ' + str(pitchAngle) + '>'
                            UNO.write(data.encode())
                            print("[PithYaw(Thread)] SENT: <motorSpeed, pitchAngle> =  " + data)
                            time.sleep(0.1)

                        # ** ____________________________________________________________________ ** #

                        # IF THIS IS THE FIRST FEW MEASUREMENT JUST USE 150 for YAW SPEED
                        else:
                            motorSpeed = 150
                            if (abs(latPixelDisp) < 150):
                                data = '<0, ' + str(pitchAngle) + '>'
                                UNO.write(data.encode())
                                # sys.stdout.flush()
                                # print("locked on to target")
                                time.sleep(0.1)

                            elif (2464 - LeftXcoord < RightXcoord):
                                data distArray= '<' + str(-motorSpeed) + ', ' + str(pitchAngle) + '>'
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

                        # If it somehow fails peeking the FUT_FINAL_DIST one time
                        if len(dist_deque) == avg_measure:
                            tempDist = dist_deque[0] - dist_deque[avg_measure-1]                # Distance covered in avg_measure measurements
                            temp_time = sum([elem for elem in measure_time_deque])  # Time for avg_measure measurements
                            speed = tempDist / temp_time # meters/second
                            # the idea is to stay ahead of the player by at least a second or two
                            FUT_FINAL_DIST = usedDistance + speed * 3
                            usedDistance = FUT_FINAL_DIST


                        # ** ________________________PITCH ANGLE: ______________________ ** #
                        # Query table for angle at usedDistance
                        row = 4 - (int(round(usedDistance / 2)) * 2) - 1  # << ENSURE THIS IS A MULTIPLE OF 2 BETWEEN 6-34
                        if row < 0: row = 0
                        elif row > 11: row = 11
                        pitchAngle = pitchAngleTable[row, 1]  # << ANGLE IS ALREADY SET BASED ON FUTURE DISTANCE

                        # ** ___________________ YAW MOTOR SPEED: ______________________ ** #
                        latPixelDisp = (2464 - LeftXcoord - RightXcoord)
                        if (2464 - LeftXcoord < RightXcoord):  # <<< CHOOSE MOTOR DIRECTION
                            latPixelDisp = -latPixelDisp

                        PixToDistApprox = usedDistance / 250  # <<< nonsense Conversion at the moment
                        x_displacement = latPixelDisp * PixToDistApprox
                        x_disp_deque.appendleft(x_displacement)
                        if len(x_disp_deque) > avg_measure:
                            x_disp_deque.pop()

                        # IF MULTIPLE RECORDED MEASUREMENTS, BASE MOTORSPEED ESTIMATION OFF PLAYERS SPEED ESTIMATION (V = omega*R):
                        if len(measure_time_deque) == avg_measure:
                            temp_time = sum([elem for elem in measure_time_deque])  # Time for avg_measure measurements
                            latSpeed = (x_disp_deque[0] - x_disp_deque[avg_measure - 1]) / temp_time
                            angularSpeed = (abs(latSpeed) / usedDistance) * 3.14  # << Convert to radians
                            # CURRENT MAXIMUM ANGULAR SPEED: Omega = 6 deg/sec OR 0.15
                            motorSpeed = (angularSpeed / MAX_YAW_SPEED) * 255
                            # For Dynamic, we want to be leading the players movement:
                            motorSpeed = motorSpeed + LEAD_SPEED
                            if motorSpeed >= 255: motorSpeed = 255
                            if motorSpeed <= 80: motorSpeed = 80
                            if (2464 - LeftXcoord < RightXcoord):
                                motorSpeed = -motorSpeed

                            # ** ___ SEND DATA ___ ** #
                            data = '<' + str(motorSpeed) + ', ' + str(pitchAngle) + '>'
                            UNO.write(data.encode())
                            print("[PithYaw(Thread)] SENT: <motorSpeed, pitchAngle> =  " + data)
                            time.sleep(0.1)
                        # ** ____________________________________________________________________ ** #

                        # IF THIS IS THE FIRST MEASUREMENT JUST USE 150 for YAW SPEED
                        else:
                            motorSpeed = 150
                            if (latPixelDisp < 150):
                                data = '<0, ' + str(pitchAngle) + '>'
                                UNO.write(data.encode())
                                # sys.stdout.flush()
                                # print("locked on to target")
                                time.sleep(0.1)

                            elif (2464 - LeftXcoord < RightXcoord):
                                data = '<' + str(-motorSpeed) + ', ' + str(pitchAngle) + '>'
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

                    dist_deque.appendleft(usedDistance)
                    if len(dist_deque) > avg_measure:
                        dist_deque.pop()

                    # ** _________________ PITCH ANGLE: __________________ ** #
                    # Query table for angle at usedDistance
                    row = 4 - (int(round(usedDistance / 2)) * 2) - 1
                    if row < 0: row = 0
                    elif row > 11: row = 11
                    pitchAngle = pitchAngleTable[row, 1] + launcherAngle  # << ANGLE IS ALREADY SET BASED ON FUTURE DISTANCE

                    # ** ___________________ YAW MOTOR SPEED: ______________________ ** #
                    latPixelDisp = (2464 - LeftXcoord - RightXcoord)
                    if (2464 - LeftXcoord < RightXcoord):  # <<< CHOOSE MOTOR DIRECTION
                        latPixelDisp = -latPixelDisp

                    PixToDistApprox = usedDistance / 250  # <<< nonsense Conversion at the moment
                    x_displacement = latPixelDisp * PixToDistApprox
                    x_disp_deque.appendleft(x_displacement)
                    if len(x_disp_deque) > avg_measure:
                        x_disp_deque.pop()

                    # IF MULTIPLE RECORDED MEASUREMENTS, BASE MOTORSPEED ESTIMATION OFF PLAYERS SPEED ESTIMATION (V = omega*R):
                    if len(measure_time_deque) == avg_measure:
                        temp_time = sum([elem for elem in measure_time_deque])  # Time for avg_measure measurements
                        latSpeed = (x_disp_deque[0] - x_disp_deque[avg_measure - 1]) / temp_time
                        angularSpeed = (abs(latSpeed) / usedDistance) * 3.14  # << Convert to radians
                        # CURRENT MAX_YAW_SPEED: w = 0.15 rad/sec
                        motorSpeed = (angularSpeed / MAX_YAW_SPEED) * 255
                        # For Dynamic, we want to be leading the players movement:
                        motorSpeed = motorSpeed + LEAD_SPEED
                        if motorSpeed >= 255: motorSpeed = 255
                        if motorSpeed <= 80: motorSpeed = 80
                        if (2464 - LeftXcoord < RightXcoord):
                            motorSpeed = -motorSpeed

                        # ** ___ SEND DATA ___ ** #
                        data = '<' + str(motorSpeed) + ', ' + str(pitchAngle) + '>'
                        UNO.write(data.encode())
                        print("[PithYaw(Thread)] SENT: <motorSpeed, pitchAngle> =  " + data)
                        time.sleep(0.1)
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
                            data = '<' + str(-motorSpeed) + ', ' + str(pitchAngle) + '>'
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


# _____________LAUNCHER THREAD_____________________#
# Connects and communicates with the Arduino Mega: Launcher Motors, Ball Feeder, Wifi, Accelerometer? (Rangefinder?)
# Launcher(Thread): ---> Arduino MEGA provide Angle values to both motors, request temperature's from the Uno, and distance measurements
# RECEIVE from MEGA:
#
# SEND to MEGA:
#
#

# Stack IO
# RECEIVE from STACKS:
#
#
#
#

# SEND to STACKS:
#

class Launcher(Thread):
    def __init__(self, sendMegaDataStack, sendLidar2Stack, guiStack, getStereoStack, getLidar1Stack, sendfinalDistStack,
                 sendTemperatureStack, getfutureDist):
        Thread.__init__(self)
        self.sendMegaDataStack = sendMegaDataStack
        self.sendLidar2Stack = sendLidar2Stack
        self.guiStack = guiStack
        self.getStereoStack = getStereoStack
        self.getLidar1Stack = getLidar1Stack
        self.sendfinalDistStack = sendfinalDistStack
        self.sendTemperatureStack = sendTemperatureStack
        self.getfutureDist = getfutureDist
        self.shutdown_flag = Event()  # <<< SHUTDOWN FLAG

    def run(self):
        # _____ Launcher common variables:
        beginVC = 1
        stopVC = 2
        fasterVC = 3
        slowerVC = 4
        pauseVC = 5
        drillCount = 0
        OLD_FUT_FINAL_DIST = None
        LaunchTime = None
        startData = False
        MegaData = MegaDataClass()

        # LAUNCHER OPTIONS:
        DYNAMIC_WAIT_TIME = 3
        STATIC_WAIT_TIME = 5

        while not startData:  # <- Add a timeout to the the start loop
            try:
                # _____ open MEGA serial port
                MEGA = serial.Serial('/dev/mega', 9600)
                MEGA.timeout = None
                baudrate = 9600,
                parity = serial.PARITY_NONE,
                stopbits = serial.STOPBITS_ONE,
                bytesize = serial.EIGHTBITS,
                print("[Launcher(Thread)] : Connected to MEGA")

                getMegaData = GetMegaData(MEGA)
                # Get TEMPERATURE and voiceCommand --> send to PITCH_YAW Thread
                temperature = int(getMegaData.strip().split(",")[1])  # temperature = int()
                voiceCommand = int(getMegaData.strip().split(",")[2])  # voice commands = int(from 1 to 5)
                MegaData.temperature = temperature
                MegaData.voiceCommand = voiceCommand
                self.sendTemperatureStack.push(temperature)
                self.sendMegaDataStack.push(MegaData)

                # ****SET TEMPERATURE CORRECTION FACTOR*****
                tempCorrect = temperature / 25
                # *****************************************

                # Get drillType data from GUI__ #
                guiData = self.guiStack.peek()
                drillSpeed = guiData.speed
                difficulty = guiData.difficulty
                drillType = guiData.drilltype
                # print("LAUNCHER: Speed:  " + str(drillSpeed) + "  Diff:  " + str(difficulty) + "  Type:  " + str(drillType))

                while voiceCommand != beginVC:
                    getMegaData = GetMegaData()
                    voiceCommand = int(getMegaData.strip().split(",")[2])  # voice commands = int(from 1 to 5)
                    time.sleep(0.5)

            except serial.SerialException as err:
                print("[Launcher(Thread)] : MEGA not detected" + err)
                continue
            except Exception as e:  # (serial.SerialException)
                print("[Launcher(Thread)] : Error in setup" + str(e))
                self.shutdown_flag.set()
                continue
            else:
                print("[Launcher(Thread)] : Setup Complete")
                startData = True

        print("[Launcher(Thread)] : Starting LOOP")
        voiceCommand = 0

        while drillCount <= 5 and not self.shutdown_flag.isSet():
            try:
                if drillType == 'Dynamic':
                    # Limit the speed at which the launching sequence runs:
                    if LaunchTime is None:
                        pass
                    else:
                        while (time.time() - LaunchTime) < DYNAMIC_WAIT_TIME:
                            print("[Launcher(Thread)] : Waiting for results from target WIFI")

                    startTime = time.time()

                    # ___________________ RECEIVE STEREO DISTANCE (Wait)______________________________________________#
                    try:
                        stereoData = self.getStereoStack.peek()
                        stereo_Distance = float(stereoData.distance)
                        # print("[Launcher(Thread)] : stereo_Distance =  " + str(stereo_Distance))
                        # oldstereo_Distance = stereo_Distance # <-- Check for new data, needs edits
                    except ValueError as verr:
                        print("[Launcher(Thread)] : StereoDistance couldnt be converted to float" + str(err))
                        if stereoData is None:
                            print("[Launcher(Thread)] : ... because Stack is Empty")
                            continue
                        else:
                            self.getStereoStack.pop()
                            print("[Launcher(Thread)] : issue with stereodata, removing it and trying again")
                        continue
                    except:
                        print("[Launcher(Thread)] : Error getting 'getStereoStack'")
                        self.shutdown_flag.set()
                        continue
                    else:
                        distanceTotal = stereo_Distance
                        rationaleDistMeasures = 1

                    # _______ RECEIVE MEGA DATA STRING (Wait): lidar_2_Distance, voiceCommand, targetTiming, targetBallSpeed _______#
                    try:
                        tempData = GetMegaData
                    except serial.SerialException as err:
                        print("[Launcher(Thread)] : MEGA not detected" + err)
                    except Exception as e:  # (serial.SerialException)
                        print("[Launcher(Thread)] : Error in setup" + str(e))
                        success = False
                        while not success:
                            try:
                                tempData = GetMegaData
                                success = True
                            except Exception as r:
                                print("[Launcher(Thread)] : GetMegaData failed" + r)
                                continue  # < Try again

                    lidar_2_Distance = int(tempData.strip("<").strip().split(",")[0])  # lidarDistance = int(cm)
                    # temperature = int(tempData.strip().split(",")[1])                            # temperature = int()
                    voiceCommand = int(tempData.strip().split(",")[2])  # voice commands = int(from 1 to 5)
                    targetTiming = float(tempData.strip().split(",")[3])  # targetTiming = float(0.0)
                    targetBallSpeed = float(tempData.strip().split(",").strip(">")[4])  # targetBallSpeed

                    # ___________________END OF RECEIVE MEGA DATA STRING ______________________________________________#

                    # ___________________ SEND MEGA DATA TO megaDataStack ______________________________________________#
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
                        continue  # <-- Starts next iteration of parent loop where the shutdown flag is caught
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
                        self.sendLidar2Stack.push(None)  # <<send empty value so we know theres no new data

                    try:
                        lidar_1_Distance = self.getLidar1Stack.peek()
                        if lidar_1_Distance is not None and abs(
                                stereo_Distance - lidar_1_Distance) <= 5:  # <<<<<<USE WEIGHTING FACTOR INSTEAD
                            rationaleDistMeasures += 1
                            lidar_1_Distance = lidar_1_Distance / 1000  # <<<<<< CONVERT to meters from mm
                            distanceTotal += lidar_1_Distance

                    except Exception as stackemp:
                        print("[Launcher(Thread)] : LIDAR_1 -> nothing in Lidar1Stack" + str(stackemp))
                        pass

                    # CALCULATE AND SEND TOTAL TO MAIN THREAD
                    FINAL_DIST = distanceTotal / rationaleDistMeasures
                    self.sendfinalDistStack.push(FINAL_DIST)

                    # In DYNAMIC Mode, the motors spin up before receiving final instructions from MAIN THREAD
                    # ____POLYNOMIAL FIT FROM THEORETICAL VALUES___ #
                    RPM = -1.13635244 * FINAL_DIST ^ 2 + 97.7378699 * FINAL_DIST + 646.034298  # <-- Polynomial fit
                    motorSpeed1, motorSpeed2 = round(
                        (RPM / 5000) * 255)  # * tempCorrect?  # Value between 0-255 (On 24 V: 0-5000 RPM)

                    # ____SEND MEGA DATA STRING _____#    IF NO VALUE SEND AS '-1'
                    MotorSpeed1 = str(motorSpeed1)
                    MotorSpeed2 = str(motorSpeed2)
                    # ***Randomize targetChoice
                    targetChoice = int(random.choice([1, 2, 3, 4]))
                    # **** NEED TO ADD A FIFTH VALUE FOR FEED/NO-FEED OF BALL****
                    ballFeed = 0
                    data = '<' + MotorSpeed1 + ',' + MotorSpeed2 + ',' + str(targetChoice) + ',' + str(
                        difficulty) + ',' + str(ballFeed) + '>'

                    # ____________________ Write data to MEGA ____________________
                    try:
                        MEGA.write(data.encode())
                    except Exception as e:
                        sendsuccess = False
                        while not sendsuccess:
                            try:
                                MEGA.write(data.encode())
                                print("[Launcher(Thread)] : Launch motors starting...")
                                sendsuccess = True
                            except:
                                if time.time() - startTime >= 1:
                                    self.shutdown_flag.set()

                    # _____ GET FUT_FINAL_DIST (No Wait)

                    if OLD_FUT_FINAL_DIST is not None:
                        OLD_FUT_FINAL_DIST = FUT_FINAL_DIST

                    try:
                        FUT_FINAL_DIST = self.getfutureDist.peek()  # <<<<< GET PREDICTED LOCATION
                    except:
                        print("[Launcher(Thread)] : No FUT_FINAL_DIST data in futureDistStack")
                        FUT_FINAL_DIST = None
                        pass

                    if FUT_FINAL_DIST is None or FUT_FINAL_DIST == OLD_FUT_FINAL_DIST:  # <- no prediction is doen  in this thread so it will send AS IS
                        ballFeed = 1
                        data = '<' + MotorSpeed1 + ',' + MotorSpeed2 + ',' + str(targetChoice) + ',' + str(
                            difficulty) + ',' + str(ballFeed) + '>'
                        # write data to MEGA
                        if (time.time() - startTime) <= 1:
                            try:
                                MEGA.write(data.encode())
                                drillCount += 1  # << ON SUCCESSFUL LAUNCH
                                LaunchTime = time.time()
                                print("[Launcher(Thread)] : Launch signal sent to MEGA")
                            except:
                                sendsuccess = False
                                while not sendsuccess:
                                    try:
                                        MEGA.write(data.encode())
                                        drillCount += 1  # << ON SUCCESSFUL LAUNCH
                                        LaunchTime = time.time()
                                        print("[Launcher(Thread)] : Launch signal (second attempt) sent to MEGA")
                                        sendsuccess = True
                                    except:
                                        if time.time() - startTime >= 1:
                                            print("[Launcher(Thread)] : sending to MEGA timed out")
                                            self.shutdown_flag.set()
                        else:
                            print("[Launcher(Thread)] : sending to MEGA timed out")
                            self.shutdown_flag.set()

                    else:
                        time.sleep(0.1)  # <-- ALLOW TIME FOR PREVIOUS DATA TO BE READ AND CLEARED FROM THE BUFFER

                        # ____POLYNOMIAL FIT FROM THEORETICAL VALUES___ #
                        RPM = -1.13635244 * FUT_FINAL_DIST ^ 2 + 97.7378699 * FUT_FINAL_DIST + 646.034298  # <-- Polynomial fit
                        motorSpeed1, motorSpeed2 = round(
                            (RPM / 5000) * 255)  # * tempCorrect? # Value between 0-255 (On 24 V: 0-5000 RPM)

                        # ____SEND MEGA DATA STRING _____#    IF NO VALUE SEND AS '-1'
                        MotorSpeed1 = str(motorSpeed1)
                        MotorSpeed2 = str(motorSpeed2)
                        # **** NEED TO ADD A FIFTH VALUE FOR FEED/NO-FEED OF BALL****
                        ballFeed = 1
                        data = '<' + MotorSpeed1 + ',' + MotorSpeed2 + ',' + str(targetChoice) + ',' + str(difficulty) + ',' + str(ballFeed) + '>'
                        # Write data to MEGA
                        if (time.time() - startTime) <= 1:
                            try:
                                MEGA.write(data.encode())
                                drillCount += 1  # << ON SUCCESSFUL LAUNCH
                                LaunchTime = time.time()
                                print("[Launcher(Thread)] : Launch signal sent to MEGA")
                            except:
                                sendsuccess = False
                                while not sendsuccess:
                                    try:
                                        MEGA.write(data.encode())
                                        drillCount += 1  # << ON SUCCESSFUL LAUNCH
                                        LaunchTime = time.time()
                                        print("[Launcher(Thread)] : Launch signal sent to MEGA")
                                        sendsuccess = True
                                    except:
                                        if time.time() - startTime >= 1:
                                            self.shutdown_flag.set()
                        else:
                            print("[Launcher(Thread)] : process took too long to keep up with drill")
                            self.shutdown_flag.set()

                    if OLD_FUT_FINAL_DIST is None:
                        OLD_FUT_FINAL_DIST = FUT_FINAL_DIST

                if drillType == 'Static':
                    # Limit the speed at which the launching sequence runs:
                    if LaunchTime is None:
                        pass
                    else:
                        while (time.time() - LaunchTime) < STATIC_WAIT_TIME:
                            print("[Launcher(Thread)] : Waiting for results from target: WIFI")

                    startTime = time.time()

                    # ___________________ RECEIVE STEREO DISTANCE (Wait)______________________________________________#
                    try:
                        stereoData = self.getStereoStack.peek()
                        stereo_Distance = float(stereoData.distance)
                        # print("[Launcher(Thread)] : stereo_Distance =  " + str(stereo_Distance))
                        # oldstereo_Distance = stereo_Distance # <-- Check for new data, needs edits
                    except ValueError as verr:
                        print("[Launcher(Thread)] : StereoDistance couldnt be converted to float" + str(verr))
                        if stereoData is None:
                            print("[Launcher(Thread)] : ... because Stack is Empty")
                            continue
                        else:
                            self.getStereoStack.pop()
                            print("[Launcher(Thread)] : issue with stereodata, removing it and trying again")
                        continue
                    except:
                        print("[Launcher(Thread)] : Error getting 'getStereoStack'")
                        self.shutdown_flag.set()
                        continue
                    else:
                        distanceTotal = stereo_Distance
                        rationaleDistMeasures = 1

                    # _______ RECEIVE MEGA DATA STRING (Wait): lidar_2_Distance, voiceCommand, targetTiming, targetBallSpeed _______#
                    try:
                        tempData = GetMegaData
                    except serial.SerialException as err:
                        print("[Launcher(Thread)] : MEGA not detected" + err)
                    except Exception as e:  # (serial.SerialException)
                        print("[Launcher(Thread)] : Error in setup" + e)
                        success = False
                        while not success:
                            try:
                                tempData = GetMegaData
                                success = True
                            except Exception as r:
                                print("[Launcher(Thread)] : GetMegaData failed" + r)
                                continue  # < Try again

                    lidar_2_Distance = int(tempData.strip("<").strip().split(",")[0])  # lidarDistance = int(cm)
                    # temperature = int(tempData.strip().split(",")[1])                            # temperature = int()
                    voiceCommand = int(tempData.strip().split(",")[2])  # voice commands = int(from 1 to 5)
                    targetTiming = float(tempData.strip().split(",")[3])  # targetTiming = float(0.0)
                    targetBallSpeed = float(tempData.strip().split(",").strip(">")[4])  # targetBallSpeed

                    # ___________________END OF RECEIVE MEGA DATA STRING ______________________________________________#

                    # ___________________ SEND MEGA DATA TO megaDataStack ______________________________________________#
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
                        continue  # <-- Starts next iteration of parent loop where the shutdown flag is caught
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
                        self.sendLidar2Stack.push(None)  # <<send empty value so we know theres no new data

                    try:
                        lidar_1_Distance = self.getLidar1Stack.peek()
                        if lidar_1_Distance is not None and abs(
                                stereo_Distance - lidar_1_Distance) <= 5:  # <<<<<<USE WEIGHTING FACTOR INSTEAD
                            rationaleDistMeasures += 1
                            lidar_1_Distance = lidar_1_Distance / 1000  # <<<<<< CONVERT to meters from mm
                            distanceTotal += lidar_1_Distance
                    except Exception as stackemp:
                        print("[Launcher(Thread)] : LIDAR_1 -> nothing in Lidar1Stack" + str(stackemp))
                        pass

                    # CALCULATE AND SEND TOTAL TO MAIN THREAD
                    FINAL_DIST = distanceTotal / rationaleDistMeasures
                    self.sendfinalDistStack.push(FINAL_DIST)

                    # In DYNAMIC Mode, the motors spin up before receiving final instructions from MAIN THREAD
                    # ____POLYNOMIAL FIT FROM THEORETICAL VALUES___ #
                    RPM = -1.13635244 * FINAL_DIST ^ 2 + 97.7378699 * FINAL_DIST + 646.034298  # <-- Polynomial fit
                    motorSpeed1, motorSpeed2 = round(
                        (RPM / 5000) * 255)  # * tempCorrect?  # Value between 0-255 (On 24 V: 0-5000 RPM)

                    # ____SEND MEGA DATA STRING _____#    IF NO VALUE SEND AS '-1'
                    MotorSpeed1 = str(motorSpeed1)
                    MotorSpeed2 = str(motorSpeed2)
                    # ***Randomize targetChoice
                    targetChoice = int(random.choice([1, 2, 3, 4]))
                    # **** NEED TO ADD A FIFTH VALUE FOR FEED/NO-FEED OF BALL****
                    ballFeed = 1
                    data = '<' + MotorSpeed1 + ',' + MotorSpeed2 + ',' + str(targetChoice) + ',' + str(difficulty) + ',' + str(ballFeed) + '>'

                    # ____________________ Write data to MEGA ____________________
                    try:
                        MEGA.write(data.encode())
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
                                if time.time() - startTime >= 1:
                                    self.shutdown_flag.set()

                if drillType == 'Manual':  # ________________________________________________________________________ #

                    while voiceCommand != beginVC and voiceCommand != stopVC:
                        # ___________________ RECEIVE STEREO DISTANCE (Wait)______________________________________________#
                        try:
                            stereoData = self.getStereoStack.peek()
                            stereo_Distance = float(stereoData.distance)
                            # print("[Launcher(Thread)] : stereo_Distance =  " + str(stereo_Distance))
                            # oldstereo_Distance = stereo_Distance # <-- Check for new data, needs edits
                        except ValueError as verr:
                            print("[Launcher(Thread)] : StereoDistance couldnt be converted to float" + str(verr))
                            if stereoData is None:
                                print("[Launcher(Thread)] : ... because Stack is Empty")
                                continue
                            else:
                                self.getStereoStack.pop()
                                print("[Launcher(Thread)] : issue with stereodata, removing it and trying again")
                            continue
                        except:
                            print("[Launcher(Thread)] : Error getting 'getStereoStack'")
                            self.shutdown_flag.set()
                            continue
                        else:
                            distanceTotal = stereo_Distance
                            rationaleDistMeasures = 1

                        # _______ RECEIVE MEGA DATA STRING (Wait): lidar_2_Distance, voiceCommand, targetTiming, targetBallSpeed _______#
                        try:
                            tempData = GetMegaData
                        except serial.SerialException as err:
                            print("[Launcher(Thread)] : MEGA not detected" + err)
                        except Exception as e:  # (serial.SerialException)
                            print("[Launcher(Thread)] : Error in setup" + e)
                            success = False
                            while not success:
                                try:
                                    tempData = GetMegaData
                                    success = True
                                except Exception as r:
                                    print("[Launcher(Thread)] : GetMegaData failed" + str(r))
                                    continue  # < Try again

                        lidar_2_Distance = int(tempData.strip("<").strip().split(",")[0])  # lidarDistance = int(cm)
                        # temperature = int(tempData.strip().split(",")[1])                            # temperature = int()
                        voiceCommand = int(tempData.strip().split(",")[2])  # voice commands = int(from 1 to 5)
                        targetTiming = float(tempData.strip().split(",")[3])  # targetTiming = float(0.0)
                        targetBallSpeed = float(tempData.strip().split(",").strip(">")[4])  # targetBallSpeed

                        # ___________________END OF RECEIVE MEGA DATA STRING ______________________________________________#

                        # ___________________ SEND MEGA DATA TO megaDataStack ______________________________________________#
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
                            continue  # <-- Starts next iteration of parent loop where the shutdown flag is caught
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
                        self.sendLidar2Stack.push(None)  # <<send empty value so we know theres no new data

                    try:
                        lidar_1_Distance = self.getLidar1Stack.peek()
                        if lidar_1_Distance is not None and abs(
                                stereo_Distance - lidar_1_Distance) <= 5:  # <<<<<<USE WEIGHTING FACTOR INSTEAD
                            rationaleDistMeasures += 1
                            lidar_1_Distance = lidar_1_Distance / 1000  # <<<<<< CONVERT to meters from mm
                            distanceTotal += lidar_1_Distance
                    except Exception as stackemp:
                        print("[Launcher(Thread)] : LIDAR_1 -> nothing in Lidar1Stack" + str(stackemp))
                        pass

                    # CALCULATE AND SEND TOTAL TO MAIN THREAD
                    FINAL_DIST = distanceTotal / rationaleDistMeasures
                    self.sendfinalDistStack.push(FINAL_DIST)

                    # In DYNAMIC Mode, the motors spin up before receiving final instructions from MAIN THREAD
                    # ____POLYNOMIAL FIT FROM THEORETICAL VALUES___ #
                    RPM = -1.13635244 * FINAL_DIST ^ 2 + 97.7378699 * FINAL_DIST + 646.034298  # <-- Polynomial fit
                    motorSpeed1, motorSpeed2 = round(
                        (RPM / 5000) * 255)  # * tempCorrect?  # Value between 0-255 (On 24 V: 0-5000 RPM)

                    # ____SEND MEGA DATA STRING _____#    IF NO VALUE SEND AS '-1'
                    MotorSpeed1 = str(motorSpeed1)
                    MotorSpeed2 = str(motorSpeed2)
                    # ***Randomize targetChoice
                    targetChoice = int(random.choice([1, 2, 3, 4]))
                    # **** NEED TO ADD A FIFTH VALUE FOR FEED/NO-FEED OF BALL****
                    ballFeed = 1
                    data = '<' + MotorSpeed1 + ',' + MotorSpeed2 + ',' + str(targetChoice) + ',' + str(difficulty) + ',' + str(ballFeed) + '>'

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
                                if time.time() - startTime >= 1:
                                    self.shutdown_flag.set()

            except Exception as e:
                print('[Launcher(Thread)] : failed because of exception: ' + e)
                self.shutdown_flag.set()
            # except:   # <-- put as many as needed, order is important
            # else:     # <-- Runs if no exception raised
            # finally:  # <-- Runs no matter what
        print("[Launcher(Thread)] : shutdown_flag detected!")

        # if self.shutdown_flag.isSet(): # <-- catch shutdown flag and do something


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
                    unoDataStr = UNO.read(15)  # READ DATA FORMATTED AS ('< 1 >')
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
                    megaDataStr = MEGA.read(25)  # READ DATA FORMATTED AS ('< 1 2 3 4 5 >')
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
                time.sleep(0.4)  # < MAYBE IMPORTANT


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

 # startMainFile STACK IO
    # RECEIVE from STACKS:
    #
    #
    #
    # SEND to STACKS:
    #
    #
    #

# MAIN FILE START
def startMainFile(speed, difficulty, drillType):  # , args): ## NOT A THREAD, performs the bulk of calculation
    # # _______________________________Main Processing_____________________________________

    # __ VARIABLES _____#
    # startCommand = 0
    stereo_Distance = 0.0
    avg_measures = 10
    lead_time = 3
    z_dist_deque = deque([])
    measure_time_deque = deque([])

    # __ GUI Input __ #
    guiData = GuiInput
    guiData.speed = speed
    guiData.difficulty = difficulty
    guiData.drilltype = drillType
    print("[MainThread] : ")
    print("Speed:  " + str(guiData.speed) + "  " + "Diff:  " + str(
        guiData.difficulty) + "  " + "Drill:  " + guiData.drilltype)
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
        pitchYawthread = PitchYaw(stereoStack, guiStack, temperatureStack, megaDataStack, finalDistStack,
                                  futureDistStack)
        pitchYawthread.start()
    except Exception as e:
        print('[MainThread] : Pitch and Yaw thread didnt start because of exception ' + e)
    try:
        startLauncherThread = Launcher(megaDataStack, lidar2Stack, guiStack, stereoStack, lidar1Stack, finalDistStack,
                                       temperatureStack, futureDistStack)
        startLauncherThread.start()
    except Exception as e:
        print('[MainThread] : Launcher thread didnt start because of exception ' + e)

        time.sleep(1)

    # ___________ "MAIN THREAD" LOOP __________ #
    while True:
        if drillType == "Dynamic":
            StartTime = time.time()
            # Get(WAIT) for stereoDistance _____________________________
            try:
                stereoResult = stereoStack.peek()
                stereo_Distance = stereoResult.distance
            except Exception as q:
                print("[MainThread] : No data in stereoStack" + q)
                while stereo_Distance is None:
                    stereoResult = stereoStack.peek()
                    stereo_Distance = stereoResult.distance

            if 5 <= stereo_Distance <= 30:  # Meters
                rationaleDistMeasures = 1
                distanceTotal = stereo_Distance

                # Get(NO_WAIT) for Lidar_1_Dist _____________________________
                try:
                    LIDAR_1_Distance = Lidar1Dist(evo)
                    evo.flushOutput()
                    time.sleep(0.05)
                    if LIDAR_1_Distance is not None and abs(stereo_Distance - LIDAR_1_Distance) <= 5:  # <<<<<<USE WEIGHTING FACTOR INSTEAD
                        rationaleDistMeasures += 1
                        distanceTotal += LIDAR_1_Distance
                        lidar1Stack.push(LIDAR_1_Distance)
                    else:
                        lidar1Stack.push(None)  # << None value indicates no GOOD new data

                except Exception as w:
                    print("[MainThread] : LIDAR_1 -> no data" + w)
                    pass

                # Get(NO_WAIT)for Lidar_2_Dist (Run on New Data EVENT() trigger?)  _____________________________
                try:
                    LIDAR_2_Distance = lidar2Stack.peek()
                    if LIDAR_2_Distance is not None and abs(stereo_Distance - LIDAR_2_Distance) <= 5:  # <<<<<<USE WEIGHTING FACTOR INSTEAD
                        rationaleDistMeasures += 1
                        distanceTotal += LIDAR_2_Distance
                except Exception as w:
                    print("[MainThread] : LIDAR_2 -> no data" + w)
                    pass

                PRE_FINAL_DIST = distanceTotal / rationaleDistMeasures

                # MAKE SURE LAUNCHER THREAD OBTAINS SIMILAR VALUE:
                # Launcher_FINAL_DIST = finalDistStack.peek()
                # if newFinalDistLauncher_flag() = True
                # Launcher_FINAL_DIST = finalDistStack.peek()
                # if abs(Launcher_FINAL_DIST - PRE_FINAL_DIST) <= 2 # Meters
                # Do everything
                # else:
                # print("[Main Thread] : Something is wrong with the data flow")

                measure_time_deque.appendleft(time.time() - StartTime)
                if measure_time_deque == avg_measures:
                    measure_time_deque.pop()

                if z_dist_deque == []:  # This is the first measurement
                    z_dist_deque.appendleft(PRE_FINAL_DIST)
                    FUT_FINAL_DIST = None
                elif len(z_dist_deque) < avg_measures:
                    z_dist_deque.appendleft(PRE_FINAL_DIST)
                    FUT_FINAL_DIST = None
                elif len(z_dist_deque) == avg_measures:  # Wait until we have 10 measurement before calculating players speed (if 5 FPS this means 2 sec)
                    temp_dist = z_dist_deque[0] - z_dist_deque[avg_measures - 1]
                    temp_time = sum([elem for elem in measure_time_deque])  # Time for avg_measure measurements

                    playerspeed = temp_dist / temp_time  # meters/second
                    # the idea is to stay ahead of the player by at least a second or two
                    FUT_FINAL_DIST = PRE_FINAL_DIST + playerspeed * lead_time
                    z_dist_deque.appendleft(PRE_FINAL_DIST)
                    z_dist_deque.pop()

                futureDistStack.push(FUT_FINAL_DIST)

            else:
                print("Player is not in Range")
                # Activate GPIO pin for notification LED
                # ***Do something about this***
                time.sleep(0.5)

        elif drillType == "Static":
            print("[MainThread] : Main doesnt do much here")
            time.sleep(1)

        elif drillType == "Manual":
            print("[MainThread] : Main doesnt do much here")
            time.sleep(1)
        else:
            print("[MainThread] : no GUI data")

# ________ Run The Program ____________ #
# startMainFile(3,4,"manual")  # for testing purposes only