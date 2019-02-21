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
import numpy as np
import argparse
import io
import time
import socket

global import_error
try:
    import cv2
    import imutils
    from picamera.array import PiRGBArray
    from picamera import PiCamera
    import_error = False
except ImportError as imp:
    print("IMPORTANT  :   WITHOUT OPENCV3.0 THE STEREOSCOPICS WILL NOT OPERATE" + str(imp))
    import_error = True

import serial
from threading import Event, Thread, Lock
import serial.tools.list_ports
import random
import PID
import sys
#from gui import *  # Custom Module


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
                 getfutureDistStack, shutdown_event, kill_event):
        Thread.__init__(self)
        self.getstereoStack = getstereoStack
        self.getguiStack = getguiStack
        self.getTemperatureStack = getTemperatureStack
        self.getmegaDataStack = getmegaDataStack
        self.getfinalDistStack = getfinalDistStack
        self.getfutureDistStack = getfutureDistStack
        self.shutdown_event = shutdown_event
        self.kill_event = kill_event

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

        # ** __ PID VARIABLES: __ ** #

        P = 1
        I = 0.1
        D = 0.05

        pid = PID.PID(P, I, D)
        pid.SetPoint = 0.0
        pid.setSampleTime(0.15)
        latPixelDisp = 0
        dynamic_pixel_buff = 1000  # (1000/Distance = 200 px max) increase the pixel 'displacement' error that is fed to PID,

        while not startData and not self.shutdown_event.isSet() and not self.kill_event.isSet():
            # _ARDUINO_UNO__SERIAL_OPEN
            try:
                # Initialize Arduino UNO
                UNO = serial.Serial("/dev/uno", 9600)  # change ACM number as found from "ls /dev/tty*"
                UNO.baudrate = 9600
                # ACQUIRE ACCELEROMETER DATA (Single Pitch reading) _______ #
                tempAngle = GetUnoData(UNO, self.shutdown_event, self.kill_event)
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
                print('[PithYaw(Thread)] : Arduino UNO not available' + str(err))
                time.sleep(2)
                continue
            else:
                print("[PitchYaw(Thread)] : Received start data")
                startData = True
                continue

            finally:
                if self.shutdown_event.isSet():
                    print("PitchYaw(Thread)] : STOP BUTTON PRESSED")
                    break
                else:
                    print("[PitchYaw(Thread)] : starting Loop")



        while not self.shutdown_event.isSet() and not self.kill_event.isSet():
            # <<< BEGINNING OF PITCHYAW LOOP _________________________________ ** #
            MEGAdata = self.getmegaDataStack.peek()
            voiceCommand = MEGAdata.voicecommand

            while voiceCommand != "beginVC" and not self.shutdown_event.isSet() and not self.kill_event.isSet():
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
                    while not cameradata and not self.shutdown_event.isSet() and not self.kill_event.isSet():
                        try:
                            stereodata = self.getstereoStack.peek()
                            cameradata = True
                        except:  # EmptyStackError # <- return EmptyStackError if Stack is empty in peek #####!!!!!!!
                            print("[PithYaw(Thread)] : no data in stereoStack")
                            self.shutdown_event.set()

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
                        row = round((usedDistance-0.99)/2)-2
                        if row < 0: row = 0
                        elif row > 11: row = 11
                        pitchAngle = pitchAngleTable[row, 1] + launcherAngle  # << ANGLE IS ALREADY SET BASED ON FUTURE DISTANCE (via row)

                        # ** ___________________ YAW MOTOR SPEED: ______________________ ** #
                        latPixelDisp = (2464 - LeftXcoord - RightXcoord)
                        latPixelDisp += (dynamic_pixel_buff / usedDistance)
                        # PICK MOTOR DIRECTION:
                        if (2464 - LeftXcoord < RightXcoord):
                            latPixelDisp = -latPixelDisp

                        # *** If we need to scale down the max speed as the player gets closer (same pixel disp at different distances correspond to different angles)
                        # if usedDistance < 8: usedDistance = 8 (makes sure minimum motorSpeed=80 (8/25 = 0.31 -> 0.31*255=80))
                        # error_scaling = usedDistance / 25  # <<< ERROR SCALING

                        # lateralDisplacement = latPixelDisp * PixToDistApprox
                        # latDispDeque.appendleft(lateralDisplacement)
                        # if len(latDispDeque) > 10:
                        #     latDispDeque.pop()
                        pid.update(latPixelDisp)
                        pid_output = pid.output  # MAXIMUM OUTPUT IS ROUGHLY: 400
                        # ***NEED PROPER TESTING TO TUNE PID AND OUTPUT SCALING
                        scaled_pid = (pid_output / 400) * 255  # (scaled_pid_output_=0-1) (AT 25m: 1*1*255 = 255, AT 5m: 1*0.2 = 50)
                        # scaled_pid = -+80 at 7.75 m

                        if scaled_pid == 0:
                            scaled_pid = 0
                        elif (scaled_pid < -255):
                            scaled_pid = -255
                        elif (scaled_pid > 255):
                            scaled_pid = 255
                        elif (0 > scaled_pid > -80):
                            scaled_pid = -80
                        elif (0 < scaled_pid < 80):
                            scaled_pid = 80

                        motorSpeed = str(scaled_pid)

                        # ** ___ SEND DATA ___ ** #
                        UNO.reset_output_buffer()
                        data = '<' + str(motorSpeed) + ', ' + str(pitchAngle) + '>'
                        UNO.write(data.encode())
                        print("[PithYaw(Thread)] SENT: <motorSpeed, pitchAngle> =  " + data)
                        time.sleep(0.1)

                    else:
                        if finalDist:
                            usedDistance = FINAL_DIST
                        else:
                            usedDistance = stereoDist
                        # << CASE 2: Have to predict FUT_FINAL_DIST ourselves if we dont get it

                        # ** ________"PREDICT" FUT_FINAL_DIST (z-distance) of the player from available data: ________ ** #

                        if len(dist_deque) == avg_measure:
                            tempDist = dist_deque[0] - dist_deque[avg_measure-1]                # Distance covered in avg_measure measurements
                            temp_time = sum([elem for elem in measure_time_deque])  # Time for avg_measure measurements
                            speed = tempDist / temp_time # meters/second
                            # the idea is to stay ahead of the player by at least a second or two
                            FUT_FINAL_DIST = usedDistance + speed * 3
                            usedDistance = FUT_FINAL_DIST


                        # ** ________________________PITCH ANGLE: ______________________ ** #
                        # Query table for angle at usedDistance
                        row = round((usedDistance-0.99)/2)-2  # << ENSURE THIS IS A MULTIPLE OF 2 BETWEEN 6-34
                        if row < 0: row = 0
                        elif row > 11: row = 11
                        pitchAngle = pitchAngleTable[row, 1]  # << ANGLE IS ALREADY SET BASED ON FUTURE DISTANCE

                        # ** ___________________ YAW MOTOR SPEED: ______________________ ** #
                        latPixelDisp = (2464 - LeftXcoord - RightXcoord)
                        latPixelDisp += (dynamic_pixel_buff / usedDistance)
                        # PICK MOTOR DIRECTION:
                        if (2464 - LeftXcoord < RightXcoord):
                            latPixelDisp = -latPixelDisp
                        # *** If we need to scale down the max speed as the player gets closer (same pixel disp at different distances correspond to different angles)
                        # if usedDistance < 8: usedDistance = 8 (makes sure minimum motorSpeed=80 (8/25 = 0.31 -> 0.31*255=80))
                        # error_scaling = usedDistance / 25  # <<< ERROR SCALING

                        # lateralDisplacement = latPixelDisp * PixToDistApprox
                        # latDispDeque.appendleft(lateralDisplacement)
                        # if len(latDispDeque) > 10:
                        #     latDispDeque.pop()
                        pid.update(latPixelDisp)
                        pid_output = pid.output  # MAXIMUM OUTPUT IS ROUGHLY: 400
                        # ***NEED PROPER TESTING TO TUNE PID AND OUTPUT SCALING
                        scaled_pid = (pid_output / 400) * 255  # (scaled_pid_output_=0-1) (AT 25m: 1*1*255 = 255, AT 5m: 1*0.2 = 50)
                        # scaled_pid = -+80 at 7.75 m

                        if scaled_pid == 0:
                            scaled_pid = 0
                        elif (scaled_pid < -255):
                            scaled_pid = -255
                        elif (scaled_pid > 255):
                            scaled_pid = 255
                        elif (0 > scaled_pid > -80):
                            scaled_pid = -80
                        elif (0 < scaled_pid < 80):
                            scaled_pid = 80

                        motorSpeed = str(scaled_pid)

                        # ** ___ SEND DATA ___ ** #
                        UNO.reset_output_buffer()
                        data = '<' + str(motorSpeed) + ', ' + str(pitchAngle) + '>'
                        UNO.write(data.encode())
                        print("[PithYaw(Thread)] SENT: <motorSpeed, pitchAngle> =  " + data)
                        time.sleep(0.1)

                except Exception as e:
                    print('[PitchYaw(Thread)] : failed because of exception ' + e)
                    continue

            if drillType == "Static" or drillType == "Manual":
                try:
                    cameradata = False
                    while not cameradata and not self.shutdown_event.isSet() and not self.kill_event.isSet():
                        try:
                            stereodata = self.getstereoStack.peek()
                            cameradata = True
                        except:
                            print("[PithYaw(Thread)] : no data in stereoStack")
                            self.shutdown_event.set()

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
                    row = round((usedDistance-0.99)/2)-2
                    if row < 0: row = 0
                    elif row > 11: row = 11
                    pitchAngle = pitchAngleTable[row, 1] + launcherAngle  # << ANGLE IS ALREADY SET BASED ON FUTURE DISTANCE

                    # ** ___________________ YAW MOTOR SPEED: ______________________ ** #
                    latPixelDisp = (2464 - LeftXcoord - RightXcoord)
                    latPixelDisp += (dynamic_pixel_buff / usedDistance)
                    # PICK MOTOR DIRECTION:
                    if (2464 - LeftXcoord < RightXcoord):
                        latPixelDisp = -latPixelDisp
                    # *** If we need to scale down the max speed as the player gets closer (same pixel disp at different distances correspond to different angles)
                    # if usedDistance < 8: usedDistance = 8 (makes sure minimum motorSpeed=80 (8/25 = 0.31 -> 0.31*255=80))
                    # error_scaling = usedDistance / 25  # <<< ERROR SCALING

                    # lateralDisplacement = latPixelDisp * PixToDistApprox
                    # latDispDeque.appendleft(lateralDisplacement)
                    # if len(latDispDeque) > 10:
                    #     latDispDeque.pop()
                    pid.update(latPixelDisp)
                    pid_output = pid.output  # MAXIMUM OUTPUT IS ROUGHLY: 400
                    # ***NEED PROPER TESTING TO TUNE PID AND OUTPUT SCALING
                    scaled_pid = (pid_output / 400) * 255  # (scaled_pid_output_=0-1) (AT 25m: 1*1*255 = 255, AT 5m: 1*0.2 = 50)
                    # scaled_pid = -+80 at 7.75 m

                    if scaled_pid == 0:
                        scaled_pid = 0
                    elif (scaled_pid < -255):
                        scaled_pid = -255
                    elif (scaled_pid > 255):
                        scaled_pid = 255
                    elif (0 > scaled_pid > -80):
                        scaled_pid = -80
                    elif (0 < scaled_pid < 80):
                        scaled_pid = 80

                    motorSpeed = str(scaled_pid)

                    # ** ___ SEND DATA ___ ** #
                    UNO.reset_output_buffer()
                    data = '<' + str(motorSpeed) + ', ' + str(pitchAngle) + '>'
                    UNO.write(data.encode())
                    print("[PithYaw(Thread)] SENT: <motorSpeed, pitchAngle> =  " + data)
                    time.sleep(0.1)
                    # ** ____________________________________________________________________ ** #

                except Exception as e:
                    print('[PitchYaw(Thread)] : failed because of exception ' + e)
                    continue
        if startData:
            UNO.set_output_flow_control(False)

        if self.shutdown_event.isSet():
            print("[PitchYaw] : STOP BUTTON PRESSED")
            time.sleep(2)
            self.shutdown_event.clear()
            # PitchYaw(self.getstereoStack, self.getguiStack, self.getTemperatureStack, self.getmegaDataStack, self.getfinalDistStack,
            #      self.getfutureDistStack, self.shutdown_event, self.kill_event)

        elif self.kill_event.isSet():
            print("[PitchYaw] : EXITING...")
            sys.exit()
        else:
            print("[PitchYaw] : Not sure what went wrong")


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
                 sendTemperatureStack, getfutureDist, shutdown_event, kill_event):
        Thread.__init__(self)
        self.sendMegaDataStack = sendMegaDataStack
        self.sendLidar2Stack = sendLidar2Stack
        self.guiStack = guiStack
        self.getStereoStack = getStereoStack
        self.getLidar1Stack = getLidar1Stack
        self.sendfinalDistStack = sendfinalDistStack
        self.sendTemperatureStack = sendTemperatureStack
        self.getfutureDist = getfutureDist
        self.shutdown_event = shutdown_event
        self.kill_event = kill_event

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
        MegaData = MegaDataClass
        arduino_data = False
        voiceCommand = 0
        diff_time = [0.3, 0.15, 0, -0.15, -0.3]

        # LAUNCHER OPTIONS:
        DYNAMIC_WAIT_TIME = 3
        STATIC_WAIT_TIME = 5
        print("[Launcher(Thread)] : starting launcher thread")

        while not startData and not self.shutdown_event.isSet() and not self.kill_event.isSet():  # <- Add a timeout to the the start loop
            try:
                # _____ open MEGA serial port
                while not arduino_data and not self.shutdown_event.isSet() and not self.kill_event.isSet():
                    try:
                        MEGA = serial.Serial('/dev/mega', 9600)
                        MEGA.timeout = None
                        print("[Launcher(Thread)] : Connected to MEGA")
                        tempData = GetMegaData(MEGA, self.shutdown_event, self.kill_event)
                        arduino_data = True
                    except serial.SerialException as err:
                        print("[Launcher(Thread)] : Arduino MEGA not available" + str(err))
                        time.sleep(2)
                        continue

                # Get TEMPERATURE and voiceCommand --> send to PITCH_YAW Thread
                temperature = int(tempData.strip().split(",")[1])  # temperature = int()
                voiceCommand = int(tempData.strip().split(",")[2])  # voice commands = int(from 1 to 5)
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

                while voiceCommand != beginVC and not self.shutdown_event.isSet() and not self.kill_event.isSet():
                    tempData = GetMegaData(MEGA, self.shutdown_event, self.kill_event)
                    voiceCommand = int(tempData.strip().split(",")[2])  # voice commands = int(from 1 to 5)
                    time.sleep(0.5)

            except serial.SerialException as err:
                print("[Launcher(Thread)] : MEGA not detected" + err)
                continue
            except Exception as e:  # (serial.SerialException)
                print("[Launcher(Thread)] : Error in setup" + str(e))
                self.shutdown_event.set()
                continue
            else:
                print("[Launcher(Thread)] : Setup Complete, Starting LOOP")
                startData = True


        while drillCount <= 5 and not self.shutdown_event.isSet() and not self.kill_event.isSet():
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
                        print("[Launcher(Thread)] : StereoDistance couldnt be converted to float" + str(verr))
                        if stereoData is None:
                            print("[Launcher(Thread)] : ... because Stack is Empty")
                            continue
                        else:
                            self.getStereoStack.pop()
                            print("[Launcher(Thread)] : issue with stereo data, removing it and trying again")
                        continue
                    except:
                        print("[Launcher(Thread)] : Error getting 'getStereoStack'")
                        self.shutdown_event.set()
                        continue
                    else:
                        distanceTotal = stereo_Distance
                        rationaleDistMeasures = 1

                    # _______ RECEIVE MEGA DATA STRING (Wait): lidar_2_Distance, voiceCommand, targetTiming, targetBallSpeed _______#
                    try:
                        tempData = GetMegaData(MEGA, self.shutdown_event, self.kill_event)
                    except serial.SerialException as err:
                        print("[Launcher(Thread)] : MEGA not detected" + err)
                    except Exception as e:  # (serial.SerialException)
                        print("[Launcher(Thread)] : Error in setup" + str(e))
                        success = False
                        while not success and not self.shutdown_event.isSet() and not self.kill_event.isSet():
                            try:
                                tempData = GetMegaData(MEGA, self.shutdown_event, self.kill_event)
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
                        self.kill_event.set()
                        sys.exit()
                    elif voiceCommand == fasterVC:
                        drillSpeed += 1
                    elif voiceCommand == slowerVC:
                        drillSpeed -= 1
                    elif voiceCommand == pauseVC:
                        self.shutdown_event.set()
                        continue  # <-- Starts next iteration of parent loop where the shutdown flag is caught


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

                    # CALCULATE THE ESTIMATE TOF:
                    difficulty_time = diff_time[difficulty-1]
                    estimated_tof = 0.120617 * FINAL_DIST + difficulty_time

                    # In DYNAMIC Mode, the motors spin up before receiving final instructions from MAIN THREAD
                    # ____POLYNOMIAL FIT FROM THEORETICAL VALUES___ #
                    RPM = -1.13635244 * FINAL_DIST ^ 2 + 97.7378699 * FINAL_DIST + 646.034298  # <-- Polynomial fit
                    MOTOR_FREQ = RPM * 0.016666666666667
                    motorSpeed1, motorSpeed2 = round((RPM / 5000) * 255)  # * tempCorrect?  # Value between 0-255 (On 24 V: 0-5000 RPM)

                    # ____SEND MEGA DATA STRING _____#    IF NO VALUE SEND AS '-1'
                    MotorSpeed1 = str(motorSpeed1)
                    MotorSpeed2 = str(motorSpeed2)
                    # ***Randomize targetChoice
                    targetChoice = int(random.choice([1, 2, 3, 4]))
                    # **** NEED TO ADD A FIFTH VALUE FOR FEED/NO-FEED OF BALL****
                    ballFeed = 0
                    data = '<' + MotorSpeed1 + ',' + MotorSpeed2 + ',' + str(targetChoice) + ',' + str(
                        difficulty) + ',' + str(ballFeed) + ','+ str(estimated_tof) + '>'

                    # ____________________ Write data to MEGA ____________________
                    try:
                        MEGA.reset_output_buffer()
                        MEGA.write(data.encode())
                        print("[Launcher(Thread)] : Launch motors starting...")
                    except Exception as e:
                        sendsuccess = False
                        while not sendsuccess and not self.shutdown_event.isSet() and not self.kill_event.isSet():
                            try:
                                MEGA.write(data.encode())
                                print("[Launcher(Thread)] : Launch motors starting...")
                                sendsuccess = True
                            except:
                                print("[Launcher(Thread)] : Mega not responding")
                                if time.time() - startTime >= 1:
                                    self.shutdown_event.set()

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
                            difficulty) + ',' + str(ballFeed) + str(estimated_tof) +  '>'
                        # write data to MEGA
                        if (time.time() - startTime) <= 1:
                            try:
                                MEGA.reset_output_buffer()
                                MEGA.write(data.encode())
                                drillCount += 1  # << ON SUCCESSFUL LAUNCH
                                LaunchTime = time.time()
                                print("[Launcher(Thread)] : Launch signal sent to MEGA")
                            except:
                                sendsuccess = False
                                while not sendsuccess and not self.shutdown_event.isSet() and not self.kill_event.isSet():
                                    try:
                                        MEGA.write(data.encode())
                                        drillCount += 1  # << ON SUCCESSFUL LAUNCH
                                        LaunchTime = time.time()
                                        print("[Launcher(Thread)] : Launch signal (second attempt) sent to MEGA")
                                        sendsuccess = True
                                    except:
                                        if time.time() - startTime >= 1:
                                            print("[Launcher(Thread)] : sending to MEGA timed out")
                                            self.shutdown_event.set()
                        else:
                            print("[Launcher(Thread)] : sending to MEGA timed out")
                            self.shutdown_event.set()

                    else:
                        time.sleep(0.1)  # <-- ALLOW TIME FOR PREVIOUS DATA TO BE READ AND CLEARED FROM THE BUFFER

                        # ____POLYNOMIAL FIT FROM THEORETICAL VALUES___ #
                        RPM = -1.13635244 * FUT_FINAL_DIST ^ 2 + 97.7378699 * FUT_FINAL_DIST + 646.034298  # <-- Polynomial fit
                        MOTOR_FREQ = RPM * 0.016666666666667
                        motorSpeed1, motorSpeed2 = round((RPM / 5000) * 255)  # * tempCorrect? # Value between 0-255 (On 24 V: 0-5000 RPM)

                        # ____SEND MEGA DATA STRING _____#    IF NO VALUE SEND AS '-1'
                        MotorSpeed1 = str(motorSpeed1)
                        MotorSpeed2 = str(motorSpeed2)
                        # **** NEED TO ADD A FIFTH VALUE FOR FEED/NO-FEED OF BALL****
                        ballFeed = 1
                        data = '<' + MotorSpeed1 + ',' + MotorSpeed2 + ',' + str(targetChoice) + ',' + str(difficulty) + ',' + str(ballFeed) + ','+ str(estimated_tof) + '>'
                        # Write data to MEGA
                        if (time.time() - startTime) <= 1:
                            try:
                                MEGA.reset_output_buffer()
                                MEGA.write(data.encode())
                                drillCount += 1  # << ON SUCCESSFUL LAUNCH
                                LaunchTime = time.time()
                                print("[Launcher(Thread)] : Launch signal sent to MEGA")
                            except:
                                sendsuccess = False
                                while not sendsuccess and not self.shutdown_event.isSet() and not self.kill_event.isSet():
                                    try:
                                        MEGA.write(data.encode())
                                        drillCount += 1  # << ON SUCCESSFUL LAUNCH
                                        LaunchTime = time.time()
                                        print("[Launcher(Thread)] : Launch signal sent to MEGA")
                                        sendsuccess = True
                                    except:
                                        if time.time() - startTime >= 1:
                                            self.shutdown_event.set()
                        else:
                            print("[Launcher(Thread)] : process took too long to keep up with drill")
                            self.shutdown_event.set()



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
                        self.shutdown_event.set()
                        continue
                    else:
                        distanceTotal = stereo_Distance
                        rationaleDistMeasures = 1

                    # _______ RECEIVE MEGA DATA STRING (Wait): lidar_2_Distance, voiceCommand, targetTiming, targetBallSpeed _______#
                    try:
                        tempData = GetMegaData(MEGA, self.shutdown_event, self.kill_event)
                    except serial.SerialException as err:
                        print("[Launcher(Thread)] : MEGA not detected" + err)
                    except Exception as e:  # (serial.SerialException)
                        print("[Launcher(Thread)] : Error in setup" + e)
                        success = False
                        while not success and not self.shutdown_event.isSet() and not self.kill_event.isSet():
                            try:
                                tempData = GetMegaData(MEGA, self.shutdown_event, self.kill_event)
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
                        self.kill_event.set()
                        sys.exit()
                    elif voiceCommand == fasterVC:
                        drillSpeed += 1
                    elif voiceCommand == slowerVC:
                        drillSpeed -= 1
                    elif voiceCommand == pauseVC:
                        self.shutdown_event.set()
                        continue  # <-- Starts next iteration of parent loop where the shutdown flag is caught

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

                    difficulty_time = diff_time[difficulty-1]
                    estimated_tof = 0.120617 * FINAL_DIST + difficulty_time

                    # In DYNAMIC Mode, the motors spin up before receiving final instructions from MAIN THREAD
                    # ____POLYNOMIAL FIT FROM THEORETICAL VALUES___ #
                    RPM = -1.13635244 * FINAL_DIST ^ 2 + 97.7378699 * FINAL_DIST + 646.034298  # <-- Polynomial fit
                    MOTOR_FREQ = RPM * 0.016666666666667
                    motorSpeed1, motorSpeed2 = round(
                        (RPM / 5000) * 255)  # * tempCorrect?  # Value between 0-255 (On 24 V: 0-5000 RPM)

                    # ____SEND MEGA DATA STRING _____#    IF NO VALUE SEND AS '-1'
                    MotorSpeed1 = str(motorSpeed1)
                    MotorSpeed2 = str(motorSpeed2)
                    # ***Randomize targetChoice
                    targetChoice = int(random.choice([1, 2, 3, 4]))
                    # **** NEED TO ADD A FIFTH VALUE FOR FEED/NO-FEED OF BALL****
                    ballFeed = 1
                    data = '<' + MotorSpeed1 + ',' + MotorSpeed2 + ',' + str(targetChoice) + ',' + str(difficulty) + ',' + str(ballFeed) + ',' + str(estimated_tof) + '>'

                    # ____________________ Write data to MEGA ____________________
                    try:
                        MEGA.write(data.encode())
                    except Exception as e:
                        sendsuccess = False
                        while not sendsuccess and not self.shutdown_event.isSet() and not self.kill_event.isSet():
                            try:
                                MEGA.reset_output_buffer()
                                MEGA.write(data.encode())
                                drillCount += 1  # << ON SUCCESSFUL LAUNCH
                                LaunchTime = time.time()
                                print("[Launcher(Thread)] : Launch signal sent to MEGA")
                                sendsuccess = True
                            except:
                                if time.time() - startTime >= 1:
                                    self.shutdown_event.set()

                if drillType == 'Manual':  # ________________________________________________________________________ #

                    while voiceCommand != beginVC and voiceCommand != stopVC and not self.shutdown_event.isSet() and not self.kill_event.isSet():
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
                            self.shutdown_event.set()
                            continue
                        else:
                            distanceTotal = stereo_Distance
                            rationaleDistMeasures = 1

                        # _______ RECEIVE MEGA DATA STRING (Wait): lidar_2_Distance, voiceCommand, targetTiming, targetBallSpeed _______#
                        try:
                            tempData = GetMegaData(MEGA, self.shutdown_event, self.kill_event)
                        except serial.SerialException as err:
                            print("[Launcher(Thread)] : MEGA not detected" + err)
                        except Exception as e:  # (serial.SerialException)
                            print("[Launcher(Thread)] : Error in setup" + e)
                            success = False
                            while not success and not self.shutdown_event.isSet() and not self.kill_event.isSet():
                                try:
                                    tempData = GetMegaData(MEGA, self.shutdown_event, self.kill_event)
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
                            self.shutdown_event.set()
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

                    difficulty_time = diff_time[difficulty-1]
                    estimated_tof = 0.120617 * FINAL_DIST + difficulty_time

                    # In DYNAMIC Mode, the motors spin up before receiving final instructions from MAIN THREAD
                    # ____POLYNOMIAL FIT FROM THEORETICAL VALUES___ #
                    RPM = -1.13635244 * FINAL_DIST ^ 2 + 97.7378699 * FINAL_DIST + 646.034298  # <-- Polynomial fit
                    MOTOR_FREQ = RPM * 0.016666666666667
                    motorSpeed1, motorSpeed2 = round((RPM / 5000) * 255)  # * tempCorrect?  # Value between 0-255 (On 24 V: 0-5000 RPM)

                    # ____SEND MEGA DATA STRING _____#    IF NO VALUE SEND AS '-1'
                    MotorSpeed1 = str(motorSpeed1)
                    MotorSpeed2 = str(motorSpeed2)
                    # ***Randomize targetChoice
                    targetChoice = int(random.choice([1, 2, 3, 4]))
                    # **** NEED TO ADD A FIFTH VALUE FOR FEED/NO-FEED OF BALL****
                    ballFeed = 1
                    data = '<' + MotorSpeed1 + ',' + MotorSpeed2 + ',' + str(targetChoice) + ',' + str(difficulty) + ',' + str(ballFeed) + ','+ str(estimated_tof) + '>'

                    # ____________________ Write data to MEGA ____________________
                    try:
                        MEGA.reset_output_buffer()
                        MEGA.write(data.encode())
                    except Exception as e:
                        sendsuccess = False
                        while not sendsuccess and not self.shutdown_event.isSet() and not self.kill_event.isSet():
                            try:
                                MEGA.reset_output_buffer()
                                MEGA.write(data.encode())
                                sendsuccess = True
                            except:
                                if time.time() - startTime >= 1:
                                    self.shutdown_event.set()

            except Exception as e:
                print('[Launcher(Thread)] : failed because of exception: ' + e)
                self.shutdown_event.set()

            if startData:
                MEGA.set_output_flow_control(False)
            # except:   # <-- put as many as needed, order is important
            # else:     # <-- Runs if no exception raised
            # finally:  # <-- Runs no matter what
        if self.shutdown_event.isSet():
            print("[Launcher(Thread)] : STOP BUTTON PRESSED")
            time.sleep(2)
            self.shutdown_event.clear()
            # Launcher(self.sendMegaDataStack, self.sendLidar2Stack, self.guiStack, self.getStereoStack, self.getLidar1Stack, self.sendfinalDistStack,
            #      self.sendTemperatureStack, self.getfutureDist, self.shutdown_event, self.kill_event)

        elif self.kill_event.isSet():
            print("[Launcher(Thread)] : EXITING...")
            sys.exit()
        elif drillCount == 5:
            print("[Launcher(Thread)] : Drill COMPLETE!")
        else:
            print("[Launcher(Thread)] : Not sure what went wrong")
        # if self.shutdown_event.isSet(): # <-- catch shutdown flag and do something


def StereoscopicsThread(stereoStack, shutdown_event, kill_event):
    focalsize = 3.04e-03
    pixelsize = 1.12e-06
    baseline = 1.0
    # datapoints = 5
    # centroid = (0, 0)
    # compvalue = "1.0"

    TCP_IP = '169.254.116.12'
    TCP_PORT = 5025
    BUFFER_SIZE = 1024
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    ap = argparse.ArgumentParser()
    ap.add_argument("-v", "--video", help="path to the (optional) video file")
    ap.add_argument("-b", "--buffer", type=int, default=8, help="max buffer size")
    args = vars(ap.parse_args())

    # define the lower and upper boundaries of the jersey ball in the HSV color space, then initialize the list of tracked points
    jerseyLower1 = (0, 70, 70)  # currently set for red
    jerseyUpper1 = (10, 255, 255)
    jerseyLower2 = (170, 70, 70)  # currently set for red
    jerseyUpper2 = (180, 255, 255)
    pts = deque(maxlen=args["buffer"])

    connected = False
    print('[Stereo] :       before connection loop')
    while not connected and not shutdown_event.isSet() and not kill_event.isSet():  # Wait for client
        try:
            s.bind((TCP_IP, TCP_PORT))
            s.listen(1)
            clientPort, addr = s.accept()
            connected = True
            print("[Stereo] : Client connected")
        except socket.error:
            print('[Stereo] :       No Client')
            time.sleep(3)
            continue
        except Exception as e:
            print('[Stereo] :       No Client' + str(e))
            time.sleep(3)
            continue

    def ProcessLoop(vs, clientPort, BUFFER_SIZE):
        ##________________BEGINNING OF LOOP________________##
        while not shutdown_event.isSet() and not kill_event.isSet():
            print("looping")
            start_time = time.time()

            try:
                image = vs.read()
                image = imutils.resize(image, width=600, height=450)
            except AttributeError as e:
                print("[Stereo.ProcessLoop] : no image")
                capture = False
                while not capture and not shutdown_event.isSet() and not kill_event.isSet():
                    image = vs.read()
                    image = imutils.resize(image, width=600, height=450)
                    try:
                        image = vs.read()
                        image = imutils.resize(image, width=600, height=450)
                        capture = True
                    except AttributeError as e:
                        print("[Stereo.ProcessLoop] : no image")
                        capture = False
                        continue
                    except:
                        shutdown_event.set()

            blurred = cv2.GaussianBlur(image, (11, 11), 0)
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

            mask0 = cv2.inRange(hsv, jerseyLower1, jerseyUpper1)
            mask1 = cv2.inRange(hsv, jerseyLower2, jerseyUpper2)
            mask = mask0 + mask1
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)

            # find contours in the mask and initialize the current (x, y) center of the ball
            cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cnts = imutils.grab_contours(cnts)
            center = None

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

            # print("test1")
            # loop over the set of tracked points
            for i in range(1, len(pts)):
                # if either of the tracked points are None, ignore
                # them
                if pts[i - 1] is None or pts[i] is None:
                    continue

                # otherwise, compute the thickness of the line and draw the connecting lines
                thickness = int(np.sqrt(args["buffer"] / float(i + 1)) * 2.5)
                cv2.line(image, pts[i - 1], pts[i], (0, 0, 255), thickness)
            # print("test2")
            try:
                # print('[StereoscopicThread] : waiting for data')
                print("[Stereo] : test1")
                data = clientPort.recv(BUFFER_SIZE)

                print("[Stereo] : test2")
                # print("[StereoscopicThread] : received data:", data)
                clientPort.send(data)  # SEND DATA BACK (COULD BE USED FOR STOP COMMAND)
                print("[Stereo] : test3")
                compvalue = data.decode()
                print(compvalue)
                print("[Stereo] : test4")
            except socket.error:
                connected = False
                while not connected and not shutdown_event.isSet() and not kill_event.isSet():
                    try:
                        data = clientPort.recv(BUFFER_SIZE)
                        # print("[StereoscopicThread] : received data:", data)
                        clientPort.send(data)  # SEND DATA BACK (COULD BE USED FOR STOP COMMAND)
                        compvalue = data.decode()
                        print(compvalue)
                        connected = True
                    except socket.error:
                        print("[Stereo] : Lost the client connection")
                        time.sleep(0.5)
                        continue

            if len(cnts) > 0:
                # print("Decoded value: " + compvalue)
                slaveval = float(compvalue)
                masterval = centroid[0]
                disparity = abs(masterval - slaveval)
                distance = (focalsize * baseline) / (disparity * pixelsize)
                # SENDS DATA TO Class, which can be "Put" using Stacks's______________________________
                results = StereoOutput
                results.distance = distance
                results.disparity = disparity
                results.masterval = masterval
                results.slaveval = slaveval
                stereoStack.push(results)
                fps = time.time() - start_time
                print("[Stereo] :   FPS =  " + str(fps) + "||    Distance:  " + str(distance))

    class PiVideoStream:
        def __init__(self, resolution=(600, 450), framerate=32):
            # initialize the camera and stream
            self.camera = PiCamera()
            self.camera.resolution = resolution
            self.camera.framerate = framerate
            self.rawCapture = PiRGBArray(self.camera, size=resolution)
            self.stream = self.camera.capture_continuous(self.rawCapture, format="bgr", use_video_port=True)

            # initialize the frame and the variable used to indicate
            # if the thread should be stopped
            self.frame = None
            self.stopped = False
            time.sleep(1)

        def start(self):
            # start the thread to read frames from the video stream
            Thread(target=self.update, args=()).start()
            return self

        def update(self):
            # keep looping infinitely until the thread is stopped
            for f in self.stream:
                # grab the frame from the stream and clear the stream in
                # preparation for the next frame
                self.frame = f.array
                self.rawCapture.truncate(0)

                # if the thread indicator variable is set, stop the thread
                # and resource camera resources
                if self.stopped:
                    self.stream.close()
                    self.rawCapture.close()
                    self.camera.close()
                    return

        def read(self):
            # return the frame most recently read
            return self.frame

        def stop(self):
            # indicate that the thread should be stopped
            self.stopped = True

    def read(self):
        # return the frame most recently read
        return self.frame

    def stop(self):
        # indicate that the thread should be stopped
        self.stopped = True

    # created a *threaded* video stream, allow the camera sensor to warmup,
    # and start the FPS counter
    print("[Stereo] starting THREADED frames from `picamera` module...")

    vs = PiVideoStream().start()

    print("[Stereo] : Initializing camera")
    time.sleep(2.0)

    ProcessLoop(vs, clientPort, BUFFER_SIZE)


def GetUnoData(UNO, shutdown_event, kill_event):
    getdata = False
    dataPresent = False
    while getdata == False:
        #print('[GetUnoData] : in UNO Serial while 1')
        if UNO.is_open:
            #print('[GetUnoData] : in the UNO is_open')
            while not dataPresent and not shutdown_event.isSet() and not kill_event.isSet():

                UNO.reset_input_buffer()

                print('[GetUnoData] : in UNO Serial while 2')
                try:
                    print('[GetUnoData] : trying to read startMarker')
                    startMarker = UNO.read().decode()
                except Exception as e:
                    print('[GetUnoData] : didnt get UNO data' + e)
                if startMarker == "<":
                    unoDataStr = UNO.read(15)  # READ DATA FORMATTED AS ('< 1 >')
                    unoDataTemp = list(unoDataStr.decode())
                    unoDataTemp.insert(0, startMarker)
                    unoData = unoDataTemp[:unoDataTemp.index(">") + 1]
                    print("[GetUnoData] : acquired UnoData  ->  ", unoData)
                    tempData = "".join(unoData)
                    print("[GetUnoData] : tempData string  ->  ", tempData)
                    dataPresent = True
                    print('[GetUnoData] : data read from UNO')
                    getdata = True
                    return tempData
        else:
            time.sleep(0.1)
            continue
        time.sleep(0.2)  # << MAY NEED TO BE CHANGED IF READ DATA IS GARBAGE


def GetMegaData(MEGA, shutdown_event, kill_event):
    getdata = False
    dataPresent = False

    while getdata == False and not shutdown_event.isSet() and not kill_event.isSet():
        print('[GetMegaData] : in Mega Serial while 1')
        if MEGA.is_open:
            print('[GetMegaData] : in the Mega is_open')
            while not dataPresent and not shutdown_event.isSet() and not kill_event.isSet():

                MEGA.reset_input_buffer()

                print('[GetMegaData] : in Mega Serial while 2')
                try:
                    print('[GetMegaData] : trying to read startMarker')
                    startMarker = MEGA.read().decode()
                except Exception as e:
                    print('[GetMegaData] : didnt get Mega data' + str(e))
                if startMarker == "<":
                    megaDataStr = MEGA.read(25)  # READ DATA FORMATTED AS ('< 1 2 3 4 5 >')
                    megaDataTemp = list(megaDataStr.decode())
                    megaDataTemp.insert(0, startMarker)
                    megaData = megaDataTemp[:megaDataTemp.index(">") + 1]
                    print("[GetMegaData] : acquired MegaData  ->  ", megaData)
                    tempData = "".join(megaData)
                    print("[GetMegaData] : tempData string  ->  ", tempData)
                    dataPresent = True
                    getdata = True
                    return tempData
        else:
            time.sleep(0.1)
            continue
        time.sleep(0.4)

def findEvo():
    # Find Live Ports, return port name if found, NULL if not
    # print ('Scanning all live ports on this PC')
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        #print p # This causes each port's information to be printed out.
        if "5740" in p[2]:
            # print ('Evo found on port ' + p[0])
            return(p[0])
    return ('NULL')

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
def startMainFile(speed, difficulty, drillType, shutdown_event, kill_event, PitchYawStart, LauncherStart, EvoStart):  # , args): ## NOT A THREAD, performs the bulk of calculation
    # # _______________________________Main Processing_____________________________________
    if not shutdown_event.isSet() and not kill_event.isSet():
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

        StartPitchYaw = PitchYawStart
        StartLauncher = LauncherStart
        EvoLidar = EvoStart

        print("[MainThread] : ")
        print("Speed:  " + str(guiData.speed) + "  " + "Diff:  " + str(
            guiData.difficulty) + "  " + "Drill:  " + guiData.drilltype)
        print("_______________________________________")

        guiStack.push(guiData)
        # ___ OPEN SERIAL PORT/S ___ #
        if EvoLidar:
            evo_data = False

            while not evo_data and not shutdown_event.isSet() and not kill_event.isSet():
                try:
                    port = findEvo()
                    evo = serial.Serial(port, baudrate=115200, timeout=2)
                    set_text = (0x00, 0x11, 0x01, 0x45)
                    evo.flushInput()
                    evo.write(set_text)
                    evo.flushOutput()
                    print("[MainThread] : Connected to Evo (LIDAR1)")
                    evo_data = True
                except serial.SerialException as e:
                    print("[MainThread] : Cannot find Evo LIDAR1" + str(e))
                    time.sleep(2)
                    continue
                except:
                    print("[MainThread] : Cannot find Evo LIDAR1")
                    time.sleep(2)
                    continue

    # _______________________ Start Threads _______________________--_ #

    if not shutdown_event.isSet() and not kill_event.isSet():
        print("Starting Threads")
        if import_error:
            print("[MainThread] : STERESCOPICS NOT STARTING")
        else:
            try:
                StereoCameras = Thread(target=StereoscopicsThread, args=[stereoStack, shutdown_event, kill_event])
                StereoCameras.start()
            except Exception as e:
                print('[MainThread] : Stereo thread failed because of exception ' + str(e))

        time.sleep(3)

        if StartPitchYaw:
            try:
                pitchYawthread = PitchYaw(stereoStack, guiStack, temperatureStack, megaDataStack, finalDistStack, futureDistStack, shutdown_event, kill_event)
                pitchYawthread.start()
            except Exception as e:
                print('[MainThread] : pitchYawthread didnt start because of exception ' + str(e))
        else:
            print('[MainThread] : pitchYawthread thread is not starting')

        if StartLauncher:
            try:
                startLauncherThread = Launcher(megaDataStack, lidar2Stack, guiStack, stereoStack, lidar1Stack, finalDistStack, temperatureStack, futureDistStack, shutdown_event, kill_event)
                startLauncherThread.start()
            except Exception as e:
                print('[MainThread] : Launcher thread didnt start because of exception ' + str(e))
                time.sleep(1)
        else:
            print('[MainThread] : startLauncherThread thread is not starting')


    if shutdown_event.isSet():
        print("[MainThread] : STOP BUTTON PRESSED")
    if kill_event.isSet():
        print("[MainThread] : EXITING...")
        sys.exit()

    # ___________ "MAIN THREAD" LOOP __________ #
    while not shutdown_event.isSet() and not kill_event.isSet():
        if drillType == "Dynamic":
            StartTime = time.time()
            # Get(WAIT) for stereoDistance _____________________________
            try:
                stereoResult = stereoStack.peek()
                stereo_Distance = stereoResult.distance

            except AttributeError as att:
                print("[MainThread] : No data in stereoStack" + str(att))
                stereo_present = False
                while not stereo_present and not shutdown_event.isSet() and not kill_event.isSet():
                    try:
                        stereoResult = stereoStack.peek()
                        stereo_Distance = stereoResult.distance
                        stereo_present = True
                    except:
                        print("[MainThread] : Waiting for Stereo...")
                        time.sleep(2)
                        continue

            except Exception as q:
                print("[MainThread] : No data in stereoStack" + str(q))
                stereo_present = False
                while not stereo_present and not shutdown_event.isSet() and not kill_event.isSet():
                    try:
                        stereoResult = stereoStack.peek()
                        stereo_Distance = stereoResult.distance
                        stereo_present = True
                    except:
                        print("[MainThread] : Waiting for Stereo...")
                        continue

            if 5 <= stereo_Distance <= 30:  # Meters
                rationaleDistMeasures = 1
                distanceTotal = stereo_Distance

                # Get(NO_WAIT) for Lidar_1_Dist _____________________________
                if EvoLidar:
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
                        print("[MainThread] : LIDAR_1 -> no data" + str(w))
                        pass

                # Get(NO_WAIT)for Lidar_2_Dist (Run on New Data EVENT() trigger?)  _____________________________
                try:
                    LIDAR_2_Distance = lidar2Stack.peek()
                    if LIDAR_2_Distance is not None and abs(stereo_Distance - LIDAR_2_Distance) <= 5:  # <<<<<<USE WEIGHTING FACTOR INSTEAD
                        rationaleDistMeasures += 1
                        distanceTotal += LIDAR_2_Distance
                except Exception as w:
                    print("[MainThread] : LIDAR_2 -> no data" + str(w))
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
                print("[MainThread] : Player is not in Range")
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

    if shutdown_event.isSet():
        print("[MainThread] : STOP BUTTON PRESSED")
        time.sleep(2)
    elif kill_event.isSet():
        print("[MainThread] : EXITING...")
        sys.exit()
    else:
        print("[MainThread] : Not sure what went wrong")