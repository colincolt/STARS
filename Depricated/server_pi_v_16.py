
'''# _________________________________READ ME______________________________________#
THREADING:
This file uses the python "threading" module to run any number of "Threads"
simultaneously, anything that needs to run continuously can be put into an
infinite loop, anything that needs to run independantly and not interfere
with other tasks. THREADS are ideal for input output operations, we do not
have a lot of math being done.

HELP:


STACKS:
<     https://interactivepython.org/runestone/static/pythonds/BasicDS/ImplementingaStackinPython.html       >
<     https://www.pythoncentral.io/stack-tutorial-python-implementation/  '''

working_on_the_Pi = True

# Packages
try:
    import tkinter
    import serial
    from threading import Event, Thread, Lock
    import serial.tools.list_ports
    import random
    import PID
    import sys
    from collections import deque
    import numpy as np
    import argparse
    import io
    import time
    import socket
    import cv2
    import imutils
    if working_on_the_Pi:
        try:
            from gpiozero import LED
            from picamera.array import PiRGBArray
            from picamera import PiCamera
        except ImportError as imp:
            print("IMPORTANT  :   ARE YOU WORKING THE RASPBERRY PI ?:  ", imp)
    import_error = False
except ImportError as imp:
    print("IMPORTANT  :   IMPORT ERROR:  " , imp , "!!!!!")
    import_error = True
    pass


# from gui import *  # Custom Module

class stack_data:
    def __init__(self, *args):
        self.args = args


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
futureDistStack = Stack()
send_mega_stack = Stack()

# GLOBAL LED NOTIFICATION LIGHTS
if working_on_the_Pi == True:
    RED_1 = LED(13)
    RED_2 = LED(21)
    RED_3 = LED(6)
    green = LED(5)
    YELLOW = LED(26)
    WHITE = LED(19)
    blue = LED(16)

# GLOBAL FLAGS/EVENTS:

# GLOBAL VARIABLES:
startMarker = '<'

def gpio_blinker(color, loop_count):
    if working_on_the_Pi == True:
        if loop_count % 2 == 0:
            color.on()
        else:
            color.off()

def loop_counter(loop_number):
    loop_number += 1
    if loop_number >= 10:
        loop_number = 1
    return loop_number
# _______PITCH AND YAW THREAD________ #

class PitchYaw(Thread):
    '''PITCH_YAW_THREAD: ---> Arduino UNO provide Angle values to both motors, request temperature's from the Uno, and distance measurements
RECEIVE from UNO:
tchAngle <-- to linear actuator
# - motorSpeed <-- to the yaw motor

# Stack IO
# RECEIVE from STACKS:
# - drillType from guiStack
# - stereoDist from stereoStack
# - FINAL_DIST from finalDistStack
# - FUT_FINAL_DIST from futureDistStack- launchAngle (from accelerometer)
SEND to UNO:
- pi
SEND to STACKS:
- launcherAngle -->  launcherAngleStack --> Launcher(Thread)
'''

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
        # ** __ RECORD A LIST OF OLD MEASUREMENTS FOR TRACKING: __ ** #
        self.dist_deque = deque([])
        self.x_disp_deque = deque([])
        self.measure_time_deque = deque([])

        # ** __ DATA TRACKERS: __ ** #
        self.startData = False
        self.last_start_time = None

        # ** __ IMPORTANT VARIABLES: __ ** #
        self.avg_measure = 10
        self.LEAD_SPEED = 25  # << Adjust the added motorSpeed of the YAW for DYNAMIC MODE (0-255)
        self.MAX_YAW_SPEED = 0.15  # << Maximum speed of Yaw in radians
        self.color = YELLOW
        self.self.usedDistance = 0.0
        # ** __ PID VARIABLES: __ ** #
        self.P = 0.3
        self.I = 0.0
        self.D = 0.0

        self.pid = PID.PID(self.P, self.I, self.D)
        self.pid.SetPoint = 0.0
        self.pid.setSampleTime(0.15)
        self.dynamic_pixel_buff = 1000  # (1000/Distance = 200 px max) increase the pixel 'displacement' error that is fed to PID,
        self.launcherAngle = 0
        self.low_limit = 80  # < LOWER LIMIT FOR YAW MOTOR POWER
        self.UNO = None
        self.LeftXcoord = 0.0
        self.RightXcoord = 0.0
        self.latPixelDisp = 0.0
        self.start_time = 0.0
        self.last_start_time = None
        self.first_measure = True
        self.drillType = ""
        self.py_loop_count = 1
        self.endtime = None
        # self.stereodata.masterval = 800

        #        self.pitchAngleTable = np.array([[5, 0],  # << Pitch angle lookup table based on estimations
        #                                    [7, 5],
        #                                    [9, 10],
        #                                    [11, 15],
        #                                    [13, 20],
        #                                    [15, 25],
        #                                    [17, 30],
        #                                    [19, 34],
        #                                    [21, 35],
        #                                    [23, 36],
        #                                    [25, 37]])

        self.pitchAngleTable = np.array([[5, 27],  # << Pitch angle lookup table based on estimations
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





    def shutdown_reset_function(self):
        if self.shutdown_event.isSet():
            motorSpeed = "300"
            pitchAngle = "0"
            data = '<' + motorSpeed + ', ' + pitchAngle + '>'
            self.UNO.write(data.encode())

        elif self.kill_event.isSet():
            motorSpeed = "300"
            pitchAngle = "0"
            data = '<' + motorSpeed + ', ' + pitchAngle + '>'
            self.UNO.write(data.encode())

    def motor_controller(self):
        # ** _________________ PITCH ANGLE: __________________ ** #

        # Query table for angle at self.usedDistance
        row = round((self.usedDistance - 0.99) / 2) - 2
        if row < 0:
            row = 0
        elif row > 11:
            row = 11
        pitchAngle = self.pitchAngleTable[row, 1] + self.launcherAngle

# ''' PID CALCULATION:(scaled_pid_output_=0-1) (AT 25m: 1*1*255 = 255, AT 5m: 1*0.2 = 50)'''

        self.pid.update(self.latPixelDisp)
        pid_output = self.pid.output  # MAXIMUM OUTPUT IS ROUGHLY: 400
        scaled_pid = int((pid_output / 300) * 255)

        if abs(scaled_pid) <= 10:
            mapped_pid = 0
        elif scaled_pid <= -100:
            mapped_pid = -100
        elif scaled_pid >= 100:
            mapped_pid = 100
        elif scaled_pid == 0:
            mapped_pid = 0
        elif scaled_pid > 0:
            mapped_pid = np.interp(scaled_pid, [0, 100], [60, 100])
        elif scaled_pid < 0:
            mapped_pid = np.interp(scaled_pid, [-100, 0], [-100, -60])


        motorSpeed = str(int(mapped_pid))

        # ** ___ SEND DATA ___ ** #
        self.UNO.reset_output_buffer()
        data = '<' + str(motorSpeed) + ', ' + str(pitchAngle) + '>'
        self.UNO.write(data.encode())
        print("[PitchYaw:Future] SENT: <motorSpeed, pitchAngle> =  " + data)


    def get_stereo(self):
        # ___________________ GET stereoStack DATA ___________________ #
        cameradata = False
        while not cameradata and not self.shutdown_event.isSet() and not self.kill_event.isSet():
            try:
                self.stereodata = self.getstereoStack.peek()
                self.LeftXcoord = self.stereodata.masterval
                self.RightXcoord = self.stereodata.slaveval
                self.stereoDist = int(self.stereodata.distance)
                cameradata = True
            except ValueError as verr:
                print("[PitchYaw] : StereoDistance couldnt be converted to float" + str(verr))
                if self.stereodata is None:
                    print("[PitchYaw] : ... because Stack is Empty")
                    continue
                else:
                    self.getStereoStack.pop()
                    print("[PitchYaw] : issue with stereo data, removing it and trying again")
                continue
            except Exception as e:
                print("[PitchYaw] : Error getting getStereoStack" + str(e))
                # time.sleep(2)
                continue
            else:
                # ** ___________________ YAW MOTOR SPEED: ______________________ ** #
                self.latPixelDisp = (2464 - self.LeftXcoord - self.RightXcoord)

    # def wait_for_begin(self):
        # _____________ Get MEGA Data _____________

        # MEGAdata = self.getmegaDataStack.peek()
        # voiceCommand = MEGAdata.voicecommand
        #
        # while voiceCommand != "beginVC" and not self.shutdown_event.isSet() and not self.kill_event.isSet():
        #     try:
        #         MEGAdata = self.getmegaDataStack.peek()
        #         voiceCommand = MEGAdata.voicecommand  # voice commands = int(from 1 to 5)
        #     except AttributeError as novoice:
        #         print("[PitchYaw] : No VoiceCommand" + str(novoice))
        #         time.sleep(0.5)
        #     except Exception as e:
        #         print("[PitchYaw] : Exception" + str(e))


    # def lateral_speed(self):
    #     ___________________ PLAYER SPEED DEQUE ___________________ #
    #
    #     if len(self.measure_time_deque) >= 2 prand len(self.dist_deque) >= 2:
    #         displacement = self.dist_deque[len(self.dist_deque) - 1] - self.dist_deque[len(self.dist_deque) - 2]
    #         changein_time = self.measure_time_deque[len(self.measure_time_deque) - 1] - self.measure_time_deque[ len(self.measure_time_deque) - 2]
    #         self.Check_speed = displacement / changein_time
    #         print("Check Speed" + str(Check_speed))
    #         if Check_speed >= 4: #might need to pop both points
    #             measure_time_deque.pop(len(measure_time_deque)-1)
    #             dist_deque.pop(len(dist_deque)-1)
    #     ________________________________________________________________________ ##

    def startup(self):
        while not self.startData and not self.shutdown_event.isSet() and not self.kill_event.isSet():
            try:
                # Initialize Arduino UNO
                uno_port = findUNO()
                self.UNO = serial.Serial(uno_port, 115200, timeout=1)  # change ACM number as found from "ls /dev/tty*"
                self.UNO.baudrate = 115200

                # _________   ___ Get ACCELEROMETER Data  ____________ #

                # tempAngle = GetUnoData(UNO, self.shutdown_event, self.kill_event)

                # ___________ Get TEMPERATURE Data _______________

                # temperature = self.getTemperatureStack.peek()
                self.tempCorrection = 1  # temperature / 25  # <<<tempCorrection factor

                # ___________ Get GUI Data _______________

                guiData = self.getguiStack.peek()
                self.drillType = guiData.drilltype
                # difficulty = guiData.difficulty
                # drillSpeed = guiData.speed

            except Exception as err:
                print('[PitchYaw] : Arduino UNO not available' + str(err))
                time.sleep(2)
                continue
            else:
                print("[PitchYaw] : Received start data")
                self.startData = True
                continue
            finally:
                if self.shutdown_event.isSet():
                    print("[PitchYaw] : STOP BUTTON PRESSED")
                    break
                else:
                    print("[PitchYaw] : UNO Connected... starting loop")
        print("[PitchYaw] : Starting")
        time.sleep(5)

    def common_data(self):
        #  print('[PitchYaw] : in common_data')
        self.py_loop_count = loop_counter(self.py_loop_count)
        gpio_blinker(self.color, self.py_loop_count)
        self.get_stereo()
        # self.wait_for_begin
        # self.lateral_speed

        # print("[PitchYaw] : completed 'common_data'")

    def dynamic_drill(self):
        while not self.shutdown_event.isSet() and not self.kill_event.isSet():
            # <<< BEGINNING OF PITCHYAW LOOP __________
            self.common_data()

            try:  # Try for FUT_FINAL_DIST _______________
                try:
                    FUT_FINAL_DIST = self.getfutureDistStack.peek()
                    if FUT_FINAL_DIST is not None:
                        futureDist = True
                    else:
                        futureDist = False  # Try for FINAL_DIST from Launcher(Thread) _______________
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
                    futureDist = False   # Try for FINAL_DIST from Launcher(Thread) _______________
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
                    self.usedDistance = FUT_FINAL_DIST
                    if not self.first_measure:  #
                        self.dist_deque.appendleft(self.usedDistance)

                    if len(self.dist_deque) > self.avg_measure:
                        self.dist_deque.pop()

                    if self.usedDistance > 25.0: self.usedDistance = 25.0

                    self.motor_controller()                         # <<<<<SEND DATA TO MOTORS
                    # time.sleep(0.1)
                #                    self.endtime = time.time() - starttime

                else:
                    if finalDist:
                        self.usedDistance = FINAL_DIST
                    else:
                        self.usedDistance = self.stereoDist

                    self.motor_controller()                         # <<<<<SEND DATA TO MOTORS

                # time.sleep(0.1)
            #                    self.endtime = time.time() - starttime

            except Exception as e:
                print('[PitchYaw] : failed because of exception ' + e)
                continue

        if self.shutdown_event.isSet():
            self.shutdown_reset_function()
            print("[PitchYaw] : STOP BUTTON PRESSED")
            time.sleep(2)
            self.shutdown_event.clear()

        elif self.kill_event.isSet():
            self.shutdown_reset_function()
            print("[PitchYaw] : EXITING...")
            sys.exit()
        else:
            print("[PitchYaw] : Not sure what went wrong")

    def static_manual(self):
        while not self.shutdown_event.isSet() and not self.kill_event.isSet():  # <<< BEGINNING OF PITCHYAW LOOP __________ #

            self.common_data()

            try:
                # Get for FINAL_DIST _______________
                try:
                    FINAL_DIST = self.getfinalDistStack.peek()
                    finalDist = True
                except:
                    finalDist = False
                    pass

                if finalDist:
                    self.usedDistance = FINAL_DIST
                else:
                    self.usedDistance = self.stereoDist

                # _________________ APPEND TO THE DISTANCE DEQUE _________________
                self.dist_deque.appendleft(self.usedDistance)
                if len(self.dist_deque) > self.avg_measure:
                    self.dist_deque.pop()
                # ** ____________________________________________________________________ ** #

                self.motor_controller()                             # <<<<<SEND DATA TO MOTORS

            except Exception as e:
                print('[PitchYaw] : failed because of exception ' + e)
                continue

        if self.shutdown_event.isSet():
            self.shutdown_reset_function()
            time.sleep(2)
            self.shutdown_event.clear()

        elif self.kill_event.isSet():
            self.shutdown_reset_function()
            print("[PitchYaw] : EXITING...")
            sys.exit()
        else:
            print("[PitchYaw] : Not sure what went wrong")

    def run(self):
        print("[PitchYaw] : Starting")

        self.startup()

        if self.drillType == "Dynamic":
            self.dynamic_drill()

        #if self.drillType == "Static" or self.drillType == "Manual":
        else:
            self.static_manual()


class Launcher(Thread):
    '''# _____________________________________LAUNCHER THREAD__________________________________#
Connects and communicates with the Arduino Mega: Launcher Motors, Ball Feeder, Wifi, Accelerometer? (Rangefinder?)
Launcher(Thread): ---> Arduino MEGA provide Angle values to both motors, request temperature's from the Uno, and distance measurements
RECEIVE from MEGA:
- Lidar_2_Distance
- temperature
- voiceCommand
- targetTiming

SEND to MEGA:
- MotorSpeed1
- MotorSpeed2
- targetChoice
- difficulty
- estimated_tof
- targetBallSpeed

Stack IO
RECEIVE from STACKS:
- guiStack
- getfutureDist
- getStereoStack
- getLidar1Stack
- sendfinalDistStack


SEND to STACKS:
- sendMegaDataStack
- sendLidar2Stack
- sendfinalDistStack
- sendTemperatureStack'''
    def __init__(self, getMegaDataStack, send_mega_stack, guiStack, getStereoStack, getLidar1Stack, sendfinalDistStack,
                 getfutureDist, shutdown_event, kill_event, send_flag):
        Thread.__init__(self)
        self.getMegaDataStack = getMegaDataStack
        self.send_mega_stack = send_mega_stack
        self.guiStack = guiStack
        self.getStereoStack = getStereoStack
        self.getLidar1Stack = getLidar1Stack
        self.sendfinalDistStack = sendfinalDistStack
        self.getfutureDist = getfutureDist
        self.shutdown_event = shutdown_event
        self.send_flag = send_flag
        self.kill_event = kill_event
        self.drillSpeed = 1
        self.difficulty = 1
        self.drillType = ""
        self.beginVC = 1
        self.stopVC = 2
        self.fasterVC = 3
        self.slowerVC = 4
        self.pauseVC = 5
        self.drillCount = 1
        self.MEGA = None
        self.OLD_FUT_FINAL_DIST = None
        self.LaunchTime = None
        self.launch_loop_count = 1
        self.stereoData = None
        self.stereo_Distance = 0.0
        self.rationaleDistMeasures = 0
        self.distanceTotal = 0
        self.used_distance = 0.0

        # LAUNCHER OPTIONS:
        self.WAIT_TIME = 0
        self.DYNAMIC_WAIT_TIME = 3  # seconds
        self.STATIC_WAIT_TIME = 5  # seconds
        self.stereo_timeout = 2  # seconds
        self.mega_timeout = 0.5  # seconds
        self.ballfeed = "0"
        # MEGA_DATA Stack values:
        self.voiceCommand = 1  # < set to 1 for testing the launcher (-1 otherwise)
        self.targetTiming = 0.0
        self.targetBallSpeed = 0
        self.startTime = 0.0
        # ____________________

    def drill_wait_time(self):
        if self.LaunchTime is not None:
            while (time.time() - self.LaunchTime) < self.WAIT_TIME:
                print("[LauncherThread] : Holding Loop for:  " + str(
                    self.WAIT_TIME) + "  seconds  Waiting for results from target WIFI")
                time.sleep(.1)

    def wait_for_voice(self):
        while self.voiceCommand != self.beginVC and not self.shutdown_event.isSet() and not self.kill_event.isSet():
            try:
                self.MEGA_DATA = self.getMegaDataStack.peek()
                self.voiceCommand = self.MEGA_DATA.voiceCommand
                self.temperature = self.MEGA_DATA.temperature
                print("VC = " + str(self.voiceCommand) + "  TEMP = " + str(self.temperature))
                time.sleep(0.5)
            except:
                print("[LauncherThread] : Waiting for Voice Command")
                time.sleep(1)
                continue

    def get_stereodata(self):
        # ___________________ RECEIVE STEREO DISTANCE (NO TIMEOUT (slow startup)_________________________________#
        break_loop = False
        while not break_loop and not self.shutdown_event.isSet() and not self.kill_event.isSet():
            try:
                self.stereoData = self.getStereoStack.peek()
                self.stereo_Distance = float(self.stereoData.distance)
                # print("[Launcher(Thread)] : stereo_Distance =  " + str(stereo_Distance))
            except ValueError as verr:
                print("[LauncherThread] : StereoDistance couldnt be converted to float" + str(verr))
                if self.stereoData is None:
                    print("[LauncherThread] : ... because Stack is Empty")
                    continue
                else:
                    self.getStereoStack.pop()
                    print("[LauncherThread] : issue with stereo data, removing it and trying again")
                    continue
            except Exception as e:
                print("[LauncherThread] : Error getting getStereoStack" + str(e))
                time.sleep(0.1)
                continue
            else:
                self.distanceTotal = self.stereo_Distance
                self.rationaleDistMeasures = 1
                break_loop = True

    def get_mega_data(self):
        # _______ RECEIVE MEGA DATA (Wait): lidar_2_Distance, voiceCommand, targetTiming, targetBallSpeed _______#
        try:
            self.MEGA_DATA = self.getMegaDataStack.peek()

            self.lidar_2_Distance = self.MEGA_DATA.lidar_2_Distance  # lidarDistance = int(cm)
            self.temperature = self.MEGA_DATA.temperature  # temperature = int()
            self.voiceCommand = self.MEGA_DATA.voiceCommand  # voice commands = int(from 1 to 5)
            self.targetTiming = self.MEGA_DATA.targetTiming  # targetTiming = float(0.0)
            self.targetBallSpeed = self.MEGA_DATA.targetBallSpeed  # targetBallSpeed

            self.lidar_2_Distance = self.lidar_2_Distance / 100  # << convert LIDAR 2 from cm to meters
            print("<" + str(self.lidar_2_Distance) + "," + str(self.temperature) + "," + str(
                self.voiceCommand) + "," + str(self.targetTiming) + "," + str(self.targetBallSpeed) + ">")
        except serial.SerialException as err:
            print("[LauncherThread] : MEGA not detected" + err)
            print("[LauncherThread] : Closing LauncherThread")
            self.shutdown_event.set()
        except Exception as e:  # (serial.SerialException)
            print("[LauncherThread] : Error in MegaData" + str(e))
            time.sleep(1)

    def send_launch_data(self):
        # In DYNAMIC Mode, the motors spin up before receiving final instructions from MAIN THREAD
        # ____POLYNOMIAL FIT FROM THEORETICAL VALUES___ #
        RPM = -1.13635244 * self.used_distance ** 2.0 + 97.7378699 * self.used_distance + 646.034298  # <-- Polynomial fit
        motorSpeed = round((RPM / 5000) * 255)  # Value between 0-255 (On 24 V: 0-5000 RPM)

        # ____SEND MEGA DATA STRING _____#    IF NO VALUE SEND AS '-

        MotorSpeed = str(motorSpeed)
        # ***Randomize targetChoice
        targetChoice = int(random.choice([1, 2, 3, 4]))
        # CALCULATE THE ESTIMATED TOF:
        estimated_tof = ((0.120617 * self.used_distance)) * 1000  # + difficulty_time

        self.send_data = '<' + MotorSpeed + ',' + MotorSpeed + ',' + str(targetChoice) + ',' + str(
            self.difficulty) + ',' + self.ballfeed + ',' + str(estimated_tof) + '>'
        # _____________________________________________________________________

        # ____________________ Write data to MEGA ____________________
        if (time.time() - self.startTime) <= 5:
            self.send_mega_stack.push(self.send_data)
            time.sleep(0.2)
            self.send_flag.set()

            if self.ballfeed =="1":
                self.drillCount += 1  # << ON SUCCESSFUL LAUNCH
                self.LaunchTime = time.time()

    def launch_motors(self):
        # < ** HANDLE THE MEGA_DATA:
        if self.targetTiming != 0:
            #  print("[LauncherThread] : targetTiming = " + str(targetTiming))
            print("[LauncherThread] : targetTiming = " + str(self.targetTiming))
            #
            #   DO SOMETHING WITH THIS
            #
            targetTiming = 0  # < reset

        if self.targetBallSpeed != 0:
            print("[LauncherThread] : targetBallSpeed = " + str(self.targetBallSpeed))
            #
            #   DO SOMETHING WITH THIS
            #
            targetBallSpeed = 0  # < reset

        # ___________________       Handle Voice input:     ______________________________________________ #

        if self.voiceCommand > 0:
            if self.voiceCommand == self.stopVC:
                self.kill_event.set()
                sys.exit()
            elif self.voiceCommand == self.fasterVC:
                self.drillSpeed += 1
            elif self.voiceCommand == self.slowerVC:
                self.drillSpeed -= 1
            elif self.voiceCommand == self.pauseVC:
                self.shutdown_event.set()

        # ___________________       GARMIN LIDAR:    _______________________________ #

        if self.lidar_2_Distance is not None and abs(self.stereo_Distance - self.lidar_2_Distance) <= 5:
            self.distanceTotal += self.lidar_2_Distance
            self.rationaleDistMeasures += 1
            print("[LauncherThread] : lidar_2_Distance:  " + str(self.lidar_2_Distance))

        # ___________________       EVO LIDAR:    _______________________________ #

        try:
            lidar_1_Distance = self.getLidar1Stack.peek()
            lidar_1_Distance = lidar_1_Distance / 1000  # <<<<<< CONVERT to meters from mm
            if lidar_1_Distance is not None and abs(
                    self.stereo_Distance - lidar_1_Distance) <= 5:  # <<<<<<USE WEIGHTING FACTOR INSTEAD
                self.rationaleDistMeasures += 1
                self.distanceTotal += lidar_1_Distance
                print("[LauncherThread] : lidar_1_Distance:  " + str(lidar_1_Distance))
        except Exception as stackemp:
            print("[LauncherThread] : LIDAR_1 -> nothing in Lidar1Stack" + str(stackemp))
            pass

        # CALCULATE AND SEND TOTAL TO MAIN THREAD
        try:
            FINAL_DIST = self.distanceTotal / self.rationaleDistMeasures
            self.sendfinalDistStack.push(FINAL_DIST)
        except ZeroDivisionError:
            print("[LauncherThread] : ** No VALID distance data **")
              # < Go back to the beginning and try again
        else:
            print("[LauncherThread] : FINAL_DIST:  " + str(FINAL_DIST))

            self.used_distance = FINAL_DIST

            self.send_launch_data()

    def launcher_startup(self):
        startData = False
        while not startData and not self.shutdown_event.isSet() and not self.kill_event.isSet():  # <- Add a timeout to the the start loop

            try:
                # INSTANTIATE THE SERIAL PORT:
                mega_port = findMEGA()
                self.MEGA = serial.Serial(mega_port, 115200, timeout=1)
                self.MEGA.baudrate = 115200

                # START COMMUNICATIONS THREAD
                mega_data_thread = Thread(target=MegaData,
                                          args=[self.MEGA, self.getMegaDataStack, self.shutdown_event, self.kill_event,
                                                self.send_flag, self.send_mega_stack])
                mega_data_thread.start()

            except serial.SerialException as err:
                print("[LauncherThread] : MEGA thread failed to start" + str(err))
                time.sleep(0.5)
                continue
            except Exception as nodata:
                print("[LauncherThread] : ** Mega data thread failed to start:  " + str(nodata) + "**")
                # *****
                self.shutdown_event.set()

                    # Get TEMPERATURE and WAIT for voiceCommand
            else:
                print("[LauncherThread] : Connected to MEGA")
                # Get drillType data from GUI__ #
                guiData = self.guiStack.peek()
                self.drillSpeed = guiData.speed
                self.difficulty = guiData.difficulty
                self.drillType = guiData.drilltype
                # print("LAUNCHER: Speed:  " + str(drillSpeed) + "  Diff:  " + str(difficulty) + "  Type:  " + str(drillType))


                # _____________________WAIT FOR VOICE COMMAND and TEMP TO BEGIN DRILL _____________________
                self.wait_for_voice()

                # ****SET TEMPERATURE CORRECTION FACTOR*****
                self.tempCorrect = 1  # self.temperature / 25
                # *****************************************
                self.startData = True

    def launcher_common(self):
        self.startTime = time.time()
        # Loop Counter:
        self.launch_loop_count = loop_counter(self.launch_loop_count)
        # LPS BLINKER
        gpio_blinker(WHITE, self.self.launch_loop_count)

        self.drill_wait_time()

    def dynamic_drill(self):
        self.WAIT_TIME = self.DYNAMIC_WAIT_TIME
        # ___________________ HOLD THE LAUNCHER SEQUENCE FOR 3-4 SECONDS: ___________________
        while self.drillCount <= 5 and not self.shutdown_event.isSet() and not self.kill_event.isSet():

            self.launcher_common()

            self.get_stereodata()

            try:
                self.get_mega_data()
            except:
                continue
            else:
                self.ballfeed = "0"

                self.launch_motors()

                # _____ GET FUT_FINAL_DIST (No Wait)
                if self.OLD_FUT_FINAL_DIST is not None:
                    self.OLD_FUT_FINAL_DIST = FUT_FINAL_DIST

                try:
                    FUT_FINAL_DIST = self.getfutureDist.peek()  # <<<<< GET PREDICTED LOCATION
                    print(FUT_FINAL_DIST)
                except:
                    FUT_FINAL_DIST = None
                    pass
                else:
                    if FUT_FINAL_DIST == 0.0 or FUT_FINAL_DIST == self.OLD_FUT_FINAL_DIST:  # <- no prediction is done in this thread so it will send AS IS

                        self.ballFeed = 1

                        self.send_launch_data()

                    else:
                        self.used_distance = FUT_FINAL_DIST
                        self.ballFeed = 1

                        self.send_launch_data()

        if self.drillCount == 5:
            print("[LauncherThread] : Drill COMPLETE!")
            self.shutdown_event.set()

        if self.shutdown_event.isSet():
            print("[LauncherThread] : STOP BUTTON PRESSED")
            time.sleep(2)
            self.shutdown_event.clear()

        elif self.kill_event.isSet():
            print("[LauncherThread] : EXITING...")
            sys.exit()

        else:
            print("[LauncherThread] : Not sure what went wrong")

    def static_drill(self):
        self.WAIT_TIME = self.STATIC_WAIT_TIME
        while self.drillCount <= 5 and not self.shutdown_event.isSet() and not self.kill_event.isSet():
            self.launcher_common()

            self.get_stereodata()

            try:
                self.get_mega_data()
            except:
                continue
            else:
                self.ballfeed = "1"

                self.launch_motors()

        if self.drillCount == 5:
            print("[LauncherThread] : Drill COMPLETE!")
            self.shutdown_event.set()

        if self.shutdown_event.isSet():
            print("[LauncherThread] : STOP BUTTON PRESSED")
            time.sleep(2)
            self.shutdown_event.clear()

        elif self.kill_event.isSet():
            print("[LauncherThread] : EXITING...")
            sys.exit()

        else:
            print("[LauncherThread] : Not sure what went wrong")

    def manual_drill(self):
        self.WAIT_TIME = 0 # <<< WAIT TIME IS HANDLED DIFFERENTLY HERE
        while self.drillCount <= 5 and not self.shutdown_event.isSet() and not self.kill_event.isSet():

            # WAIT FOR VC FOR EVERY LAUNCh
            self.wait_for_voice()

            self.launcher_common()

            self.get_stereodata()

            try:
                self.get_mega_data()
            except:
                continue
            else:
                self.ballfeed = "0"

                self.launch_motors()

        if self.drillCount == 5:
            print("[LauncherThread] : Drill COMPLETE!")
            self.shutdown_event.set()

        if self.shutdown_event.isSet():
            print("[LauncherThread] : STOP BUTTON PRESSED")
            time.sleep(2)
            self.shutdown_event.clear()

        elif self.kill_event.isSet():
            print("[LauncherThread] : EXITING...")
            sys.exit()

        else:
            print("[LauncherThread] : Not sure what went wrong")

    def run(self):
        print("[LauncherThread] : starting launcher thread")

        self.launcher_startup()

        try:
            if self.drillType == 'Dynamic':
                # ____________________________ DYNAMIC DRILL _____________________________ #
                self.dynamic_drill()

            if self.drillType == 'Static':
                # ____________________________ STATIC DRILL _____________________________ #
                self.static_drill()

            if self.drillType == 'Manual':
                # ____________________________ MANUAL DRILL _____________________________ #
                self.manual_drill()

        except Exception as e:
            print('[LauncherThread] : failed because of exception: ' + str(e))
            self.shutdown_event.set()


def StereoscopicsThread(stereoStack, shutdown_event, kill_event):
    focalsize = 3.04e-03
    pixelsize = 1.12e-06
    baseline = 0.737

    shutdown_event = shutdown_event
    kill_event = kill_event
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
    frame_width = 304
    frame_height = 224
    framerate = 10
    resolution = (frame_width, frame_height)
    stereo_loop_count = 1
    print("[Stereo] : trying to connected")

    def connectClient():
        connected = False
        while not connected and not shutdown_event.isSet() and not kill_event.isSet():  # Wait for client
            try:
                s.bind((TCP_IP, TCP_PORT))
                s.listen(1)
                clientPort, addr = s.accept()
                connected = True
                print("[Stereo] : Client connected")
                return clientPort
            except socket.error:
                print('[Stereo] :       No Client')
                time.sleep(3)
                continue
            except Exception as e:
                print('[Stereo] :       No Client' + str(e))
                time.sleep(3)
                continue

    def ProcessLoop():  # vs, clientPort, BUFFER_SIZE, frame_width, frame_height, resolution):
        stereo_loop_count = 1
        while not shutdown_event.isSet() and not kill_event.isSet():
            # start_time = time.time()
            if working_on_the_Pi == True:
                if stereo_loop_count % 2 == 0:
                    green.on()
                else:
                    green.off()

            stereo_loop_count += 1
            if stereo_loop_count >= 10:
                stereo_loop_count = 1

            try:
                image = vs.read()
                image = imutils.resize(image, width=frame_width, height=frame_height)
            except AttributeError as e:
                print("[Stereo.ProcessLoop] : no image")
                capture = False
                while not capture and not shutdown_event.isSet() and not kill_event.isSet():
                    image = vs.read()
                    image = imutils.resize(image, width=frame_width, height=frame_height)
                    try:
                        image = vs.read()
                        image = imutils.resize(image, width=frame_width, height=frame_height)
                        capture = True
                    except AttributeError as e:
                        print("[Stereo.ProcessLoop] : no image")
                        capture = False
                        continue
                    except:
                        PiVideoStream().stop()
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
                #                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                centroid = (round((M["m10"] / M["m00"]), 3), round((M["m01"] / M["m00"]), 3))
                centroid = (centroid[0] * 2464 / frame_width, centroid[1] * 2464 / frame_width)
                # only proceed if the radius meets a minimum
            try:
                data = clientPort.recv(BUFFER_SIZE)
                # print("[StereoscopicThread] : received data:", data)
                clientPort.send(data)  # SEND DATA BACK (COULD BE USED FOR STOP COMMAND)
                compvalue = data.decode()
                # print("compvalue:   "+ compvalue)
            except socket.error:
                connected = False
                while not connected and not shutdown_event.isSet() and not kill_event.isSet():
                    try:
                        data = clientPort.recv(BUFFER_SIZE)
                        clientPort.send(data)
                        compvalue = data.decode()
                        # print(compvalue)
                        connected = True
                    except socket.error:
                        print("[Stereo] : Lost the client connection")
                        time.sleep(0.5)
                        continue

            if len(cnts) > 0:
                slaveval = float(compvalue)
                masterval = centroid[0]
                disparity = abs(masterval - slaveval)
                distance = (focalsize * baseline) / (disparity * pixelsize)
                # SENDS DATA TO Class, which can be pushed using Stacks's______________________________
                results = stack_data
                results.distance = distance
                results.disparity = disparity
                results.masterval = float(masterval)
                # print("[Stereo]: Masterval  :  " + str(masterval))
                results.slaveval = slaveval
                stereoStack.push(results)
                # fps = time.time() - start_time
                # print("[Stereo] :   FPS =  " + str(fps) + "  ||    Distance:  " + str(distance))

        if shutdown_event.isSet() or kill_event.isSet():
            #            PiVideoStream().stream.close()
            #            PiVideoStream().rawCapture.close()
            #            PiVideoStream().camera.close()
            # PitchYaw(stereoStack, guiStack, megaDataStack, finalDistStack, futureDistStack,shutdown_event, kill_event).shutdown_reset_function()
            print('[Stereo] : Closing Camera...')
            PiVideoStream().stop()

    class PiVideoStream:
        def __init__(self):  # , resolution = (800, 608), framerate = 32):
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
            while not shutdown_event.isSet() and not kill_event.isSet():
                for f in self.stream:
                    # grab the frame from the stream and clear the stream in
                    # preparation for the next frame
                    self.frame = f.array
                    self.rawCapture.truncate(0)

                    if self.stopped:
                        print('[PiVideoStream] : Closing Camera...')
                        self.stream.close()
                        self.rawCapture.close()
                        self.camera.close()
                        time.sleep(3)
                        return

        def read(self):
            # return the frame most recently read
            return self.frame

        def stop(self):
            # indicate that the thread should be stopped
            self.stopped = True

    if not shutdown_event.isSet() and not kill_event.isSet():
        print("[Stereo] starting THREADED frames from `picamera` module...")
        clientPort = connectClient()
        vs = PiVideoStream().start()
        print("[Stereo] : Initializing camera")
        time.sleep(2.0)  # < Let Video Thread startup
        ProcessLoop()  # vs, clientPort, BUFFER_SIZE, frame_width, frame_height, resolution)


def GetUnoData(UNO, shutdown_event, kill_event):
    getdata = False
    dataPresent = False

    while getdata == False:
        if UNO.is_open:
            while not dataPresent and not shutdown_event.isSet() and not kill_event.isSet():

                UNO.reset_input_buffer()

                try:
                    startMarker = UNO.read().decode()
                except Exception as e:
                    print('[GetUnoData] : didnt get UNO data' + e)
                if startMarker == "<":
                    unoDataStr = UNO.read(15)  # READ DATA FORMATTED AS ('< 1 >')
                    unoDataTemp = list(unoDataStr.decode())
                    unoDataTemp.insert(0, startMarker)
                    unoData = unoDataTemp[:unoDataTemp.index(">") + 1]
                    tempData = "".join(unoData)
                    print("[GetUnoData] : tempData string  ->  ", tempData)
                    dataPresent = True
                    getdata = True
                    return tempData
        else:
            time.sleep(0.1)
            continue
        time.sleep(0.4)  # << MAY NEED TO BE CHANGED IF READ DATA IS GARBAGE


def MegaData(MEGA, sendMegaDataStack, shutdown_event, kill_event, send_flag, send_data):
    # Variables:

    mega_data = stack_data

    while not shutdown_event.isSet() and not kill_event.isSet():
        getdata = False
        if MEGA.is_open:
            try:
                startMarker = MEGA.read().decode()
            except Exception as e:
                print('[MegaDataThread] : ERROR: ' + str(e))
            else:
                if startMarker == "<":
                    megaDataStr = MEGA.read(25)  # READ DATA FORMATTED AS ('< 1 2 3 4 5 >')
                    startTime = time.time()
                    megaDataTemp = list(megaDataStr.decode())
                    megaDataTemp.insert(0, startMarker)
                    megaData = megaDataTemp[:megaDataTemp.index(">") + 1]
                    tempData = "".join(megaData)
                    getdata = True
                    # IF SEND FLAG IS SET, DATA IS SENT AFTER:
                    print("before flag")
                    if send_flag.isSet():
                        print("in if flag")

                        try:
                            data = send_data.pop()
                            print("[MegaDataThread] : " + data)
                            # MEGA.reset_output_buffer()
                            MEGA.write(data.encode())
                            print("[MegaDataThread] : SENT MEGA DATA:  " + send_data)
                        except serial.SerialException as e:
                            print("[MegaDataThread] : ** MEGA SEND FAILED **   " + str(e))
                            while (
                                    time.time() - startTime >= 0.5) and not shutdown_event.isSet() and not kill_event.isSet():
                                try:
                                    #MEGA.reset_output_buffer()
                                    MEGA.write(data.encode())
                                    print("[MegaDataThread] : Launch motors starting...")
                                except serial.SerialException as e:
                                    print("[MegaDataThread] : ** MEGA SEND FAILED **   " + str(e))
                                    if time.time() - startTime >= 0.5:
                                        pass
                                else:
                                    send_flag.clear()
                        else:
                            send_flag.clear()

            finally:
                if getdata == True:
                    try:
                        tempData = tempData.strip("<")
                        tempData = tempData.strip(">")
                        tempData = tempData.split(",")
                        print("[MegaDataThread] : Tempdata=  " + str(tempData))

                        lidar_2_Distance = int(tempData[0])  # lidarDistance = int(cm)
                        temperature = int(tempData[1])  # temperature = int()
                        voiceCommand = int(tempData[2])  # voice commands = int(from 1 to 5)
                        targetTiming = int(tempData[3])  # targetTiming = float(0.0)
                        targetBallSpeed = float(tempData[4])  # targetBallSpeed

                        mega_data.lidar_2_Distance = lidar_2_Distance
                        mega_data.temperature = temperature
                        mega_data.voiceCommand = voiceCommand
                        mega_data.targetTiming = targetTiming
                        mega_data.targetBallSpeed = targetBallSpeed

                        sendMegaDataStack.push(mega_data)
                    except Exception as e:
                        print("[MegaDataThread] : Parsing Data:  " + str(e))
        else:
            print("[MegaDataThread] : ** Mega didnt respond  **")
            time.sleep(0.01)
            continue
        # time.sleep(0.4)


def findEvo():
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        # print(p)          # This causes each port's information to be printed out.
        if "5740" in p[2]:
            return (p[0])
    return ('NULL')


def findUNO():
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        # print(p)           # This causes each port's information to be printed out.
        if "SER=85734323331351208080" in p[2]:
            return (p[0])
    return ('NULL')


def findMEGA():
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        # print(p)            # This causes each port's information to be printed out.
        if "0042" in p[2]:
            return (p[0])
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
#   stereoStack
#
# SEND to STACKS:
#   futureDistStack
#
#
def get_stereodata():
    print("dostuff")

def startMainFile(speed, difficulty, drillType, shutdown_event, kill_event, send_flag, PitchYawStart, LauncherStart, EvoStart):  # , args): ## NOT A THREAD, performs the bulk of calculation
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
        guiData = stack_data
        guiData.speed = speed
        guiData.difficulty = difficulty
        guiData.drilltype = drillType
        FUT_FINAL_DIST = 0.0

        StartPitchYaw = PitchYawStart
        StartLauncher = LauncherStart
        EvoLidar = EvoStart
        loop_count = 1

        RED_1.off()
        RED_2.off()
        RED_3.off()

        print("[MainThread] : ")
        print("Speed:  " + str(guiData.speed) + "  " + "Diff:  " + str(
            guiData.difficulty) + "  " + "Drill:  " + guiData.drilltype)
        print("_______________________________________________")

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
                if working_on_the_Pi == True:
                    RED_1.on()
            except Exception as e:
                print('[MainThread] : Stereo thread failed because of exception ' + str(e))

        if StartPitchYaw:
            try:
                pitchYawthread = PitchYaw(stereoStack, guiStack, megaDataStack, finalDistStack,
                                          futureDistStack, shutdown_event, kill_event)
                pitchYawthread.start()
                if working_on_the_Pi == True:
                    RED_2.on()
            except Exception as e:
                print('[MainThread] : pitchYawthread didnt start because of exception ' + str(e))
        else:
            print('[MainThread] : pitchYawthread thread is not starting')

        if StartLauncher:
            try:
                startLauncherThread = Launcher(megaDataStack, send_mega_stack, guiStack, stereoStack, lidar1Stack,
                                               finalDistStack,
                                               futureDistStack, shutdown_event, kill_event, send_flag)
                startLauncherThread.start()
                if working_on_the_Pi == True:
                    RED_3.on()
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
            if loop_count % 2 == 0:
                blue.on()

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
                        time.sleep(1)
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

            if 1 <= stereo_Distance <= 35:  # Meters
                rationaleDistMeasures = 1
                distanceTotal = stereo_Distance

                # Get(NO_WAIT) for Lidar_1_Dist _____________________________
                if EvoLidar:
                    try:
                        LIDAR_1_Distance = Lidar1Dist(evo)
                        evo.flushOutput()
                        time.sleep(0.05)
                        if LIDAR_1_Distance is not None and abs(
                                stereo_Distance - LIDAR_1_Distance) <= 5:  # <<<<<<USE WEIGHTING FACTOR INSTEAD
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
                    if LIDAR_2_Distance is not None and abs(
                            stereo_Distance - LIDAR_2_Distance) <= 5:  # <<<<<<USE WEIGHTING FACTOR INSTEAD
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
                    # FUT_FINAL_DIST = None
                elif len(z_dist_deque) < avg_measures:
                    z_dist_deque.appendleft(PRE_FINAL_DIST)
                    # FUT_FINAL_DIST = None
                elif len(
                        z_dist_deque) == avg_measures:  # Wait until we have 10 measurement before calculating players speed (if 5 FPS this means 2 sec)
                    temp_dist = z_dist_deque[0] - z_dist_deque[avg_measures - 1]
                    temp_time = sum([elem for elem in measure_time_deque])  # Time for avg_measure measurements

                    playerspeed = temp_dist / temp_time  # meters/second
                    # the idea is to stay ahead of the player by at least a second or two
                    FUT_FINAL_DIST = PRE_FINAL_DIST + playerspeed * lead_time
                    z_dist_deque.appendleft(PRE_FINAL_DIST)
                    z_dist_deque.pop()

                futureDistStack.push(FUT_FINAL_DIST)
                loop_count += 1
                if loop_count == 100:
                    print("[MainThread] : FUT_FINAL_DIST =   " + str(FUT_FINAL_DIST))
                    loop_count = 1

            else:
                print("[MainThread] : Player is not in Range")
                # Activate GPIO pin for notification LED
                # ***Do something about this***
                #
                #
                #
                time.sleep(0.2)

        elif drillType == "Static":
            print("[MainThread] : Main doesnt do much here")
            time.sleep(1)

        elif drillType == "Manual":
            print("[MainThread] : Main doesnt do much here")
            time.sleep(1)
        else:
            print("[MainThread] : no GUI data")
            time.sleep(1)

    if shutdown_event.isSet():
        # PitchYaw(stereoStack, guiStack, megaDataStack, finalDistStack, futureDistStack, shutdown_event, kill_event).shutdown_reset_function()
        print("[MainThread] : STOP BUTTON PRESSED")
        time.sleep(2)
    elif kill_event.isSet():
        # PitchYaw(stereoStack, guiStack, megaDataStack, finalDistStack, futureDistStack, shutdown_event, kill_event).shutdown_reset_function()
        print("[MainThread] : EXITING...")
        sys.exit()
    else:
        print("[MainThread] : Not sure what went wrong")
