import serial
import serial.tools.list_ports
import PID
import sys
from collections import deque
import numpy as np
import time
import multiprocessing as mp

# ** __ PID VARIABLES: __ ** #
try:
    P = 0.3
    I = 0.0
    D = 0.0
    pid = PID.PID(P, I, D)
    pid.SetPoint = 0.0
    pid.setSampleTime(0.15)

except Exception as e:
    print("[PitchYaw] : PID Cant set Attribute  :  ", e)
    pass

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

class data_object:
    def __init__(self, *args):
        self.args = args


def gpio_blinker(color, loop_count, Pi):
    if Pi:
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


def findUNO():
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        # print(p)           # This causes each port's information to be printed out.
        if "SER=85734323331351208080" in p[2]:
            return (p[0])
    return ('NULL')


def GetUnoData(UNO):#, shutdown_event, kill_event):
    getdata = False
    dataPresent = False

    while getdata == False:
        if UNO.is_open:
            while not dataPresent: #and not shutdown_event.isSet() and not kill_event.isSet():

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



class PitchYaw(mp.Process):
    '''PITCH_YAW_PROCESS: ---> Arduino UNO provide Angle values to both motors, request temperature's from the Uno, and distance measurements
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
- launcherAngle -->  launcherAngleStack --> Launcher
'''

    def __init__(self, guiData, stereo_data, stereo_py_main, final_dist_py,
                 future_dist_py, pi_no_pi):
        super(PitchYaw, self).__init__()
        self.guiData = guiData
        self.stereo_data = stereo_data
        self.stereo_py_main = stereo_py_main
        self.final_dist_py = final_dist_py
        self.future_dist_py = future_dist_py
        self.working_on_the_Pi = pi_no_pi
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

        if self.working_on_the_Pi:
            from gpiozero import LED
            YELLOW = LED(19)
            self.color = YELLOW

        self.usedDistance = 0.0
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
        # if self.shutdown_event.isSet():
        print("[PitchYaw] : Resetting Motors")
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

        pid.update(self.latPixelDisp)
        pid_output = pid.output  # MAXIMUM OUTPUT IS ROUGHLY: 400
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
        while not cameradata: # and not self.shutdown_event.isSet() and not self.kill_event.isSet():
            try:
                self.stereodata = self.stereo_data.get()
                self.stereo_py_main.put(self.stereodata)
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

        # MEGAdata = self.!!ChangetoJustVCQueue!!.peek()
        # voiceCommand = MEGAdata.voicecommand
        #
        # while voiceCommand != "beginVC" and not self.shutdown_event.isSet() and not self.kill_event.isSet():
        #     try:
        #         MEGAdata = self.!!ChangetoJustVCQueue!!.peek()
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
        while not self.startData:# and not self.shutdown_event.isSet() and not self.kill_event.isSet():


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

                guiData = self.guiData
                self.drillType = guiData.drilltype
                # difficulty = guiData.difficulty
                # drillSpeed = guiData.speed

            except Exception as err:
                print('[PitchYaw] : Arduino UNO not available' + str(err))
                time.sleep(2)
                continue
            else:
                print("[PitchYaw] : Received start data")
                # if self.shutdown_event.isSet():
                #     print("[PitchYaw] : STOP BUTTON PRESSED")
                #     break
                # else:
                print("[PitchYaw] : UNO Connected... starting loop")
                self.startData = True

        # print("[PitchYaw] : Starting")
        time.sleep(5)

    def common_data(self):
        #  print('[PitchYaw] : in common_data')
        gpio_blinker(self.color, self.py_loop_count, self.working_on_the_Pi)
        self.py_loop_count = loop_counter(self.py_loop_count)
        self.get_stereo()
        # self.wait_for_begin
        # self.lateral_speed

        # print("[PitchYaw] : completed 'common_data'")

    def dynamic_drill(self):
        while True: #not self.shutdown_event.isSet() and not self.kill_event.isSet():
            # <<< BEGINNING OF PITCHYAW LOOP __________
            self.common_data()

            try:  # Try for FUT_FINAL_DIST _______________
                try:
                    FUT_FINAL_DIST = self.future_dist_py.get()
                    if FUT_FINAL_DIST is not None:
                        futureDist = True
                    else:
                        futureDist = False  # Try for FINAL_DIST from Launcher(Thread) _______________
                        try:
                            FINAL_DIST = self.final_dist_py.get()
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
                        FINAL_DIST = self.final_dist_py.peek()
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



    def static_manual(self):
        while True: #not self.shutdown_event.isSet() and not self.kill_event.isSet():  # <<< BEGINNING OF PITCHYAW LOOP __________ #

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


    def run(self):
        print("[PitchYaw] : Starting")

        self.startup()

        if self.drillType == "Dynamic":
            self.dynamic_drill()

        #if self.drillType == "Static" or self.drillType == "Manual":
        else:
            self.static_manual()
