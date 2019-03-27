import serial
import serial.tools.list_ports
from collections import deque
from threading import Event
import numpy as np
import time
import multiprocessing as mp
from gpiozero import LED

startMarker = ""


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


def findUNO():
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        if "0043" in p[2]:
            return (p[0])
    return ('NULL')


class PitchYaw(mp.Process):
    '''PITCH_YAW_PROCESS: ---> Arduino UNO provide Angle values to both motors,
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

    def __init__(self, guiData, get_stereo_data, stereo_py_main, final_dist_py,
                 future_dist_py, pi_no_pi, led_color, kill_event, py_reset_event, pause_event, stereo_data_flag,
                 pymain_stereo_flag, launch_event):
        super(PitchYaw, self).__init__()
        self.guiData = guiData
        self.get_stereo_data = get_stereo_data
        self.stereoData = "0,0,0.0"
        self.stereo_py_main = stereo_py_main
        self.final_dist_py = final_dist_py
        self.future_dist_py = future_dist_py
        self.working_on_the_Pi = pi_no_pi
        self.kill_event = kill_event
        self.py_reset_event = py_reset_event
        self.pause_event = pause_event
        self.stereo_data_flag = stereo_data_flag
        self.pymain_stereo_flag = pymain_stereo_flag
        self.launch_event = launch_event
        # ** __ RECORD A LIST OF OLD MEASUREMENTS FOR TRACKING: __ ** #
        self.dist_deque = deque([])
        self.x_disp_deque = deque([])
        self.measure_time_deque = deque([])

        # ** __ DATA TRACKERS: __ ** #
        self.startData = False
        self.last_start_time = None

        # ** __ IMPORTANT VARIABLES: __ ** #
        self.avg_measure = 10
        self.Uno_write_lock = Event()
        self.LEAD_SPEED = 25  # << Adjust the added motorSpeed of the YAW for DYNAMIC MODE (0-255)
        self.MAX_YAW_SPEED = 0.15  # << Maximum speed of Yaw in radians

        # if self.working_on_the_Pi:
        self.color = led_color

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
        self.futureDist = False
        # STEREOSCOPIC CALC:
        self.focalsize = 3.04e-03
        self.pixelsize = 1.12e-06
        self.baseline = 0.737

        # **** DATA SMOOTHING PARAMETERS: *********** #
        self.max_measures = 20
        self.first_new = False
        self.distances = deque([])
        self.pixel_disps = deque([])
        self.predictions = 0

        self.replacements = 0

        self.pitchAngleTable = np.array([[7.5, 5],  # << Pitch angle lookup table based on estimations
                                         [12.5, 12],
                                         [17.5, 20],
                                         [22.5, 30],
                                         [25, 37]])

    def shutdown_reset_function(self):
        while not self.py_reset_event.is_set():
            time.sleep(0.1)

        data = '<5000,0>'
        print("-----------------")
        print("[PitchYaw] : Resetting Motors")
        print("sending reset data:  ", data)
        print("-----------------")

        time.sleep(1)
        self.Uno_write_lock.set()
        try:
            self.UNO.write(data.encode())
        except:
            print("error sending reset")
        else:
            self.py_reset_event.clear()

    def motor_controller(self):
        #        start_time1 = time.time()
        #   ______________________________ PITCH SMOOTHING  ______________________________ #
        new_distance = self.usedDistance
        # print("[PitchYaw]:", new_distance)
        if not self.futureDist:
            if len(self.distances) == self.max_measures:
                mov_dist_avgs = np.convolve(self.distances, np.ones((5)) / 5, mode='valid')

                if abs(new_distance - mov_dist_avgs[15]) <= 2.0:
                    self.replacements = 0
                    self.distances.append(new_distance)
                #                print("stereo ",self.distances)
                else:
                    self.replacements += 1
                    if self.replacements <= 10:
                        slope1 = mov_dist_avgs[14] - mov_dist_avgs[13]
                        slope2 = mov_dist_avgs[15] - mov_dist_avgs[14]
                        avg_change = (slope1 + slope2) / 2
                        new_distance = float(round(self.distances[19] + avg_change, 2))
                        # print(new_distance)
                        self.distances.append(new_distance)
                        print("replaced ", self.distances)
                    # else:
                    #     distances = deque([])
                    #     new_distance = stereo_Distance
                    #     distances.append(new_distance)
                self.distances.popleft()
            else:
                self.distances.append(new_distance)
                print("populating... ", self.distances)

        # ** _________________ PITCH ANGLE: __________________ ** #
        if new_distance <= 7.5:
            row = 0
            pitchAngle = self.pitchAngleTable[row, 1]  # + self.launcherAngle
        elif 7.5 < new_distance <= 12.5:
            row = 1
            pitchAngle = self.pitchAngleTable[row, 1]
        elif 12.5 < new_distance <= 17.5:
            row = 2
            pitchAngle = self.pitchAngleTable[row, 1]
        elif 17.5 < new_distance <= 22.5:
            row = 3
            pitchAngle = self.pitchAngleTable[row, 1]
        elif 22.5 < new_distance <= 35:
            row = 4
            pitchAngle = self.pitchAngleTable[row, 1]
        else:
            pitchAngle = 10

        #   ______________________________ YAW SMOOTHING/TRACKING  ______________________________ #

        self.latPixelDisp = (3280 - self.LeftXcoord - self.RightXcoord)

        new_pix = self.latPixelDisp

        if len(self.pixel_disps) == self.max_measures:
            mov_pix_avgs = np.convolve(self.pixel_disps, np.ones((5)) / 5, mode='valid')

            if abs(new_pix - mov_pix_avgs[15]) <= 2000:  # << REALLY LARGE SO IT WONT REALLY AFFECT ANYTHING YET
                self.pix_replace = 0
                self.pixel_disps.append(new_pix)
            #                print("stereo ",self.distances)
            else:
                self.pix_replace += 1
                if self.pix_replace <= 10:
                    p_slope1 = mov_pix_avgs[15] - mov_pix_avgs[14]
                    p_slope2 = mov_pix_avgs[14] - mov_pix_avgs[13]
                    avg_pix_change = (p_slope1 + p_slope2) / 2
                    new_pix = float(round(self.pixel_disps[19] + avg_pix_change, 2))
                    # print(new_distance)
                    self.pixel_disps.append(new_pix)
                    print("replaced pixel disp: ", self.pixel_disps)
                # else:
                #     distances = deque([])
                #     new_distance = stereo_Distance
                #     distances.append(new_distance)
            self.pixel_disps.popleft()
        else:
            self.pixel_disps.append(new_pix)
            print("populating pixel disp: ... ", self.pixel_disps)

        # ** ___ SEND DATA ___ ** #
        if self.drillType == "Dynamic":
            drill_type = "1"

            ## "PREDICTION" FOR DYNAMIC
            if self.launch_event.is_set():
                if self.predictions <= 5:
                    avg_displacement = sum(self.pixel_disps[9:19]) / 10
                    if avg_displacement < 0:
                        new_pix = - 3000
                    elif avg_displacement > 0:
                        new_pix = 3000
                    self.predictions += 1
                else:
                    self.launch_event.clear()
                    self.predictions = 0
        else:
            drill_type = "0"

        data = '<' + str(new_pix) + ', ' + str(pitchAngle) + ',' + drill_type + '>'

        #        print("[PitchYaw]: UNO Data =  ",data)
        if not self.Uno_write_lock.is_set():
            self.UNO.write(data.encode())

    #        if self.py_loop_count == 10:
    #            print("[PitchYaw:Future] SENT: <pixelDisp, pitchAngle> =  " + data)

    def get_stereo(self):
        cameradata = False
        while not cameradata and not self.kill_event.is_set() and not self.pause_event.is_set():
            try:
                # start_time2 = time.time()
                try:
                    self.stereo_data_flag.set()  # <<<<< LETS STEREO KNOW TO ADD DATA TO QUEUE
                    self.stereoData = self.get_stereo_data.get(timeout=1.5)
                    # time1 = (time.time() - start_time2)
                #                    if time1 > 0.15:
                #                    print("[get_stereo]: 'get' from Queue: ",round(time1,2))
                except Exception as e:
                    print("[PitchYaw]: getting stereo timed out (1s)", e)
                    # STEREO TIMEOUT MEANS PLAYER IS NOT IN VEIW, THERFORE WE NEED TO SCAN THE FOV
                    if self.drillType == "Dynamic":
                        data = '<' + "-1" + ', ' + "10" + ',1>'
                    else:
                        data = '<' + "-1" + ', ' + "10" + ',0>'
                    if not self.Uno_write_lock.is_set():
                        self.UNO.write(data.encode())
                    continue
                else:
                    tempData = "".join(self.stereoData)
                    tempData = tempData.strip("<")
                    tempData = tempData.strip(">")
                    tempData = tempData.split(",")
                    self.RightXcoord = int(float(tempData[0]))
                    self.LeftXcoord = int(float(tempData[1]))
                    self.disparity = abs(self.LeftXcoord - self.RightXcoord)
                    if self.disparity == 0:
                        self.disparity = 1
                    self.stereoDist = round((self.focalsize * self.baseline) / (self.disparity * self.pixelsize), 2)
                    # print("Left: ", self.LeftXcoord, " Right: ", self.RightXcoord," Dist: ", self.stereoDist)
                    # print("[PitchYaw]:  Distance  = ", self.stereoDist)
                    if self.pymain_stereo_flag.is_set():
                        self.stereo_py_main.put(tempData)
                        self.pymain_stereo_flag.clear()
                    cameradata = True
            except ValueError as verr:
                print("[PitchYaw] : StereoData couldnt be converted" + str(verr))
                if self.stereoData is None:
                    print("[PitchYaw] : ... because Stack is Empty")
                    continue
                else:
                    bad_val = self.get_stereo_data.get_nowait()
                    print("[PitchYaw] : issue with stereo data, removing it and trying again")
                continue
            except Exception as e:
                print("[PitchYaw] : Error getting getStereoStack" + str(e))
                # time.sleep(2)
                continue

        if self.kill_event.is_set():
            self.shutdown_reset_function()

        if self.pause_event.is_set():
            print("[Launcher] : Paused Drill")
            while self.pause_event.is_set():
                time.sleep(1)

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
        while not self.startData and not self.kill_event.is_set():  # and not self.shutdown_event.is_set() and not self.kill_event.is_set():
            if self.pause_event.is_set():
                print("[Launcher] : Paused Drill")
                while self.pause_event.is_set():
                    time.sleep(1)

            try:
                # ___________ Initialize Arduino UNO ___________
                uno_port = findUNO()
                self.UNO = serial.Serial(uno_port, 115200, timeout=1)  # change ACM number as found from "ls /dev/tty*"
                self.UNO.baudrate = 115200

                # ___________ Get TEMPERATURE Data _______________
                # temperature = self.getTemperatureStack.peek()
                self.tempCorrection = 1  # temperature / 25  # <<<tempCorrection factor

                # ___________ Get GUI Data _______________
                guiData = self.guiData
                self.drillType = guiData.drilltype

            except Exception as err:
                print('[PitchYaw] : Arduino UNO not available' + str(err))
                time.sleep(2)
                continue
            else:
                print("[PitchYaw] : UNO Connected... starting loop")
                self.startData = True

        if self.kill_event.is_set():
            self.shutdown_reset_function()
        # WAIT TO ENSURE THAT ALL THE PROCESSES ARE STARTED
        time.sleep(5)

    def common_data(self):
        self.start_time = time.time()
        # if self.pause_event.is_set():
        #     print("[Launcher] : Paused Drill")
        #     while self.pause_event.is_set():
        #         time.sleep(1)
        gpio_blinker(self.color, self.py_loop_count, self.working_on_the_Pi)
        self.py_loop_count = loop_counter(self.py_loop_count)

        self.get_stereo()

    def dynamic_drill(self):
        self.futureDist = False
        finalDist = False
        while not self.kill_event.is_set():
            # start_time = time.time()
            self.common_data()
            try:  # Try for FUT_FINAL_DIST _______________
                try:
                    FUT_FINAL_DIST = self.future_dist_py.get_nowait()
                    if FUT_FINAL_DIST is not None:
                        self.futureDist = True
                    else:
                        self.futureDist = False
                # Try for FINAL_DIST from Launcher(Thread) _______________
                except:
                    self.futureDist = False
                    FUT_FINAL_DIST = None  # <<<<<<<<<ENSURE THIS ALWAYS GETS A NEW VALUE NEXT ITERATION
                    try:
                        FINAL_DIST = self.final_dist_py.get_nowait()
                        if FINAL_DIST is not None:
                            finalDist = True
                        else:
                            finalDist = False
                    except:
                        finalDist = False
                        FINAL_DIST = None  # <<<<<<<<<ENSURE THIS ALWAYS GETS A NEW VALUE NEXT ITERATION
                        pass

                # **________ TWO POSSIBLE CASES: FUTURE_DIST IS AVAILABLE OR NOT ________** #

                if self.futureDist:  # << CASE 1: Only have to 'predict'/anticipate future lateral displacement _________
                    self.usedDistance = FUT_FINAL_DIST
                    if self.usedDistance > 25.0:
                        self.usedDistance = 25.0
                    self.motor_controller()
                elif finalDist:
                    self.usedDistance = FINAL_DIST
                    self.motor_controller()
                else:
                    try:
                        self.usedDistance = self.stereoDist
                        # FPS = time.time() - start_time
                        self.motor_controller()
                    except Exception as e:
                        print(e)
                        self.shutdown_reset_function()

            except Exception as e:
                print('[PitchYaw] : exception ', e)
                continue
        #            else:
        #                print("[drill]:  ",1/(time.time() - start_time3))

        if self.kill_event.is_set():
            self.shutdown_reset_function()

    def static_manual(self):
        while not self.kill_event.is_set():  # not self.shutdown_event.is_set() and not self.kill_event.is_set():  # <<< BEGINNING OF PITCHYAW LOOP __________ #

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

                self.motor_controller()  # <<<<<SEND DATA TO MOTORS

            except Exception as e:
                print('[PitchYaw] : failed because of exception ', e)
                continue

    def run(self):
        print("[PitchYaw] : Starting ... ")

        self.startup()

        if self.kill_event.is_set():
            self.shutdown_reset_function()

        if self.drillType == "Dynamic":
            self.dynamic_drill()

        # if self.drillType == "Static" or self.drillType == "Manual":
        else:
            self.static_manual()