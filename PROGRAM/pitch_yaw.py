import serial
import serial.tools.list_ports
import PID
from collections import deque
from threading import Event
import numpy as np
import time
import multiprocessing as mp
from gpiozero import LED

# ** __ PID VARIABLES: __ ** #
try:
    P = 0.4
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
        if "0043" in p[2]:
            return (p[0])
    return ('NULL')


def GetUnoData(UNO):#, shutdown_event, kill_event):
    getdata = False
    dataPresent = False

    while getdata == False:
        if UNO.is_open:
            while not dataPresent: #and not shutdown_event.is_set() and not kill_event.is_set():

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
                 future_dist_py, pi_no_pi, led_color, kill_event):
        super(PitchYaw, self).__init__()
        self.guiData = guiData
        self.stereo_data = stereo_data
        self.stereoData = "0,0,0.0"
        self.stereo_py_main = stereo_py_main
        self.final_dist_py = final_dist_py
        self.future_dist_py = future_dist_py
        self.working_on_the_Pi = pi_no_pi
        self.kill_event = kill_event
        #self.stereodata = data_object
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

        #if self.working_on_the_Pi:
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

        self.pitchAngleTable = np.array([[5, 15],  # << Pitch angle lookup table based on estimations
                                         [7, 16],
                                         [9, 17],
                                         [11, 18],
                                         [13, 19],
                                         [15, 20],
                                         [17, 21],
                                         [19, 22],
                                         [21, 23],
                                         [23, 24],
                                         [25, 25]])






    def shutdown_reset_function(self):
        # if self.shutdown_event.is_set():
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
        elif row > 10:
            row = 10
        pitchAngle = self.pitchAngleTable[row, 1] + self.launcherAngle

# ''' PID CALCULATION:(scaled_pid_output_=0-1) (AT 25m: 1*1*255 = 255, AT 5m: 1*0.2 = 50)'''

        pid.update(self.latPixelDisp)
        pid_output = pid.output  # MAXIMUM OUTPUT IS ROUGHLY: 400
        scaled_pid = int((pid_output / 280) * 255)

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
        #self.UNO.reset_output_buffer()
        data = '<' + str(motorSpeed) + ', ' + str(pitchAngle) + '>'
        self.UNO.write(data.encode())
        if self.py_loop_count == 10:
            print("[PitchYaw:Future] SENT: <motorSpeed, pitchAngle> =  " + data)


    def get_stereo(self):
        # ___________________ GET stereoStack DATA ___________________ #
        cameradata = False
        while not cameradata and not self.kill_event.is_set(): # and not self.shutdown_event.is_set() and not self.kill_event.is_set():
            try:
                self.stereoData = self.stereo_data.get()
                tempData = "".join(self.stereoData)
#                tempData = tempData.strip("<")
#                tempData = tempData.strip(">")
                tempData = tempData.split(",")
                self.RightXcoord = int(float(tempData[0]))  
                self.LeftXcoord = int(float(tempData[1]))  
                self.stereoDist = float(tempData[2])
                #print(self.stereoDist)
                self.stereo_py_main.put(tempData)
                cameradata = True
            except ValueError as verr:
                print("[PitchYaw] : StereoData couldnt be converted" + str(verr))
                if stereoData is None:
                    print("[PitchYaw] : ... because Stack is Empty")
                    continue
                else:
                    bad_val = self.stereo_data.get_nowait()
                    print("[PitchYaw] : issue with stereo data, removing it and trying again")
                continue
            except Exception as e:
                print("[PitchYaw] : Error getting getStereoStack" + str(e))
                # time.sleep(2)
                continue
            else:
                # ** ___________________ YAW MOTOR SPEED: ______________________ ** #
                self.latPixelDisp = (2464 - self.LeftXcoord - self.RightXcoord)
                #print("[PitchYaw] : ", self.stereoDist)

    # def wait_for_begin(self):
        # _____________ Get MEGA Data _____________

        # MEGAdata = self.!!ChangetoJustVCQueue!!.peek()
        # voiceCommand = MEGAdata.voicecommand
        #
        # while voiceCommand != "beginVC" and not self.shutdown_event.is_set() and not self.kill_event.is_set():
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
        while not self.startData and not self.kill_event.is_set(): # and not self.shutdown_event.is_set() and not self.kill_event.is_set():
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
                #print("[PitchYaw] : Received start data")
                # if self.shutdown_event.is_set():
                #     print("[PitchYaw] : STOP BUTTON PRESSED")
                #     break
                # else:
                print("[PitchYaw] : UNO Connected... starting loop")
                self.startData = True

        # print("[PitchYaw] : Starting")
        time.sleep(5)

    def common_data(self):
        start_comm = time.time()
        
        #  print('[PitchYaw] : in common_data')
        gpio_blinker(self.color, self.py_loop_count, self.working_on_the_Pi)
        self.py_loop_count = loop_counter(self.py_loop_count)
        self.get_stereo()
        # self.wait_for_begin
        # self.lateral_speed
#        FPS = time.time() - start_comm
#        print("[PitchYaw] ; common data FPS ", FPS)
        return
        # print("[PitchYaw] : completed 'common_data'")

    def dynamic_drill(self):
        futureDist = False
        while True and not self.kill_event.is_set(): #not self.shutdown_event.is_set() and not self.kill_event.is_set():
            #start_time = time.time()
            # <<< BEGINNING OF PITCHYAW LOOP __________
            self.common_data()
            # print('[PitchYaw] : done common_data')
            try:  # Try for FUT_FINAL_DIST _______________
                try:
                    FPS = time.time() - start_time
#                    print("[PITCHYAW] : Before the GET", FPS)
                    FUT_FINAL_DIST = self.future_dist_py.get(timeout=0.1)
                    if FUT_FINAL_DIST is not None:
                        #print('[PitchYaw] : got future dist')
                        #FPS = time.time() - start_time
#                        print("[PITCHYAW] : After the GET", FPS)
                        futureDist = True
                    else:
                        futureDist = False  # Try for FINAL_DIST from Launcher(Thread) _______________

                except:
                    futureDist = False               
                    try:
                        FINAL_DIST = self.final_dist_py.get(timeout=0.1)
                        if FINAL_DIST is not None:
                            finalDist = True
#                            print('[PitchYaw] : got final dist')
                        else:
                            finalDist = False
                    except:
                        finalDist = False
#                        print('[PitchYaw] : using stereo dist')
                        pass
                    
#                FPS = time.time() - start_time
#                print("[PITCHYAW] : After the TRYTRY", FPS)
                #except:
                 #   print('[PitchYaw] : using stereo dist')

#                    futureDist = False   # Try for FINAL_DIST from Launcher(Thread) _______________
#                    try:
#                        FINAL_DIST = self.final_dist_py.peek()
#                        if FINAL_DIST is not None:
#                            finalDist = True
#                        else:
#                            finalDist = False
#                    except:
#                        finalDist = False
                    # pass

                # **________ TWO POSSIBLE CASES: FUTURE_DIST IS AVAILABLE OR NOT ________** #

                if futureDist:
                    # << CASE 1: Only have to 'predict'/anticipate future lateral displacement _________
                    self.usedDistance = FUT_FINAL_DIST
#                    if not self.first_measure:  #
#                        self.dist_deque.appendleft(self.usedDistance)
#
#                    if len(self.dist_deque) > self.avg_measure:
#                        self.dist_deque.pop()

                    if self.usedDistance > 25.0:
                        self.usedDistance = 25.0
                    FPS = time.time() - start_time
#                    print("[PitchYaw] : drill b4 motor_controll ", FPS)
                    self.motor_controller()
                    #FPS = time.time() - start_time
                    #print("[PitchYaw] : drill after motor_controll ", FPS)                         # <<<<<SEND DATA TO MOTORS
                    # time.sleep(0.1)
                #                    self.endtime = time.time() - starttime

                else:
                    if finalDist:
                        self.usedDistance = FINAL_DIST
                    else:
                        self.usedDistance = self.stereoDist
                    #FPS = time.time() - start_time
#                    print("[PitchYaw] : drill b4 motor_controll ", FPS)
                    self.motor_controller()
                    #FPS = time.time() - start_time
#                    print("[PitchYaw] : drill after motor_controll ", FPS) # <<<<<SEND DATA TO MOTORS

                # time.sleep(0.1)
            #                    self.endtime = time.time() - starttime

            except Exception as e:
                print('[PitchYaw] : failed because of exception ' + e)
                continue



    def static_manual(self):
        while not self.kill_event.is_set():#not self.shutdown_event.is_set() and not self.kill_event.is_set():  # <<< BEGINNING OF PITCHYAW LOOP __________ #

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

if __name__ == "__main__":

    import stereo
    from collections import deque

    def findEvo():
        ports = list(serial.tools.list_ports.comports())
        for p in ports:
            # print(p)          # This causes each port's information to be printed out.
            if "5740" in p[2]:
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
                        print("[MainProcess/Lidar1Dist] : error converting to float", e)
                        return None
            except serial.SerialException as a:
                print("[MainProcess/Lidar1Dist] : No Evo Lidar present... connect it and restart the application" + a)
                return None


    working_on_the_Pi = False
    if working_on_the_Pi:
        YELLOW = LED(26)
        RED_1 = LED(13)
        GREEN = LED(16)
        BLUE = LED(20)

    stereo_Distance = 0.0
    avg_measures = 10
    lead_time = 3
    # TRACKING LISTS
    z_dist_deque = deque([])
    measure_time_deque = deque([])

    mega_data = mp.Queue()
    future_dist_py = mp.Queue()
    final_dist_py = mp.Queue()
    stereo_data = mp.Queue()
    stereo_py_main = mp.Queue()

    kill_event=Event()
    shutdown_event = Event()
    # OPTIONS:
    speed = 1
    difficulty = 1
    drillType = "Dynamic"
    EvoLidar = False


    guiData = data_object
    guiData.speed = speed
    guiData.difficulty = difficulty
    guiData.drilltype = drillType

    print("[MainProcess] : ")
    print("Speed:  " + str(guiData.speed) + "  " + "Diff:  " + str(
        guiData.difficulty) + "  " + "Drill:  " + guiData.drilltype)
    print("_______________________________________________")

    # ___ OPEN SERIAL PORT/S ___ #
    if EvoLidar:
        evo_data = False

        while not evo_data and not shutdown_event.is_set() and not kill_event.is_set():
            try:
                port = findEvo()
                evo = serial.Serial(port, baudrate=115200, timeout=2)
                set_text = (0x00, 0x11, 0x01, 0x45)
                evo.flushInput()
                evo.write(set_text)
                evo.flushOutput()
                print("[MainProcess] : Connected to Evo (LIDAR1)")
                evo_data = True
            except serial.SerialException as e:
                print("[MainProcess] : Cannot find Evo LIDAR1" + str(e))
                time.sleep(2)
                continue
            except:
                print("[MainProcess] : Cannot find Evo LIDAR1")
                time.sleep(2)
                continue

    # START STEREO
    try:
        mp.Process(target=stereo.Stereoscopics, args=[stereo_data, working_on_the_Pi, GREEN, kill_event]).start()

        if working_on_the_Pi:
            RED_1.on()

    except Exception as e:
        print('[MainProcess] : Stereo thread failed because of exception ' + str(e))

    PitchYaw(guiData, stereo_data, stereo_py_main, final_dist_py, future_dist_py, working_on_the_Pi, YELLOW, kill_event)

    while not shutdown_event.is_set() and not kill_event.is_set():
        if drillType == "Dynamic":
            if working_on_the_Pi:
                if loop_count % 2 == 0:
                    BLUE.on()
                else:
                    BLUE.off()

            loop_count += 1
            if loop_count == 100:
                print("[MainProcess] : FUT_FINAL_DIST =   " + str(FUT_FINAL_DIST))
                loop_count = 1

            StartTime = time.time()
            # Get(WAIT) for stereoDistance _____________________________
            stereo = False
            while not stereo and not shutdown_event.is_set() and not kill_event.is_set():
                try:
                    tempData = stereo_py_main.get(timeout=1)
                    stereo_Distance = float(tempData[2])
                    stereo = True
                except AttributeError as att:
                    print("[MainProcess] : No data in stereoStack" + str(att))
                    while not stereo and not kill_event.is_set():
                        try:
                            tempData = stereo_py_main.get(timeout=1)
                            stereo_Distance = float(tempData[2])
                            stereo_present = True
                        except:
                            # print("[MainProcess] : Waiting for Stereo...")
                            time.sleep(0.1)
                            continue

                except Exception as q:
                    print("[MainProcess] : No data in stereoStack" + str(q))
                    while not stereo and not shutdown_event.is_set() and not kill_event.is_set():
                        try:
                            tempData = stereo_py_main.get(timeout=1)
                            stereo_Distance = float(tempData[2])
                            stereo_present = True
                        except:
                            print("[MainProcess] : Waiting for Stereo...")
                            time.sleep(3)
                            continue

            print("[MainProcess] :  ", stereo_Distance)

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
                        #     lidar1Stack.push(LIDAR_1_Distance)
                        # else:
                        #     lidar1Stack.push(None)  # << None value indicates no GOOD new data

                    except Exception as w:
                        print("[MainProcess] : LIDAR_1 -> no data" + str(w))
                        pass

                # Get(NO_WAIT)for Lidar_2_Dist (Run on New Data EVENT() trigger?)  _____________________________
                try:
                    MEGA_DATA = mega_data.get_nowait()
                    LIDAR_2_Distance = MEGA_DATA.lidar_2_Distance
                    print("[MainProcess] : ", LIDAR_2_Distance)
                    if LIDAR_2_Distance is not None and abs(
                            stereo_Distance - LIDAR_2_Distance) <= 5:  # <<<<<<USE WEIGHTING FACTOR INSTEAD
                        rationaleDistMeasures += 1
                        distanceTotal += LIDAR_2_Distance
                except Exception as w:
                    print("[MainProcess] : LIDAR_2 -> no data" + str(w))
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

                final_dist_py.put(PRE_FINAL_DIST)

                future_dist_py.put(FUT_FINAL_DIST)



            else:
                print("[MainProcess] : Player is not in Range")
                # Activate GPIO pin for notification LED
                # ***Do something about this***
                #
                #
                #
                time.sleep(0.2)

        elif drillType == "Static":
            print("[MainProcess] : Main doesnt do much here")
            time.sleep(1)

        elif drillType == "Manual":
            print("[MainProcess] : Main doesnt do much here")
            time.sleep(1)
        else:
            print("[MainProcess] : no GUI data")
            time.sleep(1)
