'''# _________________________________READ ME______________________________________#
Processes:
This file uses the python "multiprocessing" module to run tasks that are already leveraging
the cores of the CPU, 'concurrently', self.processes are better than Threads in this aspect as it
allows each task to consume more memory as well as allowing the operating system to handle
how and when the data gets processed by the CPU, rather than creating an inefficient
"queue" of tasks ourself.

Threads:
Threads are useful when a process needs to perform CPU I/O and hardware I/O concurrently

Threads provide no benefit in python for CPU intensive tasks because of the Global Interpreter Lock (GIL).

HELP:
  '''

working_on_the_Pi = False

# Packages
import include.launcher as launcher
import include.pitch_yaw as pitch_yaw
import include.stereo as stereo

try:
    import serial
    import multiprocessing as mp
    import serial.tools.list_ports
    import sys
    from collections import deque
    import time
    import numpy as np

    if working_on_the_Pi:
        try:
            from gpiozero import LED
        except ImportError as imp:
            print("IMPORTANT  :   ARE YOU WORKING THE RASPBERRY PI ?:  ", imp)
    import_error = False
except ImportError as imp:
    print("IMPORTANT  :   IMPORT ERROR:  ", imp, "!!!!!")
    import_error = True
    pass


class data_object:
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


# GLOBAL LED NOTIFICATION LIGHTS
if working_on_the_Pi:
    RED_1 = LED(13)
    RED_2 = LED(21)
    RED_3 = LED(6)
    YELLOW = LED(26)
    GREEN = LED(16)
    BLUE = LED(20)
    WHITE = LED(19)
else:
    RED_1 = 13
    RED_2 = 21
    RED_3 = 6
    YELLOW = 26
    GREEN = 16
    BLUE = 20
    WHITE = 19
# GLOBAL VARIABLES:
global loop_count
loop_count = 1

def gpio_blinker(color):
    global loop_count
    if working_on_the_Pi:
        if loop_count % 2 == 0:
            color.on()
        else:
            color.off()

def loop_counter(loop_number):
    loop_number += 1
    if loop_number >= 10:
        loop_number = 1
    return loop_number

def findEvo():
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        if "5740" in p[2]:
            return (p[0])
    return ('NULL')

def Lidar1Dist(evo):
    while (1):
        try:
            distance = evo.readline()
            if distance == "-Inf" or distance == "+Inf":
                print("[MainProcess-Evo]: Out of Range")
            else:
                try:
                    distance = float(distance)
                    # print([]distance)
                    return distance
                except Exception as e:
                    print("[MainProcess-Evo] : error converting to float", e)
                    return None
        except serial.SerialException as a:
            print("[MainProcess-Evo] : No Evo Lidar present... connect it and restart the application" , a)
            return None


class startMainFile():
    ''' MAINFILE STACK IO
    RECEIVE:
      stereoQueue
      megaDataQueue
    SEND:
      finalDistStack
      futureDistStack
    '''
    def __init__(self,speed, difficulty, drillType, pause_event, kill_event, PitchYawStart, LauncherStart, EvoStart,show_camera,voice_con,drill_results,player_select):
        super(startMainFile,self).__init__()
        # __ VARIABLES _____#
        self.focalsize = 3.04e-03
        self.pixelsize = 1.12e-06
        self.baseline = 0.737

        # COMMUNICATION
        self.stereo_data = mp.Queue()
        self.stereo_py_main = mp.Queue()
        self.mega_data = mp.Queue()
        self.future_dist_l = mp.Queue()
        self.final_dist_l = mp.Queue()
        self.future_dist_py = mp.Queue()
        self.final_dist_py = mp.Queue()
        # SIGNALING
        self.data_flag = mp.Event()
        self.launch_event = mp.Event()
        self.py_reset_event = mp.Event()
        self.pymain_stereo_flag = mp.Event()
        self.mega_send_flag = mp.Event()
        # *** DATA TUNING PARAMETERS *** #
        self.max_measures = 20
        self.replacements = 0
        self.distances = deque([])
        self.times = deque([])
        self.first_measurement = True
        self.player_running = False
        self.processes = []
        # __ GUI Input __ #
        self.guiData = data_object
        self.guiData.speed = speed
        self.guiData.difficulty = difficulty
        self.guiData.drilltype = drillType
        # INITIALIZATION DATA
        self.show_camera = show_camera
        self.StartPitchYaw = PitchYawStart
        self.StartLauncher = LauncherStart
        self.EvoLidar = EvoStart
        self.kill_event = kill_event
        self.pause_event = pause_event
        self.voice_control = voice_con
        self.results = drill_results
        self.player_choice = player_select

    def shutdown_func(self):
        print("[MainProcess] : EXIT BUTTON PRESSED")
        self.py_reset_event.set()
        while self.py_reset_event.is_set() and self.processes[1].is_alive():
            time.sleep(0.5)
            if self.py_reset_event.is_set():
                print("[MainFile] : waiting for yaw motor reset 2")

        for p in self.processes:
            time.sleep(1)
            p.terminate()
            p.join()
            print("[MainProcess] :  ", p)

        if working_on_the_Pi:
            RED_1.off()
            RED_2.off()
            RED_3.off()
        print("------------------------------")
        print("[MainProcess]: MainFile Closed")
        print("------------------------------")
        print("")
        print("Start a new Drill?")

    def get_distances(self):
        rationaleDistMeasures = 0
        stereo = False
        while not stereo and not self.kill_event.is_set():
            if self.kill_event.is_set():
                self.shutdown_func()
            try:
                self.pymain_stereo_flag.set()
                tempData = self.stereo_py_main.get(timeout=0.2)
                RightXcoord = int(float(tempData[0]))
                LeftXcoord = int(float(tempData[1]))
                disparity = abs(LeftXcoord - RightXcoord)
                if disparity == 0:
                    disparity = 1
                stereo_Distance = round((self.focalsize * self.baseline) / (disparity * self.pixelsize), 2)
                # print("[MainProcess] : Distance =  ", stereo_Distance)
            except AttributeError as att:
                # print("[MainProcess] : No data in stereoStack" + str(att))
                continue

            except Exception as q:
                # print("[MainProcess] : No data in stereoStack" + str(q))
                continue
            else:
                try:
                    if 1 <= stereo_Distance <= 35:  # Meters
                        # ************ NEW DATA SMOOTHER (MAY NEED TUNING) ***************** #
                        first_new = False
                        new_distance = stereo_Distance
                        if not self.first_measurement:
                            self.read_time = time.time() - self.reading_time
                            self.times.append(self.read_time)
                            if len(self.times) == self.max_measures:
                                self.times.popleft()
                        self.first_measurement = False
                        self.reading_time = time.time()

                        try:
                            if len(self.distances) == self.max_measures:
                                moving_avgs = np.convolve(self.distances, np.ones((5)) / 5, mode='valid')

                                if abs(new_distance - moving_avgs[15]) <= 2.0:
                                    self.distances.append(new_distance)
                                    self.replacements = 0
                                    # print("stereo",self.distances)
                                    first_new = True
                                else:
                                    if first_new:
                                        self.distances.append(new_distance)
                                        # print("2",self.distances)
                                    else:
                                        self.replacements +=1
                                        if self.replacements <= 10:
                                            slope1 = moving_avgs[14] - moving_avgs[13]
                                            slope2 = moving_avgs[15] - moving_avgs[14]
                                            avg_change = (slope1 + slope2) / 2
                                            new_distance = float(round(self.distances[19] + avg_change,2))
                                            # print(new_distance)
                                            self.distances.append(new_distance)
                                            # print("replaced",self.distances)

                                            # DETECT RUNNiNG PLAYER:
                                            if moving_avgs[15] - moving_avgs[0] > 3:
                                                self.player_running = True
                                                self.player_speed = (moving_avgs[15] - moving_avgs[0])/sum(self.times[2:17])
                                            else:
                                                self.player_running = False
                                self.distances.popleft()
                            else:
                                self.distances.append(new_distance)
                                # print("populating",self.distances)

                        except Exception as e:
                            print("issue here",e)
                        else:
                            rationaleDistMeasures = 1.0
                            distanceTotal = new_distance
                            FINAL_DIST = float(round(distanceTotal / rationaleDistMeasures, 2))
                            stereo = True
                            # print("[MainFile]: FINAL DISTANCE CALC =  ", FINAL_DIST)
                            return FINAL_DIST
                    else:
                        print("[MainFile] stereo_Distance not in range (1-35)")
                        return None
                except:
                    print("[MainFile] : Player is out of range")
#                else:
#                    if self.EvoLidar:
#                        try:
#                            LIDAR_1_Distance = Lidar1Dist(evo)
#                            evo.flushOutput()
#                            time.sleep(0.05)
#                            if LIDAR_1_Distance is not None and abs(stereo_Distance - LIDAR_1_Distance) <= 5:  # <<<<<<USE WEIGHTING FACTOR INSTEAD
#                                print("[MainProcess] Evo : ", LIDAR_1_Distance, " meters")
#                                rationaleDistMeasures += 1
#                                distanceTotal += LIDAR_1_Distance
#                        except Exception as w:
#                            # print("[MainProcess] : LIDAR_1 -> no data" + str(w))
#                            pass
#                    # Get(NO_WAIT)for LIDAR2 DISTANCE (Run on New Data EVENT() trigger?)  _____________________________
#                    try:
#                        tempData = self.mega_data.get_nowait()
#                        tempData = tempData.strip("<")
#                        tempData = tempData.strip(">")
#                        tempData = tempData.split(",")
#
#                        lidar_2_Distance = int(tempData[0])  # lidarDistance = int(cm)
#
#                        print("[MainProcess] Garmin : ", lidar_2_Distance, " meters")
#                        if lidar_2_Distance is not None and abs(stereo_Distance - lidar_2_Distance) <= 5:  # <<<<<<USE WEIGHTING FACTOR INSTEAD
#                            rationaleDistMeasures += 1
#                            distanceTotal += lidar_2_Distance
#                    except Exception as w:
#                        # print("[MainProcess] : LIDAR_2 -> no data" + str(w))
#                        pass


    def get_future_dist(self):
        if len(self.distances) == self.max_measures:
            moving_avgs = np.convolve(self.distances, np.ones((5)) / 5, mode='valid')
            future_distance = self.distances[19]
            return future_distance
        else:
            return None

    def run(self):
#        print("[MAINFILE]: PLAYER ", self.player_choice)
        if working_on_the_Pi:
            RED_1.off()
            RED_2.off()
            RED_3.off()

        global loop_count
        print("------------------------------")
        print("")
        print("[MainProcess] : WELCOME ",self.player_choice,", BEGINNING NEW DRILL :")
        print("Speed:  " + str(self.guiData.speed) + "  " + "Diff:  " + str(
            self.guiData.difficulty) + "  " + "Drill:  " + self.guiData.drilltype)
        print("")
        print("------------------------------")
        # ____________________________ MAIN FILE STARTUP ____________________________
        if self.EvoLidar:
            evo_data = False

            while not evo_data and not self.kill_event.is_set():
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

        # ____________________________ Start PROCESSES ____________________________ #
        print("Starting PROCESSES")

        if import_error:
            print("[WARNING-MainProcess] : STERESCOPICS NOT STARTING")
        else:
            try:
                Stereo = mp.Process(target=stereo.Stereoscopics,
                                    args=[self.stereo_data, working_on_the_Pi, GREEN, self.kill_event, self.show_camera,
                                          self.pause_event])  # self.data_flag])
                Stereo.daemon = True
                self.processes.append(Stereo)
                if working_on_the_Pi:
                    RED_1.on()
            except Exception as e:
                print('[WARNING-MainProcess] : Stereo thread failed because of exception ' + str(e))

        if self.StartPitchYaw:
            try:
                PitchYaw = pitch_yaw.PitchYaw(self.guiData, self.stereo_data, self.stereo_py_main, self.final_dist_py, self.future_dist_py,
                                              working_on_the_Pi, YELLOW, self.kill_event, self.py_reset_event, self.pause_event, self.data_flag,
                                              self.pymain_stereo_flag, self.launch_event)
                self.processes.append(PitchYaw)
                if working_on_the_Pi:
                    RED_2.on()
            except Exception as e:
                print('[WARNING-MainProcess] : pitchYawthread didnt start because of exception ' + str(e))
        else:
            print('[WARNING-MainProcess] : pitchYawthread thread is not starting')

        if self.StartLauncher:
            try:
                Launch = launcher.Launcher(self.guiData, self.mega_data, self.final_dist_l, self.future_dist_l, working_on_the_Pi, WHITE,
                                           self.kill_event, self.pause_event, self.py_reset_event, self.launch_event, self.voice_control, self.results,self.player_choice)
                self.processes.append(Launch)
                if working_on_the_Pi:
                    RED_3.on()
            except Exception as e:
                print('[WARNING-MainProcess] : Launcher thread didnt start because of exception ' + str(e))
                time.sleep(1)
        else:
            print('[WARNING-MainProcess] : startLauncherThread thread is not starting')

        try:
            for p in self.processes:
                p.daemon = True
                p.start()
        except:
            print("[WARNING-MainFile] : Issue starting process", p)

        if self.kill_event.is_set():
            self.shutdown_func()

        # ____________________________ MAIN_FILE LOOP ____________________________ #
            # TO DO
            # - Put the process stuff above into a function to clean it up
            # - Instead of nesting functions make this file a class of functions (decent amount of work)
            # -\
        self.replacements = 0
        while not self.kill_event.is_set():

            if self.guiData.drilltype == "Dynamic":
# _________________ PREDICTIVE TRACKING DRILL _________________  #
                if working_on_the_Pi:
                    if loop_count % 2 == 0:
                        BLUE.on()
                    else:
                        BLUE.off()
                loop_count += 1
                if loop_count == 100:
                    loop_count = 1

                StartTime = time.time()

                try:
                    # _______________________ FINAL AVERAGED DISTANCE (STEREO + LIDAR1 + LIDAR2) _______________________ #
                    current_dist = self.get_distances()
                    # SEND THIS TO PITCHYAW AND THE LAUNCHER PROCESS
                    self.final_dist_l.put(current_dist)
                    self.final_dist_py.put(current_dist)
                except:
                    print("[MainProcess] : Waiting for FINAL distance")
                    continue
                else:
                    # _______________________ FUTURE DISTANCE PREDICTION FOR DYNAMIC _______________________ #
                    if current_dist and self.player_running:
                        future_distance = self.get_future_dist()
                        # SEND THIS TO PITCHYAW AND THE LAUNCHER PROCESS
                        self.future_dist_l.put(future_distance)
                        self.future_dist_py.put(future_distance)

# _________________ BASIC TRACKING DRILL _________________  #
            elif self.guiData.drilltype == "Static":
                if working_on_the_Pi:
                    if loop_count % 2 == 0:
                        BLUE.on()
                    else:
                        BLUE.off()
                loop_count += 1
                if loop_count == 100:
                    loop_count = 1

                try:
                    # _______________________ FINAL AVERAGED DISTANCE (STEREO + LIDAR1 + LIDAR2) _______________________ #
                    current_dist = self.get_distances()
                    # SEND THIS TO PITCHYAW AND THE LAUNCHER PROCESS
                    self.final_dist_l.put(current_dist)
                    self.final_dist_py.put(current_dist)
                except:
                    print("[MainProcess] : Waiting for FINAL distance")
                    continue

# _________________ WAIT FOR VC BASIC TRACKING DRILL _________________  #
            elif self.guiData.drilltype == "Manual":
                if working_on_the_Pi:
                    if loop_count % 2 == 0:
                        BLUE.on()
                    else:
                        BLUE.off()
                loop_count += 1
                if loop_count == 100:
                    loop_count = 1

                try:
                    # _______________________ FINAL AVERAGED DISTANCE (STEREO + LIDAR1 + LIDAR2) _______________________ #
                    current_dist = self.get_distances()
                    # print("NEW: ", current_dist)
                    # SEND THIS TO PITCHYAW AND THE LAUNCHER PROCESS
                    self.final_dist_l.put(current_dist)
                    self.final_dist_py.put(current_dist)
                except:
                    print("[MainProcess] : Waiting for FINAL distance")
                    continue
            else:
                print("[MainProcess] : no GUI data")
                time.sleep(1)

        if self.kill_event.is_set():
            self.shutdown_func()
            sys.exit()
        else:
            print("[MainProcess] : Not sure what went wrong")