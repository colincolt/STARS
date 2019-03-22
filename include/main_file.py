'''# _________________________________READ ME______________________________________#
Processes:
This file uses the python "multiprocessing" module to run tasks that are already leveraging
the cores of the CPU, 'concurrently', processes are better than Threads in this aspect as it
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
startMarker = '<'


def gpio_blinker(color, loop_count):
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


# startMainFile STACK IO
# RECEIVE from STACKS:
#   stereoQueue
#   megaDataQueue
# SEND to STACKS:
#   finalDistStack
#   futureDistStack
#

def startMainFile(speed, difficulty, drillType, pause_event, kill_event, PitchYawStart, LauncherStart, EvoStart,
                  show_camera):
    # # _______________________________Main Processing_____________________________________
    # PACKAGES:
    import include.stereo as stereo
    # __ VARIABLES _____#
    avg_measures = 10
    lead_time = 3
    focalsize = 3.04e-03
    pixelsize = 1.12e-06
    baseline = 0.737
    # TRACKING LISTS
    z_dist_deque = deque([])
    measure_time_deque = deque([])

    # COMMUNICATION
    stereo_data = mp.Queue()
    stereo_py_main = mp.Queue()
    mega_data = mp.Queue()
    future_dist_l = mp.Queue()
    final_dist_l = mp.Queue()
    future_dist_py = mp.Queue()
    final_dist_py = mp.Queue()
    data_flag = mp.Event()
    processes = []

    # __ GUI Input __ #
    guiData = data_object
    guiData.speed = speed
    guiData.difficulty = difficulty
    guiData.drilltype = drillType

    StartPitchYaw = PitchYawStart
    StartLauncher = LauncherStart
    EvoLidar = EvoStart
    kill_event = kill_event
    pause_event = pause_event
    py_reset_event = mp.Event()
    pymain_stereo_flag = mp.Event()
    mega_send_flag = mp.Event()
    loop_count = 1

    # *** DATA TUNING PARAMETERS *** #
    max_measures = 20
#    global first_new
    replacements = 0
    distances = deque([])

    if working_on_the_Pi:
        RED_1.off()
        RED_2.off()
        RED_3.off()

    print("------------------------------")
    print("")
    print("[MainProcess] : BEGINNING NEW DRILL :")
    print("Speed:  " + str(guiData.speed) + "  " + "Diff:  " + str(
        guiData.difficulty) + "  " + "Drill:  " + guiData.drilltype)
    print("")
    print("------------------------------")

    def shutdown_func():
        print("[MainProcess] : EXIT BUTTON PRESSED")
        py_reset_event.set()
        while py_reset_event.is_set() and processes[1].is_alive():
            time.sleep(0.5)
            if py_reset_event.is_set():
                print("[MainFile] : waiting for yaw motor reset 2")

        for p in processes:
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

    def get_distances(distances, replacements):
        rationaleDistMeasures = 0
        # Get(WAIT) for stereoDistance _____________________________
        stereo = False
        while not stereo and not kill_event.is_set():
            if pause_event.is_set():
                print("[MainProcess] : PAUSE BUTTON PRESSED")
                while pause_event.is_set():
                    time.sleep(1)
            if kill_event.is_set():
                shutdown_func()
            try:
                pymain_stereo_flag.set()
                # print("Main: flag is set")
                tempData = stereo_py_main.get(timeout=0.2)
                RightXcoord = int(float(tempData[0]))
                LeftXcoord = int(float(tempData[1]))
                disparity = abs(LeftXcoord - RightXcoord)
                if disparity == 0:
                    disparity = 1
                stereo_Distance = round((focalsize * baseline) / (disparity * pixelsize), 2)
#                print("[MainProcess] : Distance =  ", stereo_Distance)
            except AttributeError as att:
#                print("[MainProcess] : No data in stereoStack" + str(att))
                continue

            except Exception as q:
#                print("[MainProcess] : No data in stereoStack" + str(q))
                continue
            else:
                try:
                    if 1 <= stereo_Distance <= 35:  # Meters
                        # ************ NEW DATA SMOOTHER (MAY NEED TUNING) ***************** #
                        first_new = False
                        new_distance = stereo_Distance
#                        print("in dist check",new_distance)
                        try:
                            if len(distances) == max_measures:
#                                print("in if")
                                moving_avgs = np.convolve(distances, np.ones((5)) / 5, mode='valid')
                                
                                if abs(new_distance - moving_avgs[15]) <= 2.0:
                                    distances.append(new_distance)
                                    replacements = 0
#                                    print("stereo",distances)
                                    first_new = True
                                else:
                                    if first_new:
                                        distances.append(new_distance)
#                                        print("2",distances)
                                    else:
                                        replacements +=1
                                        if replacements <= 10:
                                            slope1 = moving_avgs[14] - moving_avgs[13]
                                            slope2 = moving_avgs[15] - moving_avgs[14]
                                            avg_change = (slope1 + slope2) / 2
                                            new_distance = float(round(distances[19] + avg_change,2))
#                                            print(new_distance)
                                            distances.append(new_distance)
#                                            print("replaced",distances)
#                                        else:
##                                            distances = deque([])
#                                            new_distance = stereo_Distance
#                                            distances.append(new_distance)
                                            
                                distances.popleft()
                                
                            else:
#                                print("in else check")
                                distances.append(new_distance)
#                                print("populating",distances)
                        except Exception as e:
                            print("issue here",e)
                        else:
                            #print(len(moving_avgs))
                            #print("[Mainfile]: Past Five Distances: ", distances)
                            #print("[Mainfile]: Used Distance: ", new_distance)
                            # ******************************************************************* #
#                            print("at the end")
                            rationaleDistMeasures = 1.0
                            distanceTotal = new_distance
                            FINAL_DIST = float(round(distanceTotal / rationaleDistMeasures, 2))
                            stereo = True
#                            print("[MainFile]: FINAL DISTANCE CALC =  ", FINAL_DIST)
                            return FINAL_DIST
                    else:
                        print("[MainFile] stereo_Distance not in range (1-35)")
                        return None
                except:
                    print("[MainFile] : Player is out of range")
#                else:
#                    if EvoLidar:
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
#                        tempData = mega_data.get_nowait()
#                        tempData = tempData.strip("<")
#                        tempData = tempData.strip(">")
#                        tempData = tempData.split(",")
#                        # print("[MainFile] : Tempdata=  " + str(tempData))
#
#                        lidar_2_Distance = int(tempData[0])  # lidarDistance = int(cm)
#                        #                        temperature = int(tempData[1])  # temperature = int()
#                        #                        voiceCommand = int(tempData[2])  # voice commands = int(from 1 to 5)
#                        #                        targetTiming = int(tempData[3])  # targetTiming = float(0.0)
#                        #                        targetBallSpeed = float(tempData[4])  # targetBallSpeed
#
#                        print("[MainProcess] Garmin : ", lidar_2_Distance, " meters")
#                        if lidar_2_Distance is not None and abs(stereo_Distance - lidar_2_Distance) <= 5:  # <<<<<<USE WEIGHTING FACTOR INSTEAD
#                            rationaleDistMeasures += 1
#                            distanceTotal += lidar_2_Distance
#                    except Exception as w:
#                        # print("[MainProcess] : LIDAR_2 -> no data" + str(w))
#                        pass

#                else:
                

        

    def get_future_dist(starttime,current_dist):
        measure_time_deque.appendleft(time.time() - starttime)

        if measure_time_deque == avg_measures:
            measure_time_deque.pop()

        if z_dist_deque == []:  # This is the first measurement
            z_dist_deque.appendleft(current_dist)
        elif len(z_dist_deque) < avg_measures:
            z_dist_deque.appendleft(current_dist)
        elif len(z_dist_deque) == avg_measures:
            temp_dist = z_dist_deque[0] - z_dist_deque[avg_measures - 1]
            temp_time = sum([elem for elem in measure_time_deque])  # Time for avg_measure measurements
            playerspeed = temp_dist / temp_time  # meters/second
            FUT_FINAL_DIST = round(current_dist + playerspeed * lead_time, 2)
            # print("[MainFile]: FUT_FINAL_DIST: ", FUT_FINAL_DIST)
            z_dist_deque.appendleft(current_dist)
            z_dist_deque.pop()
            return FUT_FINAL_DIST
    # ____________________________ MAIN FILE STARTUP ____________________________
    if EvoLidar:
        evo_data = False

        while not evo_data and not kill_event.is_set():
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
                                args=[stereo_data, working_on_the_Pi, GREEN, kill_event, show_camera,
                                      pause_event])  # data_flag])
            Stereo.daemon = True
            processes.append(Stereo)
            if working_on_the_Pi:
                RED_1.on()
        except Exception as e:
            print('[WARNING-MainProcess] : Stereo thread failed because of exception ' + str(e))

    if StartPitchYaw:
        try:
            PitchYaw = pitch_yaw.PitchYaw(guiData, stereo_data, stereo_py_main, final_dist_py, future_dist_py,
                                          working_on_the_Pi, YELLOW, kill_event, py_reset_event, pause_event, data_flag,
                                          pymain_stereo_flag)
            processes.append(PitchYaw)
            if working_on_the_Pi:
                RED_2.on()
        except Exception as e:
            print('[WARNING-MainProcess] : pitchYawthread didnt start because of exception ' + str(e))
    else:
        print('[WARNING-MainProcess] : pitchYawthread thread is not starting')

    if StartLauncher:
        try:
            Launch = launcher.Launcher(guiData, mega_data, final_dist_l, future_dist_l, working_on_the_Pi, WHITE,
                                       kill_event, pause_event, py_reset_event)
            processes.append(Launch)
            if working_on_the_Pi:
                RED_3.on()
        except Exception as e:
            print('[WARNING-MainProcess] : Launcher thread didnt start because of exception ' + str(e))
            time.sleep(1)
    else:
        print('[WARNING-MainProcess] : startLauncherThread thread is not starting')

    try:
        for p in processes:
            p.daemon = True
            p.start()
    except:
        print("[WARNING-MainFile] : Issue starting process", p)

    if kill_event.is_set():
        shutdown_func()

    # ____________________________ MAIN_FILE LOOP ____________________________ #
        # TO DO
        # - Put the process stuff above into a function to clean it up
        # - Instead of nesting functions make this file a class of functions (decent amount of work)
        # -\
    replacements = 0
    while not kill_event.is_set():

        if pause_event.is_set():
            print("[MainProcess] : PAUSE BUTTON PRESSED")
            while pause_event.is_set():
                time.sleep(1)

        if drillType == "Dynamic":
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
                current_dist = get_distances(distances,replacements)
                # SEND THIS TO PITCHYAW AND THE LAUNCHER PROCESS
                final_dist_l.put(current_dist)
                final_dist_py.put(current_dist)
            except:
                print("[Launcher] : Waiting for FINAL distance")
                continue
            else:
                # _______________________ FUTURE DISTANCE PREDICTION FOR DYNAMIC _______________________ #
                if current_dist:
                    future_distance = get_future_dist(StartTime, current_dist)
                    # SEND THIS TO PITCHYAW AND THE LAUNCHER PROCESS
                    future_dist_l.put(future_distance)
                    future_dist_py.put(future_distance)

        elif drillType == "Static":
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
                PRE_FINAL_DIST = get_distances(distances,replacements)
                # SEND THIS TO PITCHYAW AND THE LAUNCHER PROCESS
                final_dist_l.put(PRE_FINAL_DIST)
                final_dist_py.put(PRE_FINAL_DIST)
            except:
                print("[Launcher] : Waiting for FINAL distance")
                continue


        elif drillType == "Manual":
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
                PRE_FINAL_DIST = get_distances(distances, replacements)
                print("NEW: ", PRE_FINAL_DIST)
                # SEND THIS TO PITCHYAW AND THE LAUNCHER PROCESS
                final_dist_l.put(PRE_FINAL_DIST)
                final_dist_py.put(PRE_FINAL_DIST)
            except:
                print("[Launcher] : Waiting for FINAL distance")
                continue

        else:
            print("[MainProcess] : no GUI data")
            time.sleep(1)

    if kill_event.is_set():
        shutdown_func()

        sys.exit()
    else:
        print("[MainProcess] : Not sure what went wrong")
