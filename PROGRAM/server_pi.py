
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

working_on_the_Pi = False

# Packages
import stereo
import launcher
import pitch_yaw
try:
    import serial
    import multiprocessing as mp
    import serial.tools.list_ports
    import sys
    from collections import deque
    import time

    if working_on_the_Pi:
        try:
            from gpiozero import LED
        except ImportError as imp:
            print("IMPORTANT  :   ARE YOU WORKING THE RASPBERRY PI ?:  ", imp)

    import_error = False
except ImportError as imp:
    print("IMPORTANT  :   IMPORT ERROR:  " , imp , "!!!!!")
    import_error = True
    pass


# from gui import *  # Custom Module

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


# GLOBAL STACKS:


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

# GLOBAL FLAGS/EVENTS:

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
# _______PITCH AND YAW THREAD________ #



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


# startMainFile STACK IO
# RECEIVE from STACKS:
#   stereoStack
#
# SEND to STACKS:
#   futureDistStack
#
#


def startMainFile(speed, difficulty, drillType, shutdown_event, kill_event, PitchYawStart, LauncherStart, EvoStart):  # , args): ## NOT A THREAD, performs the bulk of calculation
    # # _______________________________Main Processing_____________________________________
    # PACKAGES:
    import stereo
    # __ VARIABLES _____#
    # startCommand = 0
    stereo_Distance = 0.0
    avg_measures = 10
    lead_time = 3
    # TRACKING LISTS
    z_dist_deque = deque([])
    measure_time_deque = deque([])

    # COMMUNICATION
    stereo_parent, stereo_child = mp.Pipe()
    stereo_data = mp.Queue()
    stereo_py_main = mp.Queue()
    mega_data = mp.Queue()
    future_dist_l = mp.Queue()
    final_dist_l = mp.Queue()
    future_dist_py = mp.Queue()
    final_dist_py = mp.Queue()
    # KILL = mp.Event()
    # kill_queue = mp.Queue()
    # kill_pitchyaw = mp.Queue()
    # kill_stereo = mp.Queue()

    processes = []


    # __ GUI Input __ #
    guiData = data_object
    guiData.speed = speed
    guiData.difficulty = difficulty
    guiData.drilltype = drillType
    FUT_FINAL_DIST = 0.0

    StartPitchYaw = PitchYawStart
    StartLauncher = LauncherStart
    EvoLidar = EvoStart
    kill_event = kill_event
    shutdown_event = shutdown_event
    loop_count = 1

    if working_on_the_Pi:
        RED_1.off()
        RED_2.off()
        RED_3.off()

    print("[MainProcess] : ")
    print("Speed:  " + str(guiData.speed) + "  " + "Diff:  " + str(
        guiData.difficulty) + "  " + "Drill:  " + guiData.drilltype)
    print("_______________________________________________")

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

    # _______________________ Start Threads _______________________--_ #


    print("Starting Threads")

    if import_error:
        print("[MainProcess] : STERESCOPICS NOT STARTING")
    else:
        try:
            processes.append(mp.Process(target=stereo.Stereoscopics, args=[stereo_data, working_on_the_Pi, GREEN, kill_event]))

            if working_on_the_Pi:
                RED_1.on()

        except Exception as e:
            print('[MainProcess] : Stereo thread failed because of exception ' + str(e))

    if StartPitchYaw:
        try:
            processes.append(pitch_yaw.PitchYaw(guiData, stereo_data, stereo_py_main, final_dist_py, future_dist_py,  working_on_the_Pi, YELLOW, kill_event))
            
            if working_on_the_Pi:
                RED_2.on()

        except Exception as e:
            print('[MainProcess] : pitchYawthread didnt start because of exception ' + str(e))
    else:
        print('[MainProcess] : pitchYawthread thread is not starting')

    if StartLauncher:
        try:
            processes.append(launcher.Launcher(guiData, mega_data, final_dist_l, future_dist_l, working_on_the_Pi, WHITE, kill_event))

            if working_on_the_Pi:
                RED_3.on()

        except Exception as e:
            print('[MainProcess] : Launcher thread didnt start because of exception ' + str(e))
            time.sleep(1)
    else:
        print('[MainProcess] : startLauncherThread thread is not starting')

    try:
        for p in processes:
            p.start()
    except:
        print("{MainFile} : Issue starting threads")


    if shutdown_event.isSet():
        print("[MainProcess] : STOP BUTTON PRESSED")
    if kill_event.isSet():
        print("[MainProcess] : EXITING...")
        sys.exit()

    # ___________ "MAIN THREAD" LOOP __________ #
    while not shutdown_event.isSet() and not kill_event.isSet():
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
            while not stereo and not shutdown_event.isSet() and not kill_event.isSet():
                try:
                    tempData = stereo_py_main.get(timeout=1)
                    stereo_Distance = float(tempData[2])
                    stereo = True
                except AttributeError as att:
                    print("[MainProcess] : No data in stereoStack" + str(att))
                    while not stereo and not kill_event.isSet():
                        try:
                            tempData = stereo_py_main.get(timeout=1)
                            stereo_Distance = float(tempData[2])
                            stereo_present = True
                        except:
                            #print("[MainProcess] : Waiting for Stereo...")
                            sys.stdout.flush()
                            time.sleep(0.1)
                            continue

                except Exception as q:
                    print("[MainProcess] : No data in stereoStack" + str(q))
                    while not stereo and not shutdown_event.isSet() and not kill_event.isSet():
                        try:
                            tempData = stereo_py_main.get(timeout=1)
                            stereo_Distance = float(tempData[2])
                            stereo_present = True
                        except:
                            print("[MainProcess] : Waiting for Stereo...")
                            time.sleep(3)
                            continue
                else:
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
                    if LIDAR_2_Distance is not None and abs(stereo_Distance - LIDAR_2_Distance) <= 5:  # <<<<<<USE WEIGHTING FACTOR INSTEAD
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

                final_dist_l.put(PRE_FINAL_DIST)
                final_dist_py.put(PRE_FINAL_DIST)

                future_dist_l.put(FUT_FINAL_DIST)
                future_dist_py.put(FUT_FINAL_DIST)



            elif not shutdown_event.isSet() and not kill_event.isSet():
                print("[MainProcess] : Player is not in Range")
                # Activate GPIO pin for notification LED
                # ***Do something about this***
                #
                #
                #
                #time.sleep(0.2)

            else:
                break

        elif drillType == "Static":
            print("[MainProcess] : Main doesnt do much here")
            time.sleep(1)

        elif drillType == "Manual":
            print("[MainProcess] : Main doesnt do much here")
            time.sleep(1)
        else:
            print("[MainProcess] : no GUI data")
            time.sleep(1)

    if shutdown_event.isSet():
        print("[MainProcess] : STOP BUTTON PRESSED")
        for p in processes:
            if working_on_the_Pi:
                RED_1.off()
                RED_2.off()
                RED_3.off()
            time.sleep(1)
            p.terminate()
            p.join()
            print("[MainProcess] :  ", p)
        time.sleep(2)

    elif kill_event.isSet():
        print("[MainProcess] : EXIT BUTTON PRESSED")
        for p in processes:
            if working_on_the_Pi:
                RED_1.off()
                RED_2.off()
                RED_3.off()
            time.sleep(1)
            p.terminate()
            p.join()
            print("[MainProcess] :  ", p)
        sys.exit()
    else:
        print("[MainProcess] : Not sure what went wrong")
