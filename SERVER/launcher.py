import serial
from threading import Event, Thread, Lock
import serial.tools.list_ports
import random
import sys
import time
import multiprocessing as mp




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


def findMEGA():
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        # print(p)            # This causes each port's information to be printed out.
        if "0042" in p[2]:
            return (p[0])
    return ('NULL')


def MegaData(MEGA, sendMegaDataStack, send_flag, send_data):
    # Variables:

    mega_data = data_object

    while True: #not shutdown_event.isSet() and not kill_event.isSet():
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
                                    time.time() - startTime >= 0.5):# and not shutdown_event.isSet() and not kill_event.isSet():
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


class Launcher(mp.Process):
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
-stereo_pitch_launch
- getLidar1Stack
- sendfinalDistStack


SEND to STACKS:
- sendMegaDataStack
- sendLidar2Stack
- sendfinalDistStack
- sendTemperatureStack'''
    def __init__(self, gui_data, send_mega_data, get_final_dist, get_future_dist, pi_no_pi):
        super(Launcher, self).__init__()
        self.getMegaDataStack = Stack()
        self.send_mega_data = send_mega_data
        self.gui_data = gui_data
        self.get_final_dist = get_final_dist
        self.get_future_dist = get_future_dist
        self.send_flag = Event()
        self.working_on_the_Pi = pi_no_pi
        if self.working_on_the_Pi:
            from gpiozero import LED
            YELLOW = LED(19)
            self.color = YELLOW

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
                print("[Launcher] : Holding Loop for:  " + str(
                    self.WAIT_TIME) + "  seconds  Waiting for results from target WIFI")
                time.sleep(.1)

    def wait_for_voice(self):
        while self.voiceCommand != self.beginVC: # and not self.shutdown_event.isSet() and not self.kill_event.isSet():
            try:
                self.MEGA_DATA = self.getMegaDataStack.peek()
                self.voiceCommand = self.MEGA_DATA.voiceCommand
                self.temperature = self.MEGA_DATA.temperature
                print("VC = " + str(self.voiceCommand) + "  TEMP = " + str(self.temperature))
                time.sleep(0.5)
            except:
                print("[Launcher] : Waiting for Voice Command")
                time.sleep(1)
                continue

    # def get_stereodata(self):
    #     # ___________________ RECEIVE STEREO DISTANCE (NO TIMEOUT (slow startup)_________________________________#
    #     break_loop = False
    #     while not break_loop: # and not self.shutdown_event.isSet() and not self.kill_event.isSet():
    #         try:
    #             self.stereoData = self.stereo_pitch_launch.get()
    #             self.stereo_Distance = float(self.stereoData.distance)
    #             # print("[Launcher(Thread)] : stereo_Distance =  " + str(stereo_Distance))
    #         except ValueError as verr:
    #             print("[Launcher] : StereoDistance couldnt be converted to float" + str(verr))
    #             if self.stereoData is None:
    #                 print("[Launcher] : ... because Stack is Empty")
    #                 continue
    #             else:
    #                 bad_val = self.stereo_pitch_launch.get()
    #                 print("[Launcher] : issue with stereo data  :  " ,bad_val,  ", removing it and trying again")
    #                 continue
    #         except Exception as e:
    #             print("[Launcher] : Error getting stereo_pitch_yaw" + str(e))
    #             time.sleep(0.1)
    #             continue
    #         else:
    #             self.distanceTotal = self.stereo_Distance
    #             self.rationaleDistMeasures = 1
    #             break_loop = True

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

            self.send_mega_data.put(self.MEGA_DATA)

            print("<" + str(self.lidar_2_Distance) + "," + str(self.temperature) + "," + str(
                self.voiceCommand) + "," + str(self.targetTiming) + "," + str(self.targetBallSpeed) + ">")
        except serial.SerialException as err:
            print("[Launcher] : MEGA not detected" + err)
            print("[Launcher] : Closing Launcher")
            # self.shutdown_event.set()
        except Exception as e:  # (serial.SerialException)
            print("[Launcher] : Error in MegaData" + str(e))
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
            #  print("[Launcher] : targetTiming = " + str(targetTiming))
            print("[Launcher] : targetTiming = " + str(self.targetTiming))
            #
            #   DO SOMETHING WITH THIS
            #
            targetTiming = 0  # < reset

        if self.targetBallSpeed != 0:
            print("[Launcher] : targetBallSpeed = " + str(self.targetBallSpeed))
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
                self.wait_for_voice()

        launch_data = False
        while not launch_data:
            try:
                FINAL_DIST = self.get_final_dist.get()  # <<<<< GET PREDICTED LOCATION
            except:
                print("[Launcher] : Waiting for final distance")
                time.sleep(2)
                continue
            # # ___________________       GARMIN LIDAR:    _______________________________ #
        #
        # if self.lidar_2_Distance is not None and abs(self.stereo_Distance - self.lidar_2_Distance) <= 5:
        #     self.distanceTotal += self.lidar_2_Distance
        #     self.rationaleDistMeasures += 1
        #     print("[Launcher] : lidar_2_Distance:  " + str(self.lidar_2_Distance))
        #
        # # ___________________       EVO LIDAR:    _______________________________ #
        #
        # try:
        #     lidar_1_Distance = self.getLidar1Stack.peek()
        #     lidar_1_Distance = lidar_1_Distance / 1000  # <<<<<< CONVERT to meters from mm
        #     if lidar_1_Distance is not None and abs(
        #             self.stereo_Distance - lidar_1_Distance) <= 5:  # <<<<<<USE WEIGHTING FACTOR INSTEAD
        #         self.rationaleDistMeasures += 1
        #         self.distanceTotal += lidar_1_Distance
        #         print("[Launcher] : lidar_1_Distance:  " + str(lidar_1_Distance))
        # except Exception as stackemp:
        #     print("[Launcher] : LIDAR_1 -> nothing in Lidar1Stack" + str(stackemp))
        #     pass
        #
        # # CALCULATE AND SEND TOTAL TO MAIN THREAD
        # try:
        #     FINAL_DIST = self.distanceTotal / self.rationaleDistMeasures
        #     self.sendfinalDistStack.push(FINAL_DIST)
        # except ZeroDivisionError:
        #     print("[Launcher] : ** No VALID distance data **")
        #       # < Go back to the beginning and try again
        # else:
        #     print("[Launcher] : FINAL_DIST:  " + str(FINAL_DIST))

            self.used_distance = FINAL_DIST

            self.send_launch_data()

    def launcher_startup(self):
        startData = False
        while not startData: # and not self.shutdown_event.isSet() and not self.kill_event.isSet():  # <- Add a timeout to the the start loop
            self.drillSpeed = self.gui_data.speed
            self.difficulty = self.gui_data.difficulty
            self.drillType = self.gui_data.drilltype
            try:

                # INSTANTIATE THE SERIAL PORT:
                mega_port = findMEGA()
                self.MEGA = serial.Serial(mega_port, 115200, timeout=1)
                self.MEGA.baudrate = 115200

                # START COMMUNICATIONS THREAD
                mega_data_thread = Thread(target=MegaData,
                                          args=[self.MEGA, self.getMegaDataStack, self.send_flag, self.send_mega_stack])
                mega_data_thread.start()

            except serial.SerialException as err:
                print("[Launcher] : MEGA thread failed to start" + str(err))
                time.sleep(2)
                continue
            except Exception as nodata:
                print("[Launcher] : ** Mega data thread failed to start:  " + str(nodata) + "**")
                time.sleep(2)
                # *****

                    # Get TEMPERATURE and WAIT for voiceCommand
            else:
                print("[Launcher] : Connected to MEGA")
                # Get drillType data from GUI__ #

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
        # LPS BLINKER
        gpio_blinker(self.color, self.self.launch_loop_count, self.working_on_the_Pi)

        self.launch_loop_count = loop_counter(self.launch_loop_count)

        self.drill_wait_time()

    def dynamic_drill(self):
        self.WAIT_TIME = self.DYNAMIC_WAIT_TIME
        print("[Launcher] : Beginning Drill")

        # ___________________ HOLD THE LAUNCHER SEQUENCE FOR 3-4 SECONDS: ___________________
        while self.drillCount <= 5: #and not self.shutdown_event.isSet() and not self.kill_event.isSet():
            self.launcher_common()

            # self.get_stereodata()

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
                    FUT_FINAL_DIST = self.get_future_dist.get()  # <<<<< GET PREDICTED LOCATION
                    print(FUT_FINAL_DIST)
                except:
                    FUT_FINAL_DIST = None
                    pass
                else:
                    if FUT_FINAL_DIST == 0.0 or FUT_FINAL_DIST == self.OLD_FUT_FINAL_DIST:  # <- no prediction is done in this thread so it will send AS IS

                        self.ballFeed = "1"

                        self.send_launch_data()

                    else:
                        self.used_distance = FUT_FINAL_DIST
                        self.ballFeed = "1"

                        self.send_launch_data()

        if self.drillCount == 5:
            print("[Launcher] : Drill COMPLETE!")

        else:
            print("[Launcher] : Not sure what went wrong")

    def static_drill(self):
        self.WAIT_TIME = self.STATIC_WAIT_TIME
        while self.drillCount <= 5:# and not self.shutdown_event.isSet() and not self.kill_event.isSet():
            self.launcher_common()

            # self.get_stereodata()

            try:
                self.get_mega_data()
            except:
                continue
            else:
                self.ballfeed = "1"

                self.launch_motors()

        if self.drillCount == 5:
            print("[Launcher] : Drill COMPLETE!")


        else:
            print("[Launcher] : Not sure what went wrong")

    def manual_drill(self):
        self.WAIT_TIME = 0 # <<< WAIT TIME IS HANDLED DIFFERENTLY HERE
        while self.drillCount <= 5:# and not self.shutdown_event.isSet() and not self.kill_event.isSet():

            # WAIT FOR VC FOR EVERY LAUNCh
            self.wait_for_voice()

            self.launcher_common()

            # self.get_stereodata()

            try:
                self.get_mega_data()
            except:
                continue
            else:
                self.ballfeed = "1"

                self.launch_motors()

        if self.drillCount == 5:
            print("[Launcher] : Drill COMPLETE!")


        else:
            print("[Launcher] : Not sure what went wrong")

    def run(self):
        print("[Launcher] : starting launcher thread")
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
            print('[Launcher] : failed because of exception: ' + str(e))

