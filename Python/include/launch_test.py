import random
import serial
import serial.tools.list_ports
import sys
import numpy as np
import time


def return_data(distance, left_curve, right_curve):

    curve_left = left_curve
    curve_right = right_curve
    used_distance=int(distance)

    difficulty = "1"
    ballfeed="1"

    pitchAngleTable = np.array([[7.5, 5],  # << Pitch angle lookup table based on estimations
                                [12.5, 12],
                                [17.5, 20],
                                [22.5, 30],
                                [25, 37]])

#    RPM = -1.14 * used_distance ** 2.0 + 98.0 * used_distance + 646.0  # <-- Polynomial fit
#    RPS = RPM / 60
#    PERIOD = (1 / RPS)*(1000000) - 3000
    #motor_speed = round((RPM / 5000) * 255)  # Value between 0-255 (On 24 V: 0-5000 RPM)
    if used_distance <= 7.5:
        RPM = -1.13635244 * used_distance ** 2.0 + 97.7378699 * used_distance + 646.034298  # <-- Polynomial fit
        RPS = RPM / 60
        PERIOD = (1 / RPS)*(1000000)
        print("[Launcher]: RPM: ", int(RPM), "  Period: ", int(PERIOD))

    elif 7.5 < used_distance <= 12.5:
        RPM = -1.13635244 * used_distance ** 2.0 + 97.7378699 * used_distance + 646.034298  # <-- Polynomial fit
        RPS = RPM / 60
        PERIOD = (1 / RPS) * (1000000)+ 11500
        print("[Launcher]: RPM: ", int(RPM), "  Period: ", int(PERIOD))

    elif 12.5 < used_distance <=17.5:
        RPM = -1.13635244 * used_distance ** 2.0 + 97.7378699 * used_distance + 646.034298  # <-- Polynomial fit
        RPS = RPM / 60
        PERIOD = (1 / RPS) * (1000000) +15000
        print("[Launcher]: RPM: ", int(RPM), "  Period: ", int(PERIOD))

    elif 17.5 < used_distance <= 22.5:
        RPM = -1.13635244 * used_distance ** 2.0 + 97.7378699 * used_distance + 646.034298  # <-- Polynomial fit
        RPS = RPM / 60
        PERIOD = (1 / RPS) * (1000000)+10000
        print("[Launcher]: RPM: ", int(RPM), "  Period: ", int(PERIOD))

    elif 22.5 < used_distance <= 35.0:
        RPM = -1.13635244 * used_distance ** 2.0 + 97.7378699 * used_distance + 646.034298  # <-- Polynomial fit
        RPS = RPM / 60
        PERIOD = (1 / RPS) * (1000000) +2500
        print("[Launcher]: RPM: ", int(RPM), "  Period: ", int(PERIOD))

    else: # << catches and strangeness that may happen to distance reading
        RPM = -1.13635244 * self.used_distance ** 2.0 + 97.7378699 * self.used_distance + 646.034298  # <-- Polynomial fit
        RPS = RPM / 60
        PERIOD = (1 / RPS) * (1000000)
        print("[Launcher]: RPM: ", int(RPM), "  Period: ", int(PERIOD))

    if curve_left:
        PERIOD1 = PERIOD - 3000
        PERIOD2 = PERIOD
    elif curve_right:
        PERIOD1 = PERIOD
        PERIOD2 = PERIOD - 3000
    else:  ## NEEDS SCALING
        PERIOD1 = PERIOD
        PERIOD2 = PERIOD

    motor_period1 = str(int(PERIOD1))
    motor_period2 = str(int(PERIOD2))

    targetChoice = int(random.choice([1, 2 , 3, 4]))
    estimated_tof = (0.120617 * used_distance) * 1000  # + difficulty_time
    estimated_tof = round(estimated_tof, 2)
    # motor_speed = str(int(PERIOD))
    drill_type = "0"

    #
    mega_data = '<' + motor_period1 + ',' + motor_period2 + ',' + str(targetChoice) + ',' + str(
        difficulty) + ',' + ballfeed + ',' + str(estimated_tof) + ','+drill_type+'>'

    # Query table for angle at self.usedDistance
#    row = round((used_distance - 0.99) / 2) - 2
#    if row < 0:
#        row = 0
#    elif row > 10:
#        row = 10
#    pitchAngle = pitchAngleTable[row, 1]
    if used_distance <= 7.5:
        row = 0
        pitchAngle = pitchAngleTable[row, 1] #+ self.launcherAngle
    elif 7.5 < used_distance <= 12.5:
        row = 1
        pitchAngle = pitchAngleTable[row, 1]
    elif 12.5 < used_distance <= 17.5:
        row = 2
        pitchAngle = pitchAngleTable[row, 1] 
    elif 17.5 < used_distance <= 22.5:
        row = 3
        pitchAngle = pitchAngleTable[row, 1]
    elif 22.5 < used_distance <= 35:
        row = 4
        pitchAngle = pitchAngleTable[row, 1]
    else:
        pitchAngle = 10

    uno_data = '<0,'+str(pitchAngle)+','+drill_type+'>'

    all_data = mega_data + "_" + uno_data
    return all_data


def findMEGA():
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        if "0042" in p[2]:
            return (p[0])
    return ('NULL')

def findUNO():
    print("finding uno")
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        print(p)           # This causes each port's information to be printed out.
        if "0043" in p[2]:
            return (p[0])
    return ('NULL')


def get_mega(close_event):
    mega_port = findMEGA()
    print("in mega")

    try:
        MEGA = serial.Serial(mega_port, 115200, timeout=1)

    except FileNotFoundError as e:
        print("findMEGA returned 'NULL' looking for Arduino Mega")
    except serial.SerialException as ex:
        print("findMEGA returned 'NULL' looking for Arduino Mega")

    else:
        try:
            MEGA.baudrate = 115200
            send_data = "<-1,-1,1,-1,0,0,0>"
            wait_for_data(MEGA, close_event)
            print(send_data)
            MEGA.write(send_data.encode())
            return MEGA
        except:
            close_event.set()

def get_uno():
    print("getting UNO")
    uno_port = findUNO()

    try:
        UNO = serial.Serial(uno_port, 115200, timeout=1)  # change ACM number as found from "ls /dev/tty*"

    except FileNotFoundError as e:
        print("findUNO returned 'NULL' looking for Arduino Mega")
    except serial.SerialException as ex:
        pass
    else:
        UNO.baudrate = 115200
        return UNO

def wait_for_data(MEGA, close_event):
    getdata = False
    while not getdata and not close_event.is_set():
        
        if MEGA.is_open:
            try:
                # print("in the try")
                startMarker = MEGA.read().decode()

                if startMarker == None:
                    print("start = None")
                    sys.exit()
                if startMarker == "":
                    print(startMarker)
                    print("exiting")
                    sys.exit()
            except Exception as e:
                print('[MegaDataThread] : ERROR: ' + str(e))
            except Exception as e:
                print("not receiving anything",e)
                timeout = True
            else:
                if startMarker == "<":
                    megaDataStr = MEGA.read(25)  # READ DATA FORMATTED AS ('< 1 2 3 4 5 >')
                    megaDataTemp = list(megaDataStr.decode())
                    megaDataTemp.insert(0, startMarker)
                    megaData = megaDataTemp[:megaDataTemp.index(">") + 1]
                    tempData = "".join(megaData)
                    tempData = tempData.strip("<")
                    tempData = tempData.strip(">")
                    tempData = tempData.split(",")
                    print("[MegaData] : Tempdata=  " , tempData)
                    getdata = True
        else:
            print("lost Mega")

def launch(MEGA, UNO, Mega_data, Uno_data,close_launch):
    Mega_data = Mega_data.strip("<")
    Mega_data = Mega_data.strip(">")
    Mega_data = Mega_data.split(",")
    motor_speed = str(Mega_data[0])
    targetChoice = str(Mega_data[2])
    estimated_tof = str(Mega_data[3])

    difficulty = "2"
    ballFeed = "1"
    drill_type = "0"
    send_data = '<' + motor_speed + ',' + motor_speed + ',' + '2' + ','+difficulty +','+ ballFeed +','+ estimated_tof + ','+drill_type+'>'

    time.sleep(3)

    UNO.write(str(Uno_data).encode())
    print("Uno Data:  ",Uno_data)
    #unoDat = UNO.read(10).decode()
    #print("UNO Response", unoDat)
    
#    wait_for_data(MEGA)
    print("Starting motors")

    time.sleep(2)

    wait_for_data(MEGA, close_launch)
    Mega_data = send_data
    MEGA.write(Mega_data.encode())
    print("Mega Data: ", Mega_data)
    print("launching ball")
    loop_count = 0
    while loop_count <= 5:
        loop_count += 1
        wait_for_data(MEGA, close_launch)
        if close_launch.is_set():
            sys.exit()

def run(motor_data, close_launch):
    Mega_data=motor_data[0]
    Uno_data=motor_data[1]
    print(Mega_data)
    print(Uno_data)
    MEGA = get_mega(close_launch)
    if close_launch.is_set():
        exit()
    print("got mega")
    print("getting uno 1")
    UNO = get_uno()
    print("got uno")

    launch(MEGA, UNO, Mega_data, Uno_data, close_launch)
    print("sent to launch")
