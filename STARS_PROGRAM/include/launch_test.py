import random
import serial
import serial.tools.list_ports
import sys
import numpy as np
import time


def return_data(distance):

    used_distance=int(distance)

    difficulty = "1"
    ballfeed="1"

    pitchAngleTable = np.array([[5, 0],  # << Pitch angle lookup table based on estimations
                             [7, 5],
                             [9, 10],
                             [11, 15],
                             [13, 20],
                             [15, 25],
                             [17, 30],
                             [19, 34],
                             [21, 35],
                             [23, 36],
                             [25, 37]])

    RPM = -1.13635244 * used_distance ** 2.0 + 97.7378699 * used_distance + 646.034298  # <-- Polynomial fit
    motor_speed = round((RPM / 5000) * 255)  # Value between 0-255 (On 24 V: 0-5000 RPM)
    targetChoice = int(random.choice([1, 2 , 3, 4]))
    estimated_tof = (0.120617 * used_distance) * 1000  # + difficulty_time
    estimated_tof = round(estimated_tof, 2)
    motor_speed = str(motor_speed)
    #
    mega_data = '<' + motor_speed + ',' + motor_speed + ',' + str(targetChoice) + ',' + str(
        difficulty) + ',' + ballfeed + ',' + str(estimated_tof) + '>'

    # Query table for angle at self.usedDistance
    row = round((used_distance - 0.99) / 2) - 2
    if row < 0:
        row = 0
    elif row > 10:
        row = 10
    pitchAngle = pitchAngleTable[row, 1]

    uno_data = '<0,'+str(pitchAngle)+'>'

    all_data = mega_data + "_" + uno_data
    return all_data


def findMEGA():
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        if "0042" in p[2]:
            return (p[0])
    return ('NULL')

def findUNO():
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        # print(p)           # This causes each port's information to be printed out.
        if "0043" in p[2]:
            return (p[0])
    return ('NULL')


def get_mega():
    mega_port = findMEGA()

    try:
        MEGA = serial.Serial(mega_port, 115200, timeout=1)

    except FileNotFoundError as e:
        print("findMEGA returned 'NULL' looking for Arduino Mega")
    except serial.SerialException as ex:
        pass
    else:
        MEGA.baudrate = 115200
        send_data = "<0,0,1,-1,0,0>"
        wait_for_data(MEGA)
        MEGA.write(send_data.encode())
        return MEGA
    
def get_uno():
    uno_port = findUNO()

    try:
        UNO = serial.Serial(uno_port, 115200, timeout=1)  # change ACM number as found from "ls /dev/tty*"

    except FileNotFoundError as e:
        print("findMEGA returned 'NULL' looking for Arduino Mega")
    except serial.SerialException as ex:
        pass
    else:
        UNO.baudrate = 115200
        return UNO

def wait_for_data(MEGA):
    getdata = False
    while not getdata:
        if MEGA.is_open:
            try:
                startMarker = MEGA.read().decode()
            except Exception as e:
                print('[MegaDataThread] : ERROR: ' + str(e))
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

                    getdata = False
        else:
            print("lost Mega")


def launch(MEGA, UNO, Mega_data, Uno_data,close_launch):
    Mega_data = Mega_data.strip("<")
    Mega_data = Mega_data.strip(">")
    Mega_data = Mega_data.split(",")
    motor_speed = str(Mega_data[0])
    targetChoice = str(Mega_data[1])
    estimated_tof = str(Mega_data[2])

    send_start = '<' + motor_speed + ',' + motor_speed + ',' + targetChoice + ',-1,0,' + estimated_tof + '>'

    print("Launch Data:  ",Mega_data,"  ",Uno_data)

    UNO.write(Uno_data.encode())

    wait_for_data(MEGA)
    MEGA.write(send_start.encode())
    print("Starting motors")
    time.sleep(3)
    wait_for_data(MEGA)
    MEGA.write(Mega_data.encode())
    print("launching ball")
    while True:
        wait_for_data(MEGA)
        if close_launch.is_set():
            sys.exit()


def run(motor_data, close_launch):
    Mega_data=motor_data[0]
    Uno_data=motor_data[1]
    print(Mega_data)
    print(Uno_data)
    MEGA = get_mega()
    UNO = get_uno()

    launch(MEGA, UNO, Mega_data, Uno_data, close_launch)
