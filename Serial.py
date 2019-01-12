import serial
import time

ser=serial.Serial("/dev/ttyACM0",115200)  #change ACM number as found from "dmesg -s 1024"
ser.baudrate=115200

#READ from Serial Port "ser"
read_ser=ser.readline()
print(read_ser)

#WRITE to Serial Port "ser"
ser.write('5')

sys.exit()


