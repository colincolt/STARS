
import serial, time
ser = serial.Serial('/dev/ttyUSB0', 9600)
baudrate=9600,
parity=serial.PARITY_NONE,
stopbits=serial.STOPBITS_ONE,
bytesize=serial.EIGHTBITS,
timeout=1

counter="60"
ser.write(counter)
time.sleep(5)
ser.close
