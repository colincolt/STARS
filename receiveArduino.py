import RPi.GPIO as GPIO
from lib_nrf24 import NRF24
import time
import spidev

GPIO.setmode(GPIO.BCM)

pipes = 0xF0F0F0F0E1LL

radio = NRF24(GPIO, spidev.SpiDev())
radio.begin(0, 17)

radio.setPayloadSize(8)
radio.setChannel(0x76)
radio.setDataRate(NRF24.BR_2MBPS)
radio.setPALevel(NRF24.PA_MAX)

radio.setAutoAck(True)
radio.enableDynamicPayloads()
radio.enableAckPayload()

radio.openReadingPipe(1, pipes)
radio.printDetails()
radio.startListening()

while True:

    while not radio.available(0):
        time.sleep(1/100)

    receivedMessage = []
    radio.read(receivedMessage, radio.getDynamicPayloadSize())
    print("Received: {}".format(receivedMessage))

    print("Translating our received Message into unicode characters...")
    string = ""

    for n in receivedMessage:
        if (n >= 7 and n <= 126):
            string += chr(n)
    print("Our received message decodes to: {}".format(string))

#>
