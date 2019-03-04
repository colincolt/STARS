from gpiozero import LED
import time

RED_1 = LED(20)
#RED_2 = LED(21)
#RED_3 = LED(6)
#led_pin = LED(26)
#BLUE = LED(16)
#yellow_led = LED(19)

for i in range(10):
    RED_1.on()
    time.sleep(0.2)
    RED_1.off()
    time.sleep(0.2)
    
RED_1.off()


