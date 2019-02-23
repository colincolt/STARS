from picamera import PiCamera
from time import sleep
import cv2

k = cv2.waitKey(1) & 0xFF

while True:
    camera = PiCamera()
    camera.start_preview()
    sleep(10)
    camera.stop_preview()

    if k == ord('q'):
        break
    # elif k == ord('b'):
    #     # change a variable / do something ...
    # elif k == ord('k'):
    # # change a variable / do something ...