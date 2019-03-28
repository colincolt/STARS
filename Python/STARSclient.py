from threading import Thread, Event
import sys
import numpy as np
import argparse
import time
import socket
from collections import deque

global import_error

show_camera = Event()
# show_camera.set()
try:
    import cv2
    import imutils
    import time
    from picamera.array import PiRGBArray
    from picamera import PiCamera

    import_error = False
except ImportError as imp:
    print("IMPORTANT  :   WITHOUT OPENCV3.0 THE STEREOSCOPICS WILL NOT OPERATE. " + str(imp))
    import_error = True

COLOR = "PINK"

centroid = (0, 0)
compvalue = "1.0"

HOST = '169.254.167.237'  # Define the IP address for communication
PORT = 5025
BUFFER_SIZE = 128
serverPi = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
frame_width = 1008
frame_height = 256
resolution = (frame_width, frame_height)

ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video", help="path to the (optional) video file")
ap.add_argument("-b", "--buffer", type=int, default=8, help="max buffer size")
args = vars(ap.parse_args())

if COLOR == "RED":
    hsvLower = (175, 50, 50)  # currently set for red
    hsvUpper = (180, 255, 255)

if COLOR == "PINK":
    hsvLower = (160, 80, 100)
    hsvUpper = (170, 255, 255)

pts = deque(maxlen=args["buffer"])

def ProcessLoop(vs, PORT, BUFFER_SIZE, HOST, serverPi):
    while True:
        start_time = time.time()

        try:
            image = vs.read()
            image = imutils.resize(image, width=frame_width, height=frame_height)
        except AttributeError as e:
            print("no image")
            capture = False
            while not capture:
                image = vs.read()
                image = imutils.resize(image, width=frame_width, height=frame_height)
                try:
                    image = vs.read()
                    image = imutils.resize(image, width=frame_width, height=frame_height)
                    capture = True
                except AttributeError as e:
                    print("no image")
                    capture = False
                    continue
                except:
                    print("no image")
                    break
        else:
            blurred = cv2.GaussianBlur(image, (11, 11), 0)
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, hsvLower, hsvUpper)
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)

            # find contours in the mask and initialize the current (x, y) center of the ball
            cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cnts = imutils.grab_contours(cnts)
            center = None

            if len(cnts) > 0:
                c = max(cnts, key=cv2.contourArea)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                centroid = (round((M["m10"] / M["m00"]), 3), round((M["m01"] / M["m00"]), 3))
                centroid = (centroid[0] * 3280 / frame_width, centroid[1] * 2464 / frame_height)

                value = '<'+str(int(centroid[0]))
                print("X-coordinate: " + value)
                if show_camera.isSet():
                    ((x, y), radius) = cv2.minEnclosingCircle(c)
                    if radius > 0.5:
                        cv2.circle(image, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                        cv2.circle(image, center, 5, (0, 0, 255), -1)
                    cv2.imshow("Frame", mask)  #  mask
                    key = cv2.waitKey(1) & 0xFF

                # Send Data and Sockets Reconnection loop
                try:
                    serverPi.send((value).encode())
                    #returndata = serverPi.recv(BUFFER_SIZE)
                except socket.error:
                    connected = False
                    print('connection lost... reconnecting')
                    while not connected:
                        try:
                            serverPi = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                            serverPi.connect((HOST, PORT))
                            connected = True
                            print('reconnection successful')
                        except socket.error:
                            print("socket error")
                            time.sleep(2)
                        except Exception as e:
                            print("Some exception:  ", e)  # try:

                #FPS = time.time() - start_time
                #print("FPS: " + str(FPS))

            else:
                print("[StereoClient] : didnt detect anything pink")


class PiVideoStream:
    def __init__(self, resolution=(frame_width, frame_height), framerate=32):
        # initialize the camera and stream
        self.camera = PiCamera()
        self.camera.resolution = resolution
        self.camera.framerate = framerate
        self.rawCapture = PiRGBArray(self.camera, size=resolution)
        self.stream = self.camera.capture_continuous(self.rawCapture, format="bgr", use_video_port=True)

        # initialize the frame and the variable used to indicate
        # if the thread should be stopped
        self.frame = None
        self.stopped = False
        print("[Camera Thread] : Initializing camera")
        time.sleep(1)

    def start(self):
        # start the thread to read frames from the video stream
        Thread(target=self.update, args=()).start()
        return self

    def update(self):
        # keep looping infinitely until the thread is stopped
        for f in self.stream:
            # grab the frame from the stream and clear the stream in
            # preparation for the next frame
            self.frame = f.array
            self.rawCapture.truncate(0)

            # if the thread indicator variable is set, stop the thread
            # and resource camera resources
            if self.stopped:
                self.stream.close()
                self.rawCapture.close()
                self.camera.close()
                return

    def read(self):
        # return the frame most recently read
        return self.frame

    def stop(self):
        # indicate that the thread should be stopped
        self.stopped = True


def read(self):
    # return the frame most recently read
    return self.frame


def stop(self):
    # indicate that the thread should be stopped
    self.stopped = True


# created a *threaded* video stream, allow the camera sensor to warmup,
# and start the FPS counter

print("[INFO] starting THREADED frames from `picamera` module...")
vs = PiVideoStream().start()
time.sleep(2.0)

talking = False
while not talking:
    try:
        serverPi.connect((HOST, PORT))
        talking = True
    except:
        print('Stereoscopics:   No Server')
        time.sleep(3)
        continue
print('connected to serverpi')

ProcessLoop(vs, PORT, BUFFER_SIZE, HOST, serverPi)
