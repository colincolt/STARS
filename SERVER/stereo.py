from threading import Event, Thread, Lock
from collections import deque
import argparse
import time
import socket
import cv2
import imutils
import sys

working_on_the_Pi = False
if working_on_the_Pi:
    try:
        from gpiozero import LED
        from picamera.array import PiRGBArray
        from picamera import PiCamera
    except ImportError as imp:
        print("IMPORTANT  :   ARE YOU WORKING THE RASPBERRY PI ?:  ", imp)
        sys.stdout.flush()
    else:
        GREEN = LED(5)

class data_object:
    def __init__(self, *args):
        self.args = args

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


def StereoscopicsThread(stereo_data):
    focalsize = 3.04e-03
    pixelsize = 1.12e-06
    baseline = 0.737
    stereo_data = stereo_data

    TCP_IP = '169.254.116.12'
    TCP_PORT = 5025
    BUFFER_SIZE = 1024
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    ap = argparse.ArgumentParser()
    ap.add_argument("-v", "--video", help="path to the (optional) video file")
    ap.add_argument("-b", "--buffer", type=int, default=8, help="max buffer size")
    args = vars(ap.parse_args())

    # define the lower and upper boundaries of the jersey ball in the HSV color space, then initialize the list of tracked points
    jerseyLower1 = (0, 70, 70)  # currently set for red
    jerseyUpper1 = (10, 255, 255)
    jerseyLower2 = (170, 70, 70)  # currently set for red
    jerseyUpper2 = (180, 255, 255)
    pts = deque(maxlen=args["buffer"])
    frame_width = 304
    frame_height = 224
    framerate = 10
    resolution = (frame_width, frame_height)
    stereo_loop_count = 1
    print("[Stereo] : trying to connected")
    sys.stdout.flush()

    def connectClient():
        connected = False
        while not connected: # and not shutdown_event.isSet() and not kill_event.isSet():  # Wait for client
            try:
                s.bind((TCP_IP, TCP_PORT))
                s.listen(1)
                clientPort, addr = s.accept()
                connected = True
                print("[Stereo] : Client connected")
                return clientPort
            except socket.error:
                print('[Stereo] :       No Client')
                sys.stdout.flush()
                time.sleep(3)
                continue
            except Exception as e:
                print('[Stereo] :       No Client' + str(e))
                sys.stdout.flush()
                time.sleep(3)
                continue

    def ProcessLoop():  # vs, clientPort, BUFFER_SIZE, frame_width, frame_height, resolution):
        stereo_loop_count = 1
        while True: #not shutdown_event.isSet() and not kill_event.isSet():
            # start_time = time.time()
            if working_on_the_Pi:
                gpio_blinker(GREEN, stereo_loop_count)

            stereo_loop_count = loop_counter(stereo_loop_count)

            try:
                image = vs.read()
                image = imutils.resize(image, width=frame_width, height=frame_height)
            except AttributeError as e:
                print("[Stereo.ProcessLoop] : no image")
                capture = False
                while not capture: # and not shutdown_event.isSet() and not kill_event.isSet():
                    image = vs.read()
                    image = imutils.resize(image, width=frame_width, height=frame_height)
                    try:
                        image = vs.read()
                        image = imutils.resize(image, width=frame_width, height=frame_height)
                        capture = True
                    except AttributeError as e:
                        print("[Stereo.ProcessLoop] : no image")
                        capture = False
                        continue
                    except:
                        PiVideoStream().stop()
                        # shutdown_event.set()

            blurred = cv2.GaussianBlur(image, (11, 11), 0)
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

            mask0 = cv2.inRange(hsv, jerseyLower1, jerseyUpper1)
            mask1 = cv2.inRange(hsv, jerseyLower2, jerseyUpper2)
            mask = mask0 + mask1
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)

            # find contours in the mask and initialize the current (x, y) center of the ball
            cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cnts = imutils.grab_contours(cnts)
            center = None

            # only proceed if at least one contour was found
            if len(cnts) > 0:
                # find the largest contour in the mask, then use it to compute the minimum enclosing circle and centroid
                c = max(cnts, key=cv2.contourArea)
                #                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                centroid = (round((M["m10"] / M["m00"]), 3), round((M["m01"] / M["m00"]), 3))
                centroid = (centroid[0] * 2464 / frame_width, centroid[1] * 2464 / frame_width)
                # only proceed if the radius meets a minimum
            try:
                data = clientPort.recv(BUFFER_SIZE)
                # print("[StereoscopicThread] : received data:", data)
                clientPort.send(data)  # SEND DATA BACK (COULD BE USED FOR STOP COMMAND)
                compvalue = data.decode()
                # print("compvalue:   "+ compvalue)
            except socket.error:
                connected = False
                while not connected:# and not shutdown_event.isSet() and not kill_event.isSet():
                    try:
                        data = clientPort.recv(BUFFER_SIZE)
                        clientPort.send(data)
                        compvalue = data.decode()
                        # print(compvalue)
                        connected = True
                    except socket.error:
                        print("[Stereo] : Lost the client connection")
                        time.sleep(0.5)
                        continue

            if len(cnts) > 0:
                slaveval = float(compvalue)
                masterval = centroid[0]
                disparity = abs(masterval - slaveval)
                distance = (focalsize * baseline) / (disparity * pixelsize)
                # SENDS DATA TO Class, which can be pushed using Stacks's______________________________
                results = data_object
                results.distance = distance
                results.disparity = disparity
                results.masterval = float(masterval)
                # print("[Stereo]: Masterval  :  " + str(masterval))
                results.slaveval = slaveval

                stereo_data.put(results)
                # fps = time.time() - start_time
                # print("[Stereo] :   FPS =  " + str(fps) + "  ||    Distance:  " + str(distance))

        # if shutdown_event.isSet() or kill_event.isSet():
            #            PiVideoStream().stream.close()
            #            PiVideoStream().rawCapture.close()
            #            PiVideoStream().camera.close()
            # PitchYaw(stereoStack, guiStack, megaDataStack, finalDistStack, futureDistStack,shutdown_event, kill_event).shutdown_reset_function()
        print('[Stereo] : Closing Camera...')
        PiVideoStream().stop()

    class PiVideoStream:
        def __init__(self):  # , resolution = (800, 608), framerate = 32):
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
            time.sleep(1)

        def start(self):
            # start the thread to read frames from the video stream
            Thread(target=self.update, args=()).start()
            return self

        def update(self):
            # keep looping infinitely until the thread is stopped
            while True: #not shutdown_event.isSet() and not kill_event.isSet():
                for f in self.stream:
                    # grab the frame from the stream and clear the stream in
                    # preparation for the next frame
                    self.frame = f.array
                    self.rawCapture.truncate(0)

                    if self.stopped:
                        print('[PiVideoStream] : Closing Camera...')
                        self.stream.close()
                        self.rawCapture.close()
                        self.camera.close()
                        time.sleep(3)
                        return

        def read(self):
            # return the frame most recently read
            return self.frame

        def stop(self):
            # indicate that the thread should be stopped
            self.stopped = True

    print("[Stereo] starting THREADED frames from `picamera` module...")
    sys.stdout.flush()
    clientPort = connectClient()
    vs = PiVideoStream().start()
    print("[Stereo] : Initializing camera")
    sys.stdout.flush()
    time.sleep(2.0)  # < Let Video Thread startup
    ProcessLoop()  # vs, clientPort, BUFFER_SIZE, frame_width, frame_height, resolution)
