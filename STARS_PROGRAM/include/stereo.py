from threading import Event, Thread, Lock
from collections import deque
import argparse
import time
import socket
import cv2
import imutils
import sys

# LOCATION OPTIONS ARE:
# - hall
# - gym
# - capstone
LOCATION = "capstone"

working_on_the_Pi = True
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

def gpio_blinker(led_color, loop_count):
    if working_on_the_Pi:
        if loop_count % 2 == 0:
            led_color.on()
        else:
            led_color.off()

def loop_counter(loop_number):
    loop_number += 1
    if loop_number >= 10:
        loop_number = 1
    return loop_number
# _______PITCH AND YAW THREAD________ #


def Stereoscopics(stereo_data, pi_no_pi, led_color, kill_event, show_camera):
    focalsize = 3.04e-03
    pixelsize = 1.12e-06
    baseline = 0.737
    
    stereo_data = stereo_data
    GREEN = led_color
    working_on_the_Pi = pi_no_pi
    kill_event = kill_event
    show_camera = show_camera

    TCP_IP = '169.254.116.12'
    TCP_PORT = 5025
    BUFFER_SIZE = 1024
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    ap = argparse.ArgumentParser()
    ap.add_argument("-v", "--video", help="path to the (optional) video file")
    ap.add_argument("-b", "--buffer", type=int, default=8, help="max buffer size")
    args = vars(ap.parse_args())

    # define the lower and upper boundaries of the jersey ball in the HSV color space, then initialize the list of tracked points
    if LOCATION == "hall":
        jerseyLower1 = (0, 60, 60)  # currently set for red
        jerseyUpper1 = (5, 255, 255)
        jerseyLower2 = (175, 60, 60)  # currently set for red
        jerseyUpper2 = (180, 255, 255)

    if LOCATION == "capstone":
        jerseyLower1 = (0, 50, 50)  # currently set for red
        jerseyUpper1 = (5, 255, 255)
        jerseyLower2 = (175, 50, 50)  # currently set for red
        jerseyUpper2 = (180, 255, 255)

    if LOCATION == "gym":
        print("Havent setup yet")


    pts = deque(maxlen=args["buffer"])
    frame_width = 640
    frame_height = 448
    framerate = 20
    resolution = (frame_width, frame_height)
    stereo_loop_count = 1
    print("[Stereo] : trying to connected")
    sys.stdout.flush()


    # def send_receive():

    def receive_data():
        try:
            data = clientPort.recv(BUFFER_SIZE)
            clientPort.send(data)  # SEND DATA BACK (COULD BE USED FOR STOP COMMAND)
            right_xcoord = data.decode()
            return right_xcoord
        except socket.error:
            print("[Stereo] : missed client data")
            while not kill_event.is_set():  # and not shutdown_event.is_set() and not kill_event.is_set():
                try:
                    data = clientPort.recv(BUFFER_SIZE)
                    clientPort.send(data)
                    right_xcoord = data.decode()
                    return right_xcoord
                except socket.error:
                    print("[Stereo] : Lost the client connection")
                    time.sleep(0.5)
                    continue

    def sendto_queue(left_xcoord,right_xcoord, start_time):
        disparity = abs(left_xcoord - right_xcoord)
        distance = round((focalsize * baseline) / (disparity * pixelsize),2)
        
#        distance_list.appendleft(distance)
#        if len(distance_list) == max_measures:
#            moving_avg = sum(distance_list) / max_measures
#            if distance_list[0] - moving_avg <= 100:
#                distance_list.pop()
#            else:
#                distance_list.popleft()
        # SENDS DATA TO Class, which can be pushed using Stacks's______________________________
        data = str(right_xcoord), ",", str(left_xcoord), ",", str(distance)
        stereo_data.put(data)
        fps = time.time() - start_time
        #print("[Stereo] :   FPS =   ", distance)

    def connectClient():
        connected = False
        while not connected and not kill_event.is_set(): # and not shutdown_event.is_set() and not kill_event.is_set():  # Wait for client
            try:
                s.bind((TCP_IP, TCP_PORT))
                s.listen(1)
                clientPort, addr = s.accept()
                connected = True
                print("[Stereo] : Client connected")
                return clientPort
            except socket.error as se:
                print('[Stereo] :       No Client', se)
                sys.stdout.flush()
                time.sleep(3)
                continue
            except Exception as e:
                print('[Stereo] :       No Client' + str(e))
                sys.stdout.flush()
                time.sleep(3)
                continue
#        sys.exit()

    def ProcessLoop():  # vs, clientPort, BUFFER_SIZE, frame_width, frame_height, resolution):
        stereo_loop_count = 1
        while not kill_event.is_set(): #not shutdown_event.is_set() and not kill_event.is_set():
            start_time = time.time()
            if working_on_the_Pi:
                gpio_blinker(GREEN, stereo_loop_count)

            stereo_loop_count = loop_counter(stereo_loop_count)

            try:
                image = vs.read()
                image = imutils.resize(image, width=frame_width, height=frame_height)
            except AttributeError as e:
                print("[Stereo.ProcessLoop] : no image")
                capture = False
                while not capture and not kill_event.is_set(): # and not shutdown_event.is_set() and not kill_event.is_set():
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

            if len(cnts) > 0:
                c = max(cnts, key=cv2.contourArea)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                centroid = (round((M["m10"] / M["m00"]), 3), round((M["m01"] / M["m00"]), 3))
                centroid = (centroid[0] * 2464 / frame_width, centroid[1] * 2464 / frame_width)

                if show_camera.is_set():
                    ((x, y), radius) = cv2.minEnclosingCircle(c)

                    if radius > 0.5:
                        # draw the circle and centroid on the frame, then update the list of tracked points
                        cv2.circle(image, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                        cv2.circle(image, center, 5, (0, 0, 255), -1)

                    cv2.imshow("Frame", image)
                    key = cv2.waitKey(1) & 0xFF

                right_xcoord = receive_data()
                #fps = time.time() - start_time
                #print("FPS =  ",fps)

                try:
                    right_xcoord = float(right_xcoord)
                    left_xcoord = centroid[0]
                    sendto_queue(left_xcoord,right_xcoord,start_time)
                except ValueError as val:
                    print("Value Error:  ->  ",val)
        

        if kill_event.is_set():
            print("[Stereo] : closing socket connection")
            PiVideoStream().stop()
            clientPort.close()
            #            clientPort.shutdown()

            print('[Stereo] : Closing Camera...')


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
            # while not kill_event.is_set(): #not shutdown_event.is_set() and not kill_event.is_set():
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


if __name__ == "__main__":
    import multiprocessing as mp

    stereo_data = mp.Queue()
    kill_event = Event()
    working_on_the_Pi = True
    if working_on_the_Pi:
        GREEN = LED(16)

    mp.Process(target=Stereoscopics, args=[stereo_data, working_on_the_Pi, GREEN, kill_event]).start()

    while True:
        data = stereo_data.get()
        tempData="".join(data)
        tempData = tempData.strip("<")
        tempData = tempData.strip(">")
        tempData = tempData.split(",")
        RightXcoord = int(float(tempData[0]))
        LeftXcoord = int(float(tempData[1]))
        stereoDist = float(tempData[2])

        print("RightXcoord:  ",RightXcoord,"  LeftXcoord:  ", LeftXcoord,"  stereoDist:  ",stereoDist)






