from threading import Thread
from multiprocessing import Process, Queue
import sys
import numpy as np
import argparse
import time
import socket
import selectors
from collections import deque

global import_error

try:
    import cv2
    import imutils
    import io
    from picamera.array import PiRGBArray
    from picamera import PiCamera
    import_error = False
except ImportError as imp:
    print("IMPORTANT  :   WITHOUT OPENCV3.0 THE STEREOSCOPICS WILL NOT OPERATE" + str(imp))
    import_error = True

focalsize = 3.04e-03
pixelsize = 1.12e-06
baseline = 0.737
datapoints = 5
frame_width = 320
frame_height = 224
centroid = (0, 0)
compvalue = "1.0"

TCP_IP = '169.254.116.12'
TCP_PORT = 5025
BUFFER_SIZE = 1024
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sel = selectors.DefaultSelector()

ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video", help="path to the (optional) video file")
ap.add_argument("-b", "--buffer", type=int, default=8, help="max buffer size")
args = vars(ap.parse_args())

# define the lower and upper boundaries of the jersey ball in the HSV color space, then initialize the list of tracked points
jerseyLower1 = (0, 50, 50)  # currently set for red
jerseyUpper1 = (10, 255, 255)
jerseyLower2 = (170, 50, 50)  # currently set for red
jerseyUpper2 = (180, 255, 255)
pts = deque(maxlen=args["buffer"])

connected = False
print('[Stereo] :       before connection loop')
while not connected:  # Wait for client
    try:
        s.bind((TCP_IP, TCP_PORT))
        s.listen(1)
        # sock.settimeout(5)
        clientPort, addr = s.accept()
        s.setblocking(False)
        sel.register(s, selectors.EVENT_READ, data = None)
        connected = True
        print("[Stereo] : Client connected")
    except socket.error:
        print('[Stereo] :       No Client')
        time.sleep(3)
        continue
    except Exception as e:
        print('[Stereo] :       No Client'+str(e))
        time.sleep(3)
        continue

def recvData(center,q):
    try:
        #print('[StereoscopicThread] : waiting for data')
#            print("test1")
        data = clientPort.recv(BUFFER_SIZE)
#            print("test2")
        #print("[StereoscopicThread] : received data:", data)
        clientPort.send(data)  # SEND DATA BACK (COULD BE USED FOR STOP COMMAND)
#            print("test3")
        recv_val = data.decode()
#        print("Client value = " + recv_val)
        q.put(recv_val)
#            print("test4")
    except socket.error:
        connected = False
        while not connected: #and not shutdown_event.isSet() and not kill_event.isSet():
            try:
                data = clientPort.recv(BUFFER_SIZE)
                print("[StereoscopicThread] : received data:", data)
                clientPort.send(data)  # SEND DATA BACK (COULD BE USED FOR STOP COMMAND)
                recv_val = data.decode()
#                print("Client value = " + recv_val)
                return recv_val
                connected = True
            except socket.error:
                print("[Stereo] : Lost the client connection")
                time.sleep(0.5)
    
def ProcessLoop(vs, clientPort, BUFFER_SIZE):
##________________BEGINNING OF LOOP________________##
    while True:
#        print("looping")
        start_time = time.time()
        q = Queue()

        ## CHANGE THE WAY YOU ACQUIRE IMAGE
        try:
            image = vs.read()
            # image = np.frombuffer(image, dtype=np.uint8)
            image = imutils.resize(image, width=frame_width, height=frame_height)
        except AttributeError as e:
            print("no image")
            capture = False
            while not capture:
                image = vs.read()
                # image = np.frombuffer(image, dtype=np.uint8)
                image = imutils.resize(image, width=frame_width, height=frame_height)
                try:
                    image = vs.read()
                    # image = np.frombuffer(image, dtype=np.uint8)
                    image = imutils.resize(image, width=frame_width, height=frame_height)
                    capture = True
                except AttributeError as e:
                    print("no image")
                    capture = False
                    continue
                except:
                    print("no image")
                    break

        
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
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            centroid = (round((M["m10"] / M["m00"]), 3), round((M["m01"] / M["m00"]), 3))
            centroid = (centroid[0] * 2464 / frame_width, centroid[1] * 2464 / frame_height)

            # only proceed if the radius meets a minimum size
            if radius > 0.1:
                # draw the circle and centroid on the frame, then update the list of tracked points
                cv2.circle(image, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                cv2.circle(image, center, 5, (0, 0, 255), -1)

        # update the points queue
        pts.appendleft(center)
        
        #print("test1")
        # loop over the set of tracked points
#        for i in range(1, len(pts)):
#            # if either of the tracked points are None, ignore
#            # them
#            if pts[i - 1] is None or pts[i] is None:
#                continue
#
#            # otherwise, compute the thickness of the line and draw the connecting lines
#            thickness = int(np.sqrt(args["buffer"] / float(i + 1)) * 2.5)
#            cv2.line(image, pts[i - 1], pts[i], (0, 0, 255), thickness)
        #print("test2")
        tcp = Process(target=recvData, args = (center,q))
        tcp.start()
        compvalue = q.get(True)
#        print("Client value = " + compvalue)

#        try:
#            #print('[StereoscopicThread] : waiting for data')
##            print("test1")
#            data = clientPort.recv(BUFFER_SIZE)
##            print("test2")
#            #print("[StereoscopicThread] : received data:", data)
#            clientPort.send(data)  # SEND DATA BACK (COULD BE USED FOR STOP COMMAND)
##            print("test3")
#            compvalue = data.decode()
#            print(compvalue)
##            print("test4")
#        except socket.error:
#            connected = False
#            while not connected: #and not shutdown_event.isSet() and not kill_event.isSet():
#                try:
#                    data = clientPort.recv(BUFFER_SIZE)
#                    print("[StereoscopicThread] : received data:", data)
#                    clientPort.send(data)  # SEND DATA BACK (COULD BE USED FOR STOP COMMAND)
#                    compvalue = data.decode()
#                    print(compvalue)
#                    connected = True
#                except socket.error:
#                    print("[Stereo] : Lost the client connection")
#                    time.sleep(0.5)
                    
        cv2.imshow("Frame", image)
        key = cv2.waitKey(1) & 0xFF
        
        if len(cnts) > 0:
            #print("Decoded value: " + compvalue)
            slaveval = float(compvalue)
            masterval = centroid[0]
            disparity = abs(masterval - slaveval)
            distance = (focalsize * baseline) / (disparity * pixelsize)
            # SENDS DATA TO Class, which can be "Put" using Stacks's______________________________
            # results = StereoOutput
            # results.distance = distance
            # results.disparity = disparity
            # results.masterval = masterval
            # results.slaveval = slaveval
            # stereoStack.push(results)
            fps = time.time() - start_time
            print("[Stereo] :   FPS =  " + str(fps) + "||    Distance:  " + str(distance))
        
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

# Allow the camera to warmup:
time.sleep(2.0)

ProcessLoop(vs, clientPort, BUFFER_SIZE)
