from threading import Thread
import sys
import numpy as np
import argparse
import time
import socket
from collections import deque

global import_error

try:
    import cv2
    import imutils
    import io
    import time
    from picamera.array import PiRGBArray
    from picamera import PiCamera
    import_error = False
except ImportError as imp:
    print("IMPORTANT  :   WITHOUT OPENCV3.0 THE STEREOSCOPICS WILL NOT OPERATE" + str(imp))
    import_error = True

centroid = (0, 0)
compvalue = "1.0"

HOST = '169.254.116.12' # Define the IP address for communication
PORT = 5025
BUFFER_SIZE = 1024
talking = False
serverPi = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

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

while not talking:
    try:
        serverPi.connect((HOST, PORT))
        talking = True
    except:
        print('Stereoscopics:   No Server')
        time.sleep(3)
        continue

connected = True
print('connected to serverpi')

def ProcessLoop(vs, PORT, BUFFER_SIZE, HOST, serverPi):
    while True:
        print("looping")
        start_time = time.time()

        ## CHANGE THE WAY YOU ACQUIRE IMAGE
        try:
            image = vs.read()
            # image = np.frombuffer(image, dtype=np.uint8)
            image = imutils.resize(image, width=600, height=450)
        except AttributeError as e:
            print("no image")
            capture = False
            while not capture:
                image = vs.read()
                # image = np.frombuffer(image, dtype=np.uint8)
                image = imutils.resize(image, width=600, height=450)
                try:
                    image = vs.read()
                    # image = np.frombuffer(image, dtype=np.uint8)
                    image = imutils.resize(image, width=600, height=450)
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
        # find the largest contour in the mask, then use it to compute the minimum
        # enclosing circle and centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            centroid = (round((M["m10"] / M["m00"]),3), round((M["m01"] / M["m00"]),3))
            centroid = (centroid[0]*2464/600, centroid[1]*2464/600)
        # only proceed if the radius meets a minimum size
            if radius > 0.1:
    #        # draw the circle and centroid on the frame,then update the list of tracked points
                cv2.circle(image, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                cv2.circle(image, center, 5, (0, 0, 255), -1)
    # 
    ## update the points queue
        pts.appendleft(center)
    ## loop over the set of tracked points
        for i in range(1, len(pts)):
    #    # if either of the tracked points are None, ignore them
                if pts[i - 1] is None or pts[i] is None:
                        continue
    #    # otherwise, compute the thickness of the line and draw the connecting lines
                thickness = int(np.sqrt(args["buffer"] / float(i + 1)) * 2.5)
                cv2.line(image, pts[i - 1], pts[i], (0, 0, 255), thickness)


        # show the frame to our screen
        # hypotenuse = ((center[0])**(2)+(center[1])**(2))**(0.5)
        if len(cnts) > 0:
            cv2.putText(image, "Center : " + str(center) , (50,50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50), 2)
    #    # cv2.putText(image, "Hypotenuse : " + str(int(hypotenuse)), (50,100), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50), 2)
            cv2.putText(image, "Radius : " + str(int(radius)), (50,100), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50), 2)
    #
        cv2.imshow("Frame", image)
        key = cv2.waitKey(1) & 0xFF
        
        if len(cnts) > 0:
            value = str(centroid[0])
            #value = str(x)
            print ("X-coordinate: " + value)
            # Send Data and Sockets Reconnection loop
            try:
                serverPi.send((value).encode())
                data = serverPi.recv(BUFFER_SIZE)
            except socket.error:
                connected = False
                serverPi = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                print('connection lost... reconnecting')
                while not connected:
                    try:
                        serverPi.connect((HOST, PORT))
                        connected = True
                        print('reconnection successful')
                    except socket.error:
                        time.sleep(2)
                    
class PiVideoStream:
    def __init__(self, resolution=(600, 450), framerate=32):
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

ProcessLoop(vs, PORT, BUFFER_SIZE, HOST, serverPi)
##________________BEGINNING OF LOOP________________##

