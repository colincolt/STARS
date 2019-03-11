# Imports
import cv2
import time
import threading
import imutils
import argparse
import sys
from collections import deque
import numpy as np
import socket
import io



# Camera settings go here
imageWidth = 640
imageHeight = 400
frameRate = 20
processingThreads = 4

# Shared values
global running
global cap
global frameLock
global processorPool
running = True
frameLock = threading.Lock()

COLOR = "PINK"

HOST = '169.254.116.12'
PORT = 5025
BUFFER_SIZE = 24
serverPi = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Setup the camera
cap = cv2.VideoCapture(0)
#cap.set(cv2.CAP_PROP_FRAME_WIDTH, imageWidth);
#cap.set(cv2.CAP_PROP_FRAME_HEIGHT, imageHeight);
cap.set(cv2.CAP_PROP_FPS, frameRate);
if not cap.isOpened():
    cap.open()

ap = argparse.ArgumentParser()
ap.add_argument("-b", "--buffer", type=int, default=8,
                help="max buffer size")
args = vars(ap.parse_args())

# define the lower and upper boundaries of the "green"
# ball in the HSV color space, then initialize the
# list of tracked points
if COLOR == "PINK":
    hsvLower = (160, 60, 60)
    hsvUpper = (170, 255, 255)
if COLOR == "RED":
    hsvLower = (0, 50, 50)  # currently set for red
    hsvUpper = (5, 255, 255)
pts = deque(maxlen=args["buffer"])

class Stack:
    def __init__(self):
        self.items = []

    def isEmpty(self):
        return self.items == []

    def push(self, item):
        self.items.append(item)

    def pop(self):
        return self.items.pop()

    def peek(self):
        if not (self.isEmpty()):
            return self.items[len(self.items) - 1]
        else:
            return None

    def size(self):
        return len(self.items)

output_data = Stack()

# Image processing thread, self-starting
class ImageProcessor(threading.Thread):
    def __init__(self, name, output_data, autoRun=True):
        super(ImageProcessor, self).__init__()
        self.event = threading.Event()
        self.output_data = output_data
        self.eventWait = (2.0 * processingThreads) / frameRate
        self.name = str(name)
        print('Processor thread %s started with idle time of %.2fs' % (self.name, self.eventWait))
        self.start()

    def run(self):
        # This method runs in a separate thread
        global running
        global frameLock
        global processorPool
        while running:
            # Wait for an image to be written to the stream
            self.event.wait(self.eventWait)
            if self.event.isSet():
                if not running:
                    break
                try:
                    self.ProcessImage(self.nextFrame)
                finally:
                    # Reset the event
                    self.nextFrame = None
                    self.event.clear()
                    # Return ourselves to the pool at the back
                    with frameLock:
                        processorPool.insert(0, self)

        print('Processor thread %s terminated' % (self.name))

    def ProcessImage(self, image):
        frame = image

        # handle the frame from VideoCapture or VideoStream
        frame = frame[1] if args.get("video", False) else frame

        # resize the frame, blur it, and convert it to the HSV
        # color space
        frame = imutils.resize(frame, width=imageWidth, height=imageHeight)
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # construct a mask for the color "green", then perform
        # a series of dilations and erosions to remove any small
        # blobs left in the mask
        mask = cv2.inRange(hsv, hsvLower, hsvUpper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        # find contours in the mask and initialize the current
        # (x, y) center of the ball
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        center = None

        # only proceed if at least one contour was found
        if len(cnts) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(cnts, key=cv2.contourArea)
#            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            centroid = (round((M["m10"] / M["m00"]), 3), round((M["m01"] / M["m00"]), 3))
            centroid = (centroid[0] * 2464 / imageWidth, centroid[1] * 2464 / imageHeight)

            self.output_data.push(centroid[0])

            # only proceed if the radius meets a minimum size

#            if radius > 10:
#                # draw the circle and centroid on the frame,
#                # then update the list of tracked points
#                cv2.circle(frame, (int(x), int(y)), int(radius),
#                           (0, 255, 255), 2)
#                cv2.circle(frame, center, 5, (0, 0, 255), -1)
#

# update the points queue
#        pts.appendleft(center)
#            # loop over the set of tracked points
#        for i in range(1, len(pts)):
#            # if either of the tracked points are None, ignore
#            # them
#            if pts[i - 1] is None or pts[i] is None:
#                continue
#
#            # otherwise, compute the thickness of the line and
#            # draw the connecting lines
#            thickness = int(np.sqrt(args["buffer"] / float(i + 1)) * 2.5)
#            cv2.line(frame, pts[i - 1], pts[i], (0, 0, 255), thickness)

# show the frame to our screen
#        cv2.imshow("Frame", frame)
#        key = cv2.waitKey(1) & 0xFF

# Processing for each image goes here
### TODO ###

# Image capture thread, self-starting
class ImageCapture(threading.Thread):
    def __init__(self):
        super(ImageCapture, self).__init__()
        self.start()

    # Stream delegation loop
    def run(self):
        # This method runs in a separate thread
        global running
        global cap
        global processorPool
        global frameLock
        while running:
            # Grab the oldest unused processor thread
            with frameLock:
                if processorPool:
                    processor = processorPool.pop()
                else:
                    processor = None
            if processor:
                # Grab the next frame and send it to the processor
                success, frame = cap.read()
                if success:
                    processor.nextFrame = frame
                    processor.event.set()
                else:
                    print('Capture stream lost...')
                    running = False
            else:
                # When the pool is starved we wait a while to allow a processor to finish
                time.sleep(0.01)
        print('Capture thread terminated')



talking = False
while not talking:
    try:
        serverPi.connect((HOST, PORT))
        talking = True
    except:
        print('Stereoscopics:   No Server')
        time.sleep(2)
        continue
print('connected to serverpi')

# Create some threads for processing and frame grabbing
processorPool = [ImageProcessor(i + 1, output_data) for i in range(processingThreads)]
allProcessors = processorPool[:]
captureThread = ImageCapture()

# Main loop, basically waits until you press CTRL+C
# The captureThread gets the frames and passes them to an unused processing thread
frame_counter = 0
try:
    print('Press CTRL+C to quit')
    start_time = time.time()
    while running:
        try:
            right_x = output_data.pop()
            right_x = "<"+str(right_x)
            print(right_x)
            try:
                serverPi.send(right_x.encode())
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
                        print("Some exception:  ", e)

        except IndexError as i:
            pass

        # time.sleep(1)
except KeyboardInterrupt:
    print('\nUser shutdown')
except:
    e = sys.exc_info()
    print()
    print(e)
    print('\nUnexpected error, shutting down!')

# Cleanup all processing threads
running = False
while allProcessors:
    # Get the next running thread
    with frameLock:
        processor = allProcessors.pop()
    # Send an event and wait until it finishes
    processor.event.set()
    processor.join()

# Cleanup the capture thread
captureThread.join()

# Cleanup the camera object
cap.release()