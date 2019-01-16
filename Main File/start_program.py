# _________________________________READ ME______________________________________#
# THREADING:
# This file uses the python "threading" module to run any number of "Threads"
# simultaneously, anything that needs to run continuously can be put into an
# infinite loop, anything that needs to run independantly and not interfere
# with other tasks. THREADS are ideal for input output operations, we do not
# have a lot of math being done.
#
# QUEUE:
# Notice the use of Queues to input output data to and from threads, read the
# documentation to get a better understanding (https://docs.python.org/3/library/asyncio-queue.html)
#
# BASIC PROGRAM STRUCTURE:
# for now at least the way that this is structured has the Stereoscopic code
# run in the main thread/program, this means LIMIT any code outside of the
# threads, otherwise the FPS of the algorithm is compromised

# PROGRAM OPTIONS:


# Stereo packages
from collections import deque
from imutils.video import VideoStream
import numpy as np
import argparse
import cv2
import imutils
import time
import socket
import struct
import math
import signal
from threading import *
from queue import LifoQueue

class StereoOutput:
    def __init__(self, p1, p2, p3, p4):
        self.p1 = p1
        self.p2 = p2
        self.p3 = p3
        self.p4 = p4


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
         if not(self.isEmpty()):
             return self.items[len(self.items)-1]

         # TODO: CRAIG PLEASE LET ME KNOW WHAT TO DO
         return 0

     def size(self):
         return len(self.items)


# #____________________________________THREADS__________________________________###

# PITCH_YAW_THREAD: communicates to the Arduino Uno in order to provide Angle
# values to both motors, request temperature's from the Uno, and distance measurements
# from the Arduino Mega or Serial.
# These distance measurements are also provided to the LAUNCHER THREAD

class PitchYaw(Thread):
    def __init__(self, getAngleStack, getDistanceStack):
        Thread.__init__(self)
        self.getAngleStack = getAngleStack
        self.getDistanceStack = getDistanceStack

    def run(self):
        print('Starting Pitch Yaw thread')
        while True:
            try:
                Angle = self.getAngleStack.peek()
                distance = self.getDistanceStack.peek()
                leftXcoord = self.
                rightXcoord = 

                # TODO: insert MATH to get reliable distance then use distance with pitch motor

            except Exception as e:
                print('Pitch yaw thread failed because of exception ' + e)
                continue


## Sterepscopic thread
class Stereoscopics(Thread):
    def __init__(self, difficulty, drillType, args, sendAngleStack):
        Thread.__init__(self)
        self.difficulty = difficulty
        self.drillType = drillType
        self.sendAngleStack = sendAngleStack
        self.args = args

    def run(self):
        print('Starting Stereoscopics thread')

        # subprocess.call(["source", "~/.profile"])
        # subprocess.call(["workon", "cv"])

        focalsize = 3.04e-03
        pixelsize = 1.12e-06
        baseline = 0.737
        datapoints = 5
        centroid = (0,0)
        masterval = 1.1
        compvalue = 1

        TCP_IP = '192.168.137.200'
        TCP_PORT = 5005
        BUFFER_SIZE = 20
        no_talk=True

        args = self.args

        ##________________Sockets TCP/IP Communication__________________________________###########
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        while no_talk == True:      # Wait for client
            try:
                s.bind((TCP_IP, TCP_PORT))
                s.listen(1)
                conn, addr = s.accept()
                no_talk = False
            except:
                print('Stereoscopics:       No Client')
                time.sleep(5)
                continue


        # define the lower and upper boundaries of the jersey
        # ball in the HSV color space, then initialize the
        # list of tracked points
        jerseyLower1 = (0,50,50) #currently set for red
        jerseyUpper1 = (10,255,255)
        jerseyLower2 = (170, 50, 50) #currently set for red
        jerseyUpper2 = (180, 255, 255)
        pts = deque(maxlen=args["buffer"])

        #initialize array of distance values
        distvals = []

        # if a video path was not supplied, grab the reference
        # to the webcam
        if not args.get("video", False):
            vs = VideoStream(src=0).start()

        # otherwise, grab a reference to the video file
        else:
            vs = cv2.VideoCapture(args["video"])

        # allow the camera or video file to warm up
        time.sleep(2.0)

        ##________________BEGINNING OF LOOP________________##
        while True:
            print('starting stereo loop')
            # grab the current frame
            # start = time.time()
            start = time.time()
            frame = vs.read()

            # handle the frame from VideoCapture or VideoStream
            frame = frame[1] if args.get("video", False) else frame

            # if we are viewing a video and we did not grab a frame,
            # then we have reached the end of the video
            if frame is None:
                break

            # resize the frame, blur it, and convert it to the HSV
            # color space
            frame = imutils.resize(frame, width=600)
            blurred = cv2.GaussianBlur(frame, (11, 11), 0)
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

            # construct a mask for the color "green", then perform
            # a series of dilations and erosions to remove any small
            # blobs left in the mask
            mask0 = cv2.inRange(hsv, jerseyLower1, jerseyUpper1)
            mask1 = cv2.inRange(hsv, jerseyLower2, jerseyUpper2)
            mask = mask0+mask1
            # greenLower = (29, 86, 6)
            #   greenUpper = (64, 255, 255)
            #   mask = cv2.inRange(hsv, greenLower, greenUpper)
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)

            # find contours in the mask and initialize the current
            # (x, y) center of the ball
            cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                    cv2.CHAIN_APPROX_SIMPLE)
            cnts = imutils.grab_contours(cnts)
            center = None
            # centroid = None

            # only proceed if at least one contour was found
            if len(cnts) > 0:
                # find the largest contour in the mask, then use it to compute the minimum enclosing circle and centroid
                c = max(cnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                # print("Center: " + center[0])
                centroid = (round((M["m10"] / M["m00"]),3), round((M["m01"] / M["m00"]),3))
                centroid = (centroid[0]*2464/600, centroid[1]*2464/600)
                # print("Centroid: " + str(centroid[0]))

                # only proceed if the radius meets a minimum size
                if radius > 0.1:
                    # draw the circle and centroid on the frame,
                    # then update the list of tracked points
                    cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                    cv2.circle(frame, center, 5, (0, 0, 255), -1)

            # update the points queue
            pts.appendleft(center)
            # print("center appended")

            # loop over the set of tracked points
            for i in range(1, len(pts)):
                    # if either of the tracked points are None, ignore
                    # them
                    if pts[i - 1] is None or pts[i] is None:
                            continue

                    # otherwise, compute the thickness of the line and
                    # draw the connecting lines
                    thickness = int(np.sqrt(args["buffer"] / float(i + 1)) * 2.5)
                    cv2.line(frame, pts[i - 1], pts[i], (0, 0, 255), thickness)

            while 1:
                data = conn.recv(BUFFER_SIZE)
                if not data: break
                # print ("received data:", data)
                conn.send(data) #echo
                compvalue = data.decode()
                break

            if len(cnts) > 0:
                #print("Decoded value: " + compvalue)
                slaveval = float(compvalue)
                masterval = centroid[0]
                # masterval = x*2464/600
                # disparity = abs(float(compvalue)-centroid[0])
                disparity = abs(masterval-slaveval)
                distance = (focalsize*baseline)/(disparity*pixelsize)
                # SENDS DATA TO Class, which can be "Put" using Queue's______________________________
                results = StereoOutput()
                results.distance = distance
                results.disparity = disparity
                results.masterval = masterval
                results.slaveval = slaveval
                self.sendAngleStack.push(results)
                # ##_______________________________________________
                distvals.append(distance)
                length = len(distvals)
                sumweights = 0
                weight = 0
                denominator = 0
                if length < datapoints:
                    average = sum(distvals)/len(distvals)
                if length > datapoints:
                    distvals.pop(0)
                    for i in range(0, datapoints-1):
                        weight = ((i+1)*distvals[i])
                        sumweights = sumweights + weight
                    for j in range(1, datapoints):
                        denominator = denominator + j
                    average = sumweights/denominator

              # average = sum(distvals)/len(distvals)
              # show the frame to our screen
                cv2.putText(frame, "Center: " + str(center), (50,50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50), 2)
                cv2.putText(frame, "Radius: " + str(int(radius)), (50,100), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50), 2)
                cv2.putText(frame, "Distance: " + str(round(average,2)), (50,150), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50), 2)

            #cv2.imshow("Frame", frame)

        # if we are not using a video file, stop the camera video stream
        if not args.get("video", False):
            vs.stop()

        # otherwise, release the camera
        else:
            vs.release()

        end = time.time()
        # print(end-start)

        # close all windows
        cv2.destroyAllWindows()
        conn.close()

def startLauncher(speed, sendVoiceCommandStack, sendDistanceStack): ## Connects and communicates with the Arduino Mega: Launcher Motors, Ball Feeder, Wifi, Accelerometer? (Rangefinder?)
    currRestPos = ""
    currDistance = ""

    while True:
        try:
            # newRestPos = getRestingPosition()
            if newRestPos:
                currRestPos = newRestPos

            # newDistance = getDistance()
            if newDistance:
                currDistance = newDistance

            # TODO: do calculation and stuff
            calc = ""
            sendVoiceCommandStack.push(calc)
            sendDistanceStack.push(self.currDistance)

        except Exception as e:
            print('Launcher thread failed because of exception ' + e)
            continue


# MAIN FILE START
def startMainFile(speed, difficulty, drillType, args): ## NOT A THREAD, performs the bulk of calculation

    # NOTE: cannot gets args from command line so must be passed in via the gui
    # # construct the argument parse and parse the arguments
    # ap = argparse.ArgumentParser()
    # ap.add_argument("-v", "--video",
    #     help="path to the (optional) video file")
    # ap.add_argument("-b", "--buffer", type=int, default=8,
    #     help="max buffer size")
    # args = vars(ap.parse_args())

    # # _______________________________Main Processing_____________________________________# #
    AngleStack = Stack()
    distanceStack = Stack()
    voiceCommandStack = Stack()
    # Arduino Mega Port:
    Mega=serial.Serial("/dev/ttyUSB1",115200)        #Arduino Mega
    Mega.baudrate=115200
    # Terabee Rangefinder Port:
    evo = serial.Serial(portname, baudrate=115200, timeout=2)   ##MANUALY INPUT the 'portname' /tty/.../
    set_text = (0x00, 0x11, 0x01, 0x45)
    evo.flushInput()
    evo.write(set_text)
    evo.flushOutput()
    # Start Threads
    pitchYawthread = PitchYaw(AngleStack, distanceStack)
    pitchYawthread.start()
    stereoscopicsThread = Stereoscopics(difficulty, drillType, args, AngleStack)
    stereoscopicsThread.start()
    startLauncherThread = startLauncher(speed, voiceCommandStack, distanceStack)
    startLauncherThread.start()
    


