from imutils.video import VideoStream
from collections import deque
import numpy as np
import argparse
import cv2
import imutils
import time
import socket
import serial
from threading import *
import serial.tools.list_ports
import sys
import os
import binascii
import smtplib
import statistics
from picamera.array import PiRGBArray
from picamera import PiCamera

focalsize = 3.04e-03
pixelsize = 1.12e-06
baseline = 0.737
datapoints = 5
centroid = (0, 0)
compvalue = 1

TCP_IP = '169.254.116.12'
TCP_PORT = 5025
BUFFER_SIZE = 1024
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

connected = False
while not connected:  # Wait for client
    try:
        s.bind((TCP_IP, TCP_PORT))
        s.listen(1)
        # sock.settimeout(5)
        clientPort, addr = s.accept()
        connected = True
    except socket.error:
        print('Stereoscopics:       No Client')
        time.sleep(3)
        continue

connected = True

print("Client connected")

ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video",
                help="path to the (optional) video file")
ap.add_argument("-b", "--buffer", type=int, default=8, help="max buffer size")
args = vars(ap.parse_args())
##________________Sockets TCP/IP Communication__________________________________###########

# define the lower and upper boundaries of the jersey ball in the HSV color space, then initialize the list of tracked points
jerseyLower1 = (0, 50, 50)  # currently set for red
jerseyUpper1 = (5, 255, 255)
jerseyLower2 = (175, 50, 50)  # currently set for red
jerseyUpper2 = (180, 255, 255)
pts = deque(maxlen=args["buffer"])

frameRate = 10
framesize=(600, 450)
# initialize array of distance values
distvals = []

vs = VideoStream(src=0, resolution = framesize, framerate = frameRate).start()

time.sleep(0.1)
print('starting stereo loop')

##________________BEGINNING OF LOOP________________##
while True:
    # grab the current frame
    image = vs.read()
    
    #image = cv2.imdecode(np.fromstring(frame.getvalue(), dtype=np.uint8), 1)
    blurred = cv2.GaussianBlur(image, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    mask0 = cv2.inRange(hsv, jerseyLower1, jerseyUpper1)
    mask1 = cv2.inRange(hsv, jerseyLower2, jerseyUpper2)
    mask = mask0 + mask1
    # greenLower = (29, 86, 6)
    # greenUpper = (64, 255, 255)
    # mask = cv2.inRange(hsv, greenLower, greenUpper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    # find contours in the mask and initialize the current (x, y) center of the ball
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
        centroid = (round((M["m10"] / M["m00"]), 3), round((M["m01"] / M["m00"]), 3))
        centroid = (centroid[0] * 2464 / 600, centroid[1] * 2464 / 600)
        # print("Centroid: " + str(centroid[0]))

        # only proceed if the radius meets a minimum size
        if radius > 0.1:
            # draw the circle and centroid on the frame, then update the list of tracked points
            cv2.circle(image, (int(x), int(y)), int(radius), (0, 255, 255), 2)
            cv2.circle(image, center, 5, (0, 0, 255), -1)

    # update the points queue
    pts.appendleft(center)

    # loop over the set of tracked points
    for i in range(1, len(pts)):
        # if either of the tracked points are None, ignore
        # them
        if pts[i - 1] is None or pts[i] is None:
            continue

        # otherwise, compute the thickness of the line and draw the connecting lines
        thickness = int(np.sqrt(args["buffer"] / float(i + 1)) * 2.5)
        cv2.line(image, pts[i - 1], pts[i], (0, 0, 255), thickness)

    try:
        print('in the try')
        data = clientPort.recv(BUFFER_SIZE)
        print ("received data:", data)
        clientPort.send(data)           # SEND DATA BACK (COULD BE USED FOR STOP COMMAND)
        compvalue = data.decode()
    except socket.error:
        connected = False
        while not connected:
            try:
                data = clientPort.recv(BUFFER_SIZE)
                print ("received data:", data)
                clientPort.send(data)           # SEND DATA BACK (COULD BE USED FOR STOP COMMAND)
                compvalue = data.decode()
            except socket.error:
                time.sleep(2)

    if len(cnts) > 0:
        # print("Decoded value: " + compvalue)
        slaveval = float(compvalue)
        masterval = centroid[0]
        # masterval = x*2464/600
        # disparity = abs(float(compvalue)-centroid[0])
        disparity = abs(masterval - slaveval)
        distance = (focalsize * baseline) / (disparity * pixelsize)
        print(distance)
        # SENDS DATA TO Class, which can be "Put" using Queue's______________________________
#        results = StereoOutput
#        results.distance = distance
#        results.disparity = disparity
#        results.masterval = masterval
#        results.slaveval = slaveval
#        stereoStack.push(results)
        # _____________DISPLAY VIDEO AND MARKERS___________________________ #
        distvals.append(distance)
        length = len(distvals)
        sumweights = 0
        weight = 0
        denominator = 0
        if length < datapoints:
            average = sum(distvals) / len(distvals)
        if length > datapoints:
            distvals.pop(0)
            for i in range(0, datapoints - 1):
                weight = ((i + 1) * distvals[i])
                sumweights = sumweights + weight
            for j in range(1, datapoints):
                denominator = denominator + j
            average = sumweights / denominator

        # average = sum(distvals)/len(distvals)
        # show the frame to our screen
        cv2.putText(image, "Center: " + str(center), (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50, 170, 50), 2)
        cv2.putText(image, "Radius: " + str(int(radius)), (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50, 170, 50),
                    2)
        cv2.putText(image, "Distance: " + str(round(average, 2)), (50, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.75,
                    (50, 170, 50), 2)

    cv2.imshow("Frame", image)  # Show the Video feed
