# coding:utf-8

# import the necessary packages
# from collections import deque
from imutils.video import VideoStream
import numpy as np
import argparse
import cv2
import imutils
import time
import serial
import threading

import pygame
from pygame.locals import *

import sys

IMAGE_WIDTH = 500
DETECT_TH   = 10

g_radius = 0
g_center = (0, 0)
lock = threading.Lock()


def stop(ser):
    ser.write('m0:0\n'.encode())
    ser.write('m1:0\n'.encode())

def turn_right(ser):
    ser.write('m0:-100\n'.encode())
    ser.write('m1:100\n'.encode())

def turn_left(ser):
    ser.write('m0:100\n'.encode())
    ser.write('m1:-100\n'.encode())

def forward(ser):
    ser.write('m0:100\n'.encode())
    ser.write('m1:100\n'.encode())

def backward(ser):
    ser.write('m0:-100\n'.encode())
    ser.write('m1:-100\n'.encode())

def tracking():
    global g_radius
    global g_center
    global lock
    pygame.init()
    screen = pygame.display.set_mode((400, 330))    # 画面を作成
    pygame.display.set_caption("keyboard event")    # タイトルを作成

    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=None)
    time.sleep(2)

    state = 0x00

    while True:
        pygame.event.pump()
        key = pygame.key.get_pressed()

        with lock:
            radius = g_radius
            center = g_center
        
        if key[K_RIGHT]:
            print("right")
            state = 0x01
            turn_right(ser)

        elif key[K_LEFT]:
            print("left")
            state = 0x01
            turn_left(ser)

        elif key[K_UP]:
            print("up")
            state = 0x01
            forward(ser)
        elif key[K_DOWN]:
            print("down")
            state = 0x01
            backward(ser)
        else:
            if state == 0x01:
                stop(ser)

        if key[K_s]:
            state = 0xff
            print("stop")
            stop(ser)

        if key[K_a]:
            state = 0x02
            print("auto")
        if key[K_q]:
            print("q")
            break
        
        
        if state == 0x02:
            if radius > DETECT_TH:
                print("red object is detected.")
                x = center[0]
                diff = x - IMAGE_WIDTH/2
            
                print(diff)
                time.sleep(0.1)
                if diff > 10:
                    turn_right(ser)
                    time.sleep(0.04)
                    stop(ser)
                elif diff < -10:
                    turn_left(ser)
                    time.sleep(0.04)
                    stop(ser)
                else:
                    forward(ser)
                    time.sleep(0.5)
                    stop(ser)
            else:
                stop(ser)

def detect():
    global g_radius
    global g_center
    global lock

    # construct the argument parse and parse the arguments
    ap = argparse.ArgumentParser()
    ap.add_argument("-v", "--video",
        help="path to the (optional) video file")
    ap.add_argument("-b", "--buffer", type=int, default=64,
        help="max buffer size")
    args = vars(ap.parse_args())

    # define the lower and upper boundaries of the "green"
    # ball in the HSV color space, then initialize the
    # list of tracked points
    redLower = (0, 100, 100)
    redUpper = (10, 255, 255)

    redLower2 = (170, 100, 100)
    redUpper2 = (180, 255, 255)
    
    radius = 0

    # if a video path was not supplied, grab the reference
    # to the webcam
    if not args.get("video", False):
        vs = VideoStream(src=0).start()

    # otherwise, grab a reference to the video file
    else:
        vs = cv2.VideoCapture(args["video"])

    # allow the camera or video file to warm up
    time.sleep(1.0)

    # keep looping
    while True:
        # grab the current frame
        frame = vs.read()

        # handle the frame from VideoCapture or VideoStream
        frame = frame[1] if args.get("video", False) else frame

        # if we are viewing a video and we did not grab a frame,
        # then we have reached the end of the video
        if frame is None:
            break

        # resize the frame, blur it, and convert it to the HSV
        # color space
        frame = imutils.resize(frame, width=IMAGE_WIDTH)
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # construct a mask for the color "green", then perform
        # a series of dilations and erosions to remove any small
        # blobs left in the mask

        mask1 = cv2.inRange(hsv, redLower, redUpper)
        mask2 = cv2.inRange(hsv, redLower2, redUpper2)
        mask = cv2.bitwise_or(mask1, mask2)
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
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            # only proceed if the radius meets a minimum size
            if radius > DETECT_TH:    
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                cv2.circle(frame, (int(x), int(y)), int(radius),
                    (0, 255, 255), 2)
                cv2.circle(frame, center, 5, (0, 0, 255), -1)
        else:
            radius = 0

        with lock:
            g_radius = radius
            g_center = center

        # show the frame to our screen
        cv2.imshow("Frame", frame)
        key = cv2.waitKey(1)&0xff

        # if the 'q' key is pressed, stop the loop
        if key == ord('q'):
            break


    # if we are not using a video file, stop the camera video stream
    if not args.get("video", False):
        vs.stop()

    # otherwise, release the camera
    else:
        vs.release()

    # close all windows
    cv2.destroyAllWindows()



if __name__ == "__main__":
    thread_1 = threading.Thread(target=detect)
    thread_2 = threading.Thread(target=tracking)

    thread_1.start()
    thread_2.start()

