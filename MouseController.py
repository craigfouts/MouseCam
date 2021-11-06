import cv2 as cv
import math as m
import mouse
import numpy as np
import pygetwindow as gw
import sys
from HandTracking import HandDetector
from win32api import GetSystemMetrics


window_width = GetSystemMetrics(0)
window_height = GetSystemMetrics(1)


detector = HandDetector(detect_conf=0.75)
cam_width, cam_height = 640, 480
try:
    cap = cv.VideoCapture(1)
    detector.find_hands(cap.read()[1])
except:
    cap = cv.VideoCapture(0)
cap.set(3, cam_width)
cap.set(4, cam_height)


if __name__ == '__main__':
    while True:
        _, img = cap.read()
        img = detector.find_hands(img)
        lms = detector.find_landmarks(img, draw_lms=(4, 8))
        if len(lms) > 0:
            # Thumb and forefinger tracking 
            x1, y1 = lms[4][1], lms[4][2]
            x2, y2 = lms[8][1], lms[8][2]
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
            length = m.hypot(x2 - x1, y2 - y1)
            if length < 100:
                oldx, oldy = mouse.get_position()
                newx = np.interp(cx, (50, 600), (window_width, 0))
                newy = np.interp(cy, (50, 600), (0, window_height))
                mouse.move(newx, newy, absolute=True, duration=0.0)
            cv.line(img, (x1, y1), (x2, y2), (255, 0, 0), 3)
            cv.circle(img, (cx, cy), 15, (255, 0, 0), cv.FILLED)
        cv.imshow('Image', img)
        key = cv.waitKey(10)
        if key == 113:
            cv.destroyAllWindows()
            sys.exit()
