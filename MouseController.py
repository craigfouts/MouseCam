import cv2 as cv
import math as m
import mouse
import numpy as np
import pygetwindow as gw
from HandTracking import HandDetector


detector = HandDetector(detect_conf=0.75)
cam_width, cam_height = 640, 480
try:
    cap = cv.VideoCapture(1)
    detector.find_hands(cap.read()[1])
except:
    cap = cv.VideoCapture(0)
cap.set(3, cam_width)
cap.set(4, cam_height)
