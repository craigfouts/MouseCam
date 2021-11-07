import ctypes

WINDOW_WIDTH = ctypes.windll.user32.GetSystemMetrics(78)
WINDOW_HEIGHT = ctypes.windll.user32.GetSystemMetrics(79)
WIDTH_BUFFER, HEIGHT_BUFFER = 0.5 * WINDOW_WIDTH, WINDOW_HEIGHT
WINDOW_WIDTH_RANGE = (-WIDTH_BUFFER, WINDOW_WIDTH)
WINDOW_HEIGHT_RANGE = (-2 * HEIGHT_BUFFER, WINDOW_HEIGHT + HEIGHT_BUFFER)

import cv2
import mediapipe as mp


class HandDetector:
    def __init__(self, max_hands=2, detect_conf=0.5, track_conf=0.5):
        self.max_hands = max_hands
        self.detect_conf = detect_conf
        self.track_conf = track_conf
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(tuple(locals().values())[1:])
        self.mp_draw = mp.solutions.drawing_utils

    def find_hands(self, img, draw=True):
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        self.results = self.hands.process(img_rgb)
        if self.results.multi_hand_landmarks:
            for hand_lms in self.results.multi_hand_landmarks:
                if draw:
                    connections = self.mp_hands.HAND_CONNECTIONS
                    self.mp_draw.draw_landmarks(img, hand_lms, connections)
        return img

    def find_landmarks(self, img, hand_idx=0, draw_lms=()):
        lms = []
        if self.results.multi_hand_landmarks:
            hand = self.results.multi_hand_landmarks[hand_idx]
            for id, lm in enumerate(hand.landmark):
                h, w = img.shape[:2]
                x, y = int(lm.x * w), int(lm.y * h)
                lms.append((id, x, y))
                if id in draw_lms:
                    cv2.circle(img, (x, y), 10, (255, 0, 0), cv2.FILLED)
        return lms

import cv2 as cv
import math as m
import mouse
import numpy as np
import sys
import warnings

warnings.simplefilter('ignore')


class MouseController:
    def __init__(self, cam=0, cam_width=640, cam_height=480, detect_conf=0.75):
        self.detector = HandDetector(detect_conf=detect_conf)
        try:
            self.cap = cv.VideoCapture(cam)
            self.detector.find_hands(self.cap.read()[1])
        except:
            self.cap = cv.VideoCapture(0)
        self.cap.set(3, cam_width)
        self.cap.set(4, cam_height)
        self.tracking_mode = 0
        self.position = np.zeros(2)

    def get_snapshot(self):
        img = self.cap.read()[1]
        img = self.detector.find_hands(img)
        lms = self.detector.find_landmarks(img, draw_lms=(0, 5, 4, 8, 12, 20))
        return img, lms

    def get_points(self, lms, points=(0, 5, 4, 8, 12, 20)):
        result = np.zeros((len(points), 3))
        for idx, point in enumerate(points):
            result[idx][0], result[idx][1] = lms[point][1], lms[point][2]
        return result

    def get_dists(self, lms, points, root=0):
        rootx, rooty = lms[root][1], lms[root][2]
        result = np.zeros(len(points))
        for idx, point in enumerate(points):
            distx = np.abs(point[0] - rootx)
            disty = np.abs(point[1] - rooty)
            result[idx] = m.hypot(distx, disty)
        return result

    def get_pos(self, point, cam_range=(50, 600)):
        newx = np.interp(point[0], cam_range, WINDOW_WIDTH_RANGE)
        newy = np.interp(point[1], cam_range, WINDOW_HEIGHT_RANGE)
        return np.array([newx, newy])

    def check_quit(self, target=113, delay=10):
        key = cv.waitKey(delay)
        if key == target:
            cv.destroyAllWindows()
            sys.exit()

    def track(self, move_thresh=15, dist_thresh=75, mode_thresh=25):
        right_pressed = False
        scrolling = False
        candidate, epoch = 0, 0
        while True:
            img, lms = self.get_snapshot()
            if len(lms) > 0:
                points = self.get_points(lms)
                dists = self.get_dists(lms, points, root=5)
                position = self.get_pos(points[0])
                if np.sum(np.abs(position - self.position)) > move_thresh:
                    mouse.move(position[0], position[1], duration=0.02)
                self.position = position

                if dists[2] > dist_thresh and dists[3] > dist_thresh and dists[4] > dist_thresh and dists[5] > dist_thresh and self.tracking_mode != 0:
                    proposal = 0
                elif dists[2] < dist_thresh and dists[3] > dist_thresh and dists[4] < dist_thresh and dists[5] < dist_thresh and self.tracking_mode != 1:
                    proposal = 1
                elif dists[2] > dist_thresh and dists[3] > dist_thresh and dists[4] > dist_thresh and dists[5] < dist_thresh and self.tracking_mode != 2:
                    proposal = 2
                elif dists[2] > dist_thresh and dists[3] < dist_thresh and dists[4] < dist_thresh and dists[5] > dist_thresh and self.tracking_mode != 3:
                    proposal = 3
                else:
                    proposal = self.tracking_mode
                
                if proposal == candidate:
                    epoch += 1
                else:
                    candidate = proposal
                    epoch = 0

                if epoch > mode_thresh:
                    if candidate != self.tracking_mode:
                        print('Mode:', candidate)
                    self.tracking_mode, epoch = candidate, 0

                left_pressed = mouse.is_pressed('left')
                if self.tracking_mode == 2:
                    if dists[3] < dist_thresh and dists[4] < dist_thresh and not left_pressed:
                        mouse.press('left')
                    elif dists[3] > dist_thresh and dists[4] < dist_thresh and not right_pressed:
                        mouse.click('right')
                        right_pressed = True
                    elif dists[2] < dist_thresh and not scrolling:
                        if position[1] > WINDOW_HEIGHT / 2.0 + 50:
                            mouse.wheel(-1)
                        elif position[1] < WINDOW_HEIGHT / 2.0 - 50:
                            mouse.wheel(1)
                    elif dists[3] > dist_thresh and dists[4] > dist_thresh and left_pressed:
                        if left_pressed:
                            mouse.release('left')
                        if right_pressed:
                            right_pressed = False
                        
            cv.imshow('Galaxy Rodent', img)
            self.check_quit()


if __name__ == '__main__':
    controller = MouseController(cam=1)
    controller.track(mode_thresh=15)
