import cv2 as cv
import math as m
import mouse
import numpy as np
import sys
import warnings
from HandTracking import HandDetector
from Utilities import WINDOW_WIDTH_RANGE, WINDOW_HEIGHT_RANGE

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
        lms = self.detector.find_landmarks(img, draw_lms=(0, 5, 4, 8, 20))
        return img, lms

    def get_points(self, lms, points=(0, 5, 4, 8, 20)):
        result = np.zeros((5, 3))
        for idx, point in enumerate(points):
            result[idx][0], result[idx][1] = lms[point][1], lms[point][2]
        return result

    def get_dists(self, lms, points, root=0):
        rootx, rooty = lms[root][1], lms[root][2]
        result = np.zeros(5)
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

                if dists[2] > dist_thresh and dists[3] > dist_thresh and dists[4] > dist_thresh and self.tracking_mode != 0:
                    proposal = 0
                elif dists[2] < dist_thresh and dists[3] > dist_thresh and dists[4] < dist_thresh and self.tracking_mode != 1:
                    proposal = 1
                elif dists[2] > dist_thresh and dists[3] > dist_thresh and dists[4] < dist_thresh and self.tracking_mode != 2:
                    proposal = 2
                elif dists[2] > dist_thresh and dists[3] < dist_thresh and dists[4] > dist_thresh and self.tracking_mode != 3:
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
                if self.tracking_mode == 1:
                    if dists[3] < dist_thresh and not left_pressed:
                        mouse.press('left')
                    elif dists[3] > dist_thresh and left_pressed:
                        mouse.release('left')
            cv.imshow('Galaxy Rodent', img)
            self.check_quit()


if __name__ == '__main__':
    controller = MouseController(cam=1)
    controller.track(mode_thresh=15)
